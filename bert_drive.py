# -*- coding: utf-8 -*-
"""Drive Large Bert live from a steering wheel, pedals or gamepad.

This is the Pi side. It is started by bert_wheel.py on the PC over SSH, so
there is no open port and the SSH key is the password. It reads one JSON line
per input sample on stdin and writes status lines on stdout.

    in : {"fwd": -1..1, "turn": -1..1, "side": -1..1, "horn": 0|1, "look": [pan, tilt] (optional)}
         (fwd + is forward, turn + is LEFT, side + is crab-walk right)
         or {"cmd": "stand" | "relax" | "quit"}
    out: {"state": ..., "dist": cm, "load_v": V, "pi_v": V, "msg": ...}
         {"jpg": base64 JPEG} from the head camera, VIDEO_FPS times a second

Safety (on top of everything in large_bert_mcp.Bert: battery cut-offs,
idle relax, quiet hours for the horn):
  * deadman: no input line for DEADMAN_SECONDS -> stop and stand still
  * SSH drops or the PC client quits -> stdin closes -> relax, lights off
  * step length and gait speed are capped (bert_config MAX_STEP_LENGTH /
    MAX_SPEED, reverse and crab-walk at REVERSE_FRACTION of that)
  * forward is blocked while the ultrasonic sees something closer than
    MIN_OBSTACLE_CM (turning, crab-walking and reversing still work)
  * refuses to start while large_bert_mcp.py or cat_patrol.py holds the robot

Run from Code/Server on the Pi:
    sudo ~/bert-venv/bin/python bert_drive.py          # real robot (normally via bert_wheel.py)
    python3 bert_drive.py --sim                        # pretend robot, for testing
"""
import argparse
import json
import os
import queue
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)

# The Freenove code prints to stdout, which is our status channel. Keep a private
# copy of the real stdout for status lines and point fd 1 at stderr.
_OUT = os.fdopen(os.dup(1), "w", buffering=1)
os.dup2(2, 1)

DEADMAN_SECONDS = 0.5     # no input for this long: stop
REVERSE_FRACTION = 0.6    # reverse and crab-walk are capped at this share of full speed
DEADZONE = 0.08           # inputs smaller than this count as zero
MIN_GAIT_SPEED = 5        # gait speed at a light touch of the pedal (Freenove 2..10)
VIDEO_FPS = 12            # head camera frames sent to the PC per second
VIDEO_SIZE = (640, 480)
VIDEO_QUALITY = 60        # JPEG quality; about 30 KB a frame
ABORT_CHECK_SECONDS = 0.1 # how often a step in progress re-checks the input and the ultrasonic
STATUS_SECONDS = 0.5
BATTERY_EVERY_CYCLES = 4


def log(message):
    sys.stderr.write(time.strftime("%H:%M:%S ") + "[drive] " + message + "\n")
    sys.stderr.flush()


_OUTBOX = queue.Queue(maxsize=8)


def _writer():
    """The only thread that writes to the PC. If the PC stops reading, this one blocks,
    never the driving loop, so the deadman and mid-step stops keep working."""
    while True:
        line = _OUTBOX.get()
        try:
            _OUT.write(line)
            _OUT.flush()
        except (BrokenPipeError, OSError, ValueError):
            pass


threading.Thread(target=_writer, daemon=True).start()


def send(_droppable=False, **fields):
    """Queue a line for the PC. Video frames are dropped when the link is behind;
    status lines push out the oldest queued line instead of waiting."""
    line = json.dumps(fields) + "\n"
    if _droppable and _OUTBOX.qsize() > 2:
        return
    while True:
        try:
            _OUTBOX.put_nowait(line)
            return
        except queue.Full:
            try:
                _OUTBOX.get_nowait()
            except queue.Empty:
                pass


def stream_video(bert, stop):
    """Send head-camera frames to the PC until stop is set. Never touches the legs."""
    import base64
    import cv2
    try:
        camera = bert.get_camera()
    except Exception as exc:
        log("No camera: %s" % exc)
        send(msg="No video: %s" % exc)
        return
    stamp = 0.0
    while not stop.is_set():
        start = time.time()
        frame = camera.read(newer_than=stamp, timeout=1.0)
        if frame is None:
            continue
        stamp = time.time()
        if (frame.shape[1], frame.shape[0]) != VIDEO_SIZE:
            frame = cv2.resize(frame, VIDEO_SIZE)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), VIDEO_QUALITY])
        if ok:
            send(_droppable=True, jpg=base64.b64encode(buf.tobytes()).decode("ascii"))
        time.sleep(max(0.0, 1.0 / VIDEO_FPS - (time.time() - start)))


def other_owners():
    """PIDs of large_bert_mcp.py / cat_patrol.py / main.py that would hold the servos and camera."""
    found = []
    if not os.path.isdir("/proc"):
        return found
    me = {os.getpid(), os.getppid()}
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in me:
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        if len(args) > 1 and b"python" in os.path.basename(args[0]) and b"--sim" not in args:
            for a in args[1:]:
                name = os.path.basename(a)
                if name in (b"large_bert_mcp.py", b"cat_patrol.py", b"main.py"):
                    found.append("%s (%s)" % (entry, name.decode()))
    return found


def clamp(value, low, high):
    return max(low, min(high, value))


def shape(value):
    """Deadzone, then a gentle curve so small inputs give fine control."""
    value = clamp(float(value), -1.0, 1.0)
    if abs(value) < DEADZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    value = (abs(value) - DEADZONE) / (1.0 - DEADZONE)
    return sign * value ** 1.5


class Abort(Exception):
    """Raised inside a gait cycle to end it mid-stride."""


class SleepShim:
    """Stands in for the time module inside Freenove's control.py. run_gait sleeps
    10 ms between leg positions; each sleep in the driving thread asks the driver
    whether to stop, so a 1-3 s gait cycle can end as soon as the pedal lifts."""
    def __init__(self, real, driver):
        self._real = real
        self._driver = driver

    def __getattr__(self, name):
        return getattr(self._real, name)

    def sleep(self, seconds):
        self._real.sleep(seconds)
        d = self._driver
        if d.step_thread == threading.get_ident():
            reason = d.should_abort()
            if reason:
                raise Abort(reason)


def sign(x):
    return (x > 0) - (x < 0)


class Driver:
    def __init__(self, bert, B):
        self.bert = bert
        self.B = B
        self.lock = threading.Lock()
        self.cmd = {"fwd": 0.0, "turn": 0.0, "side": 0.0, "horn": 0}
        self.look_to = None          # newest head position asked for, not yet applied
        self.last_look = None
        self.last_rx = 0.0
        self.eof = False
        self.pending = []           # one-off commands: stand, relax
        self.moving = False
        self.blocked = False
        self.state = "ready"
        self.cycles = 0
        self.step_thread = None      # set while a gait cycle runs in the main thread
        self.step_dir = None
        self.last_check = 0.0
        self.last_status = 0.0
        self.dist = None
        control = sys.modules.get("control")   # loaded by the real robot only
        if control is not None:                # make control.py's gait interruptible
            control.time = SleepShim(time, self)

    # ---- input thread ------------------------------------------------------
    def reader(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                self.last_rx = time.time()
                if "cmd" in msg:
                    self.pending.append(str(msg["cmd"]))
                else:
                    for key in ("fwd", "turn", "side", "horn"):
                        if key in msg:
                            self.cmd[key] = msg[key]
                    if "look" in msg and msg["look"] != self.last_look:
                        self.look_to = self.last_look = msg["look"]
        self.eof = True

    def take(self):
        with self.lock:
            cmd = dict(self.cmd)
            age = time.time() - self.last_rx
            pending, self.pending = self.pending, []
        return cmd, age, pending

    def apply_look(self):
        with self.lock:
            target, self.look_to = self.look_to, None
        if target is not None:
            try:
                self.bert.look(float(target[0]), float(target[1]))
            except Exception as exc:
                log("Look failed: %s" % exc)

    def should_abort(self):
        """Called about every 10 ms during a gait cycle. Returns a reason to stop now, or None."""
        now = time.time()
        if now - self.last_check < ABORT_CHECK_SECONDS:
            return None
        self.last_check = now
        self.apply_look()            # the head can move while he walks
        cmd, age, _ = self.take_peek()
        if age > DEADMAN_SECONDS:
            return "no input (deadman)"
        f, t, s = shape(cmd["fwd"]), shape(cmd["turn"]), shape(cmd["side"])
        if f == 0 and t == 0 and s == 0:
            return "standing"
        if (sign(f), sign(t), sign(s)) != self.step_dir:
            return "changing direction"
        if self.step_dir[0] > 0:
            self.dist = self.bert.hw.distance()
            if self.dist is not None and 0 < self.dist < self.B.MIN_OBSTACLE_CM:
                self.blocked = True
                return "blocked: obstacle %.0f cm" % self.dist
        if now - self.last_status > STATUS_SECONDS:
            self.report()
        return None

    def take_peek(self):
        with self.lock:
            return dict(self.cmd), time.time() - self.last_rx, None

    def report(self):
        self.last_status = time.time()
        load, pi = self.bert.last_volts
        send(state=self.state, dist=None if self.dist is None else round(self.dist), load_v=load, pi_v=pi,
             low=self.bert.battery_low)

    # ---- helpers -----------------------------------------------------------
    def halt(self, state):
        if self.moving and not self.bert.battery_low:
            self.bert.hw.stand()
        self.moving = False
        self.state = state

    def horn(self):
        from large_bert_mcp import in_quiet_hours
        if in_quiet_hours():
            return
        def beep():
            try:
                self.bert.hw.buzzer(True)
                time.sleep(0.15)
            finally:
                self.bert.hw.buzzer(False)
        threading.Thread(target=beep, daemon=True).start()

    # ---- main loop ---------------------------------------------------------
    def run(self):
        B = self.B
        threading.Thread(target=self.reader, daemon=True).start()
        horn_was = 0
        while not self.eof:
            cmd, age, pending = self.take()
            for c in pending:
                if c == "quit":
                    return
                if c == "relax":
                    self.moving = False
                    self.bert._relax()
                    self.state = "relaxed"
                elif c == "stand":
                    try:
                        self.bert.require_can_move()
                        self.bert.hw.stand()
                        self.bert.relaxed = False
                        self.state = "standing"
                    except Exception as exc:
                        self.state = "refused"
                        send(state="refused", msg=str(exc))
                self.bert.touch()

            self.apply_look()
            fwd, turn, side = shape(cmd["fwd"]), shape(cmd["turn"]), shape(cmd["side"])
            if age > DEADMAN_SECONDS:
                if self.moving:
                    log("No input for %.1fs: stopping." % age)
                    self.halt("no input (deadman)")
                fwd = turn = side = 0.0
                cmd["horn"] = 0

            if cmd.get("horn") and not horn_was:
                self.horn()
            horn_was = cmd.get("horn", 0)

            if fwd > 0:
                self.dist = self.bert.hw.distance()
                self.blocked = self.dist is not None and 0 < self.dist < B.MIN_OBSTACLE_CM
                if self.blocked:
                    fwd = 0.0
            else:
                self.blocked = False

            if fwd == 0 and turn == 0 and side == 0:
                if self.moving:
                    self.halt("blocked: obstacle %.0f cm" % self.dist if self.blocked else "standing")
                elif self.blocked:
                    self.state = "blocked: obstacle %.0f cm" % self.dist
                time.sleep(0.03)
            else:
                try:
                    self.bert.require_can_move()
                except Exception as exc:
                    self.halt("refused")
                    send(state="refused", msg=str(exc))
                    time.sleep(0.5)
                    continue
                if self.cycles % BATTERY_EVERY_CYCLES == 0 and not self.bert.check_battery(force=True):
                    self.moving = False
                    self.state = "battery low"
                    continue
                cap = REVERSE_FRACTION if fwd < 0 else 1.0
                forward = int(round(fwd * B.MAX_STEP_LENGTH * cap))
                sideways = int(round(side * B.MAX_STEP_LENGTH * REVERSE_FRACTION))
                turn_val = int(round(turn * 10))
                effort = max(abs(fwd), abs(turn), abs(side))
                speed = int(round(MIN_GAIT_SPEED + effort * (B.MAX_SPEED - MIN_GAIT_SPEED)))
                if fwd < 0 or (side and not fwd):
                    speed = min(speed, int(round(B.MAX_SPEED * REVERSE_FRACTION + 1)))
                speed = int(clamp(speed, 2, B.MAX_SPEED))
                if forward == 0 and sideways == 0 and turn_val == 0:
                    time.sleep(0.03)
                    continue
                self.moving = True
                self.bert.relaxed = False
                self.state = ("blocked ahead, " if self.blocked else "") + "driving"
                self.step_dir = (sign(forward), sign(turn_val), sign(sideways))
                self.step_thread = threading.get_ident()
                try:
                    self.bert.hw.walk(forward, sideways, turn_val, speed)   # one gait cycle, 1-3 s
                except Abort as why:
                    self.step_thread = None
                    reason = str(why)
                    if reason.startswith("no input"):
                        log("No input for over %.1fs: stopped mid-step." % DEADMAN_SECONDS)
                    self.halt(reason)            # feet back under him before the next step
                finally:
                    self.step_thread = None
                self.cycles += 1
                self.bert.touch()

            if time.time() - self.last_status > STATUS_SECONDS:
                if not self.moving or self.dist is None:
                    self.dist = self.bert.hw.distance()
                self.report()


def main():
    parser = argparse.ArgumentParser(description="Live drive listener for Large Bert (stdin/stdout, run over SSH).")
    parser.add_argument("--sim", action="store_true", help="pretend robot, logs instead of moving")
    parser.add_argument("--no-video", action="store_true", help="don't stream the head camera")
    args = parser.parse_args()

    if not args.sim:
        owners = other_owners()
        if owners:
            msg = ("Bert is busy: %s holds the servos and camera. Stop it first "
                   "(for the MCP server, close the Claude session using it)." % ", ".join(owners))
            log(msg)
            send(state="busy", msg=msg)
            sys.exit(3)

    import bert_config as B
    import large_bert_mcp as M          # reuse Bert: battery cut-offs, idle relax, clean shutdown
    bert = M.Bert(sim=args.sim)

    import atexit
    import signal
    atexit.register(bert.close)

    def on_signal(signum, _frame):
        bert.close()
        os._exit(0)
    for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(sig, on_signal)

    try:
        bert.hw.lights((0, 0, 60))
    except Exception:
        pass
    send(state="ready", msg="Large Bert drive ready (%s)." % ("simulated" if args.sim else "real robot"),
         max_step=B.MAX_STEP_LENGTH, max_speed=B.MAX_SPEED, obstacle_cm=B.MIN_OBSTACLE_CM,
         load_v=bert.last_volts[0], pi_v=bert.last_volts[1])
    log("Ready (%s)." % ("simulated" if args.sim else "real robot"))
    video_stop = threading.Event()
    if not args.no_video:
        threading.Thread(target=stream_video, args=(bert, video_stop), daemon=True).start()
    try:
        Driver(bert, B).run()
    finally:
        video_stop.set()
        time.sleep(0.2)
        bert.close()


if __name__ == "__main__":
    main()
