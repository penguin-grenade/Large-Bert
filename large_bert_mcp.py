# -*- coding: utf-8 -*-
"""Large Bert's MCP server: lets Claude drive the Freenove Big Hexapod.

Tools: status, walk, turn, stop, stand_up, relax, look, snapshot, who_is_there,
body_tilt, lights, beep, patrol_start, patrol_stop, patrol_status, listen, lidar_scan, cruise, orientation.

Safety (settings in bert_config.py, battery cut-offs from patrol_config.py):
  * every walk/turn is capped in steps and seconds, and checks the stop flag,
    the battery and (going forward) the ultrasonic sensor before each step
  * stop() interrupts a walk between gait cycles; pressing Esc in Claude does too
  * the legs relax by themselves after IDLE_RELAX_SECONDS without a command
  * below MIN_LOAD_BATTERY_V / MIN_PI_BATTERY_V it relaxes and refuses to move
  * when the connection drops or the server exits, the robot relaxes, lights off
  * saying "Bert, stop" (or just "stop") near the ReSpeaker mic stops him on the Pi at once,
    without waiting on Claude (bert_ears.py; turn off with --no-ears)

Run from Code/Server on the Pi (stop main.py and cat_patrol.py first):
    python3 large_bert_mcp.py            # stdio, for Claude over SSH (default)
    python3 large_bert_mcp.py --http     # HTTP on the LAN, needs the bearer token
    python3 large_bert_mcp.py --sim      # fake robot, to try it on any computer
    python3 large_bert_mcp.py --check    # quick hardware self-test, no Claude needed
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)                      # Freenove code reads point.txt etc. from here
sys.path.insert(0, HERE)


PROTOCOL_OUT = None


def _protect_stdout():
    """In stdio mode stdout carries the MCP protocol, but the Freenove code prints to
    stdout. Point file descriptor 1 (and print) at stderr and keep a private copy of
    the real stdout for MCP messages only."""
    global PROTOCOL_OUT
    PROTOCOL_OUT = os.fdopen(os.dup(1), "wb", buffering=0)
    os.dup2(2, 1)


ARGS = None
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MCP server for Large Bert the hexapod.")
    parser.add_argument("--http", action="store_true", help="serve MCP over HTTP instead of stdio")
    parser.add_argument("--port", type=int, help="HTTP port (default from bert_config.py)")
    parser.add_argument("--sim", action="store_true", help="fake robot and camera, for testing anywhere")
    parser.add_argument("--no-move", action="store_true", help="refuse all walking/turning (look, lights and status only)")
    parser.add_argument("--check", action="store_true", help="run a quick hardware self-test and exit")
    parser.add_argument("--no-ears", action="store_true", help="no microphone: no voice stop, no listen tool")
    ARGS = parser.parse_args()
    if not ARGS.http and not ARGS.check:
        _protect_stdout()

import atexit
import copy
import math
import secrets
import signal
import threading
import time
from collections import deque

import bert_config as B

try:
    import patrol_config as C
except ImportError:                 # patrol files not copied: use the same fixed values
    class C:
        MIN_LOAD_BATTERY_V = 6.4
        MIN_PI_BATTERY_V = 6.4
        QUIET_HOURS = (22, 7)
        TURN_SIGN = 1
        PAN_SIGN = 1
        DEG_PER_TURN_CYCLE = 31
        HEAD_PAN_CHANNEL = 1
        HEAD_TILT_CHANNEL = 0
        PAN_CENTER = 108
        TILT_CENTER = 105
        STEP_LENGTH = 30
        CAMERA_SIZE = (640, 480)
        MODE = "sentry"

MIN_LOAD_V = getattr(C, "MIN_LOAD_BATTERY_V", 6.4)
MIN_PI_V = getattr(C, "MIN_PI_BATTERY_V", 6.4)


def log(message):
    sys.stderr.write(time.strftime("%H:%M:%S ") + message + "\n")
    sys.stderr.flush()


def clamp(value, low, high):
    return max(low, min(high, value))


GAITS = ("tripod", "wave")


def check_gait(gait):
    gait = (gait or "tripod").strip().lower()
    if gait not in GAITS:
        raise Refused("unknown gait %r: use %s" % (gait, " or ".join(GAITS)))
    return gait


def gait_scale(gait):
    """How far one cycle of this gait goes compared with a tripod cycle of the same step length."""
    if gait == "wave":
        return getattr(B, "WAVE_STRIDE_FACTOR", 5.0 / 3.0) / 2.0   # tripod moves 2 step lengths a cycle
    return 1.0


def in_quiet_hours():
    hours = getattr(C, "QUIET_HOURS", None)
    if not hours:
        return False
    start, end = hours
    hour = time.localtime().tm_hour
    return start <= hour < end if start < end else (hour >= start or hour < end)


COLOURS = {
    "off": (0, 0, 0), "black": (0, 0, 0), "white": (255, 255, 255), "red": (255, 0, 0),
    "green": (0, 255, 0), "blue": (0, 0, 255), "yellow": (255, 200, 0), "amber": (255, 120, 0),
    "orange": (255, 80, 0), "purple": (160, 0, 255), "pink": (255, 60, 150), "cyan": (0, 255, 255),
}


def parse_colour(colour):
    text = colour.strip().lower()
    if text in COLOURS:
        return COLOURS[text]
    text = text.lstrip("#")
    if len(text) == 6:
        try:
            return tuple(int(text[i:i + 2], 16) for i in (0, 2, 4))
        except ValueError:
            pass
    raise ValueError("Unknown colour %r. Use a name (%s) or hex like #ff8800." % (colour, ", ".join(sorted(COLOURS))))


# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------
class Hardware:
    """The real robot. Reuses cat_patrol.Robot so the patrol and Claude share one set of objects."""
    def __init__(self):
        from cat_patrol import Robot
        self.robot = Robot()
        self.control = self.robot.control
        self._home_points = copy.deepcopy(self.control.body_points)
        self._home_height = self.control.body_height
        self.lift = getattr(B, "STEP_LIFT_MM", 40)

    def walk(self, forward, sideways, turn, speed, gait="tripod"):
        if gait == "wave":                   # experimental: one leg in the air at a time (bert_gait.py)
            import bert_gait
            self.robot.power_on()
            bert_gait.wave_cycle(self.control, sideways, forward, speed, clamp(turn, -10, 10) * C.TURN_SIGN,
                                 lift=self.lift)
        else:
            self.robot.walk(forward=forward, sideways=sideways, turn=turn, speed=speed, lift=self.lift)

    def set_height(self, height_mm, lift_mm):
        """Feet height_mm below the body (taller = more ground clearance). Kept for later stands and walks."""
        self.lift = lift_mm
        start = -self._home_height
        self.stand()                         # stand at the current height first (wakes the servos)
        steps = max(1, int(round(abs(height_mm - start) / 5.0)))
        for i in range(1, steps + 1):        # rise or sink about 5 mm at a time, not in one jump
            h = start + (height_mm - start) * i / steps
            for point in self._home_points:
                point[2] = -h
            self._home_height = -h
            self.stand()
            time.sleep(0.1)                  # stand() returns at once; give the servos time to follow

    def stand(self):
        self.control.body_points = copy.deepcopy(self._home_points)
        self.control.body_height = self._home_height
        self.robot.stand()

    def relax(self):
        self.robot.buzzer.set_state(False)
        self.robot.control.relax(True)
        self.robot.power_off()

    def look(self, pan, tilt):
        self.robot.look(pan, tilt)

    def distance(self):
        try:
            return self.robot.distance()
        except Exception:
            return None

    def battery(self):
        load, pi = self.robot.adc.read_battery_voltage()
        return float(load), float(pi)

    def lights(self, rgb):
        if getattr(self.robot.led, "is_support_led_function", False):
            self.robot.led.led_index(0x7f, *rgb)
            return True
        return False

    def buzzer(self, on):
        self.robot.buzzer.set_state(on)

    def body_tilt(self, roll, pitch, yaw):
        self.robot.power_on()
        points = self.control.calculate_posture_balance(roll, pitch, yaw)
        self.control.transform_coordinates(points)
        self.control.set_leg_angles()


class FakeHardware:
    """Pretend robot for --sim: logs instead of moving."""
    def __init__(self):
        self.robot = None
        self.volts = (7.6, 7.6)

    def walk(self, forward, sideways, turn, speed, gait="tripod"):
        log("[sim] %s gait cycle forward=%s sideways=%s turn=%s speed=%s" % (gait, forward, sideways, turn, speed))
        time.sleep(0.2)

    def stand(self):
        log("[sim] stand")

    def relax(self):
        log("[sim] relax")

    def look(self, pan, tilt):
        log("[sim] head pan=%s tilt=%s" % (pan, tilt))

    def distance(self):
        return 120.0

    def battery(self):
        return self.volts

    def lights(self, rgb):
        log("[sim] lights %s" % (rgb,))
        return True

    def buzzer(self, on):
        pass

    def body_tilt(self, roll, pitch, yaw):
        log("[sim] body tilt %s %s %s" % (roll, pitch, yaw))

    def set_height(self, height_mm, lift_mm):
        log("[sim] body height %s mm, foot lift %s mm" % (height_mm, lift_mm))


class LockedHardware:
    """Serialises every hardware call (I2C and servos don't like two threads at once)."""
    def __init__(self, hw):
        self._hw = hw
        self.lock = threading.RLock()

    def __getattr__(self, name):
        attr = getattr(self._hw, name)
        if not callable(attr):
            return attr

        def call(*args, **kwargs):
            with self.lock:
                return attr(*args, **kwargs)
        return call


class LockedRobot:
    """Same idea for the cat_patrol.Robot object the patrol thread uses."""
    def __init__(self, robot, lock):
        self._robot = robot
        self._lock = lock

    def __getattr__(self, name):
        attr = getattr(self._robot, name)
        if not callable(attr):
            return attr

        def call(*args, **kwargs):
            with self._lock:
                return attr(*args, **kwargs)
        return call


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class FakeCamera:
    def read(self, newer_than=0.0, timeout=2.0):
        import numpy as np
        w, h = C.CAMERA_SIZE
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:, :, 1] = np.linspace(40, 160, w, dtype=np.uint8)[None, :]
        frame[h // 2:, :, :] = (60, 110, 70)
        try:
            import cv2
            cv2.putText(frame, "Large Bert (sim) " + time.strftime("%H:%M:%S"), (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        except ImportError:
            pass
        return frame

    def close(self):
        pass


def encode_jpeg(frame, quality=80):
    import cv2
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return buf.tobytes()


# ---------------------------------------------------------------------------
# Bert: state, safety and the actual actions
# ---------------------------------------------------------------------------
class Refused(Exception):
    """A safety rule said no. The message is shown to Claude."""


class Bert:
    def __init__(self, sim=False, no_move=False, use_ears=None):
        self.sim = sim
        if use_ears is None:
            use_ears = not sim               # a simulated Bert leaves the real mic alone
        self.no_move = no_move
        self.hw = LockedHardware(FakeHardware() if sim else Hardware())
        self.stop_event = threading.Event()
        self.last_activity = time.time()
        self.relaxed = True
        self.pan, self.tilt = C.PAN_CENTER, C.TILT_CENTER
        self.battery_low = False
        self.last_volts = (None, None)
        self.last_battery_read = 0.0
        self.camera = None
        self.vision = None
        self.patrol = None
        self.patrol_thread = None
        self.patrol_log = deque(maxlen=200)
        self.patrol_info = {}
        self._closed = False
        self.voice_stopped = False
        self.ears = None
        self.ears_error = None
        if use_ears:
            try:
                import bert_ears
                self.ears = bert_ears.Ears(on_stop=self._voice_stop, log=lambda m: log("[ears] " + m))
            except Exception as exc:             # no mic, no vosk, no model: everything else still works
                self.ears_error = str(exc)
                log("Voice stop is off: %s" % exc)
        else:
            self.ears_error = "turned off (--no-ears)"
        self.lidar = None
        self.lidar_error = None
        self.imu = None
        self.imu_error = None
        self.check_battery(force=True)
        threading.Thread(target=self._watchdog, daemon=True).start()

    # ---- safety ----------------------------------------------------------
    def touch(self):
        self.last_activity = time.time()

    def patrol_running(self):
        return self.patrol_thread is not None and self.patrol_thread.is_alive()

    def check_battery(self, force=False):
        """Read the battery (at most every BATTERY_CHECK_SECONDS unless forced). Returns True if OK."""
        if not force and time.time() - self.last_battery_read < B.BATTERY_CHECK_SECONDS:
            return not self.battery_low
        try:
            load, pi = self.hw.battery()
            if load < MIN_LOAD_V or pi < MIN_PI_V:   # one bad reading can be noise: read again
                time.sleep(0.2)
                load, pi = self.hw.battery()
        except Exception as exc:
            log("Battery read failed: %s" % exc)
            return not self.battery_low
        self.last_volts = (round(load, 2), round(pi, 2))
        self.last_battery_read = time.time()
        low = load < MIN_LOAD_V or pi < MIN_PI_V
        if low and not self.battery_low:
            log("Battery low (load %.2f V, pi %.2f V). Relaxing." % (load, pi))
            self.battery_low = True
            self.stop_event.set()
            self._relax()
        elif self.battery_low and load >= MIN_LOAD_V + B.BATTERY_RECOVER_MARGIN_V \
                and pi >= MIN_PI_V + B.BATTERY_RECOVER_MARGIN_V:
            log("Battery back to normal.")
            self.battery_low = False
        return not self.battery_low

    def battery_getting_low(self):
        load = self.last_volts[0]
        return bool(load is not None and load < getattr(B, "BATTERY_WARN_V", 7.0))

    def _voice_stop(self, heard):
        """Called from the ears thread when someone says "Bert, stop"."""
        self.voice_stopped = True
        self.stop_event.set()
        log("Voice stop (heard %r)." % heard)
        threading.Thread(target=self._voice_stop_finish, daemon=True).start()

    def _voice_stop_finish(self):
        try:
            self.stop()
        except Exception as exc:
            log("Voice stop: %s" % exc)

    def listen(self, seconds):
        if self.ears is None:
            raise Refused("the microphone isn't set up (%s)" % (self.ears_error or "voice off"))
        try:
            result = self.ears.listen(seconds)
        except RuntimeError as exc:
            raise Refused(str(exc))
        self.touch()
        if result["heard_stop"]:
            result["note"] = "A stop word was heard, so Large Bert stopped."
        return result

    def require_can_move(self):
        if self.no_move:
            raise Refused("Movement is turned off (server started with --no-move).")
        if self.patrol_running():
            raise Refused("The cat patrol is running and owns the legs. Call patrol_stop first.")
        if not self.check_battery(force=True):
            raise Refused("Battery too low to move (load %s V, pi %s V; cut-offs %.1f V / %.1f V). "
                          "Please charge Large Bert." % (self.last_volts[0], self.last_volts[1], MIN_LOAD_V, MIN_PI_V))

    def require_no_patrol(self):
        if self.patrol_running():
            raise Refused("The cat patrol is running. Call patrol_stop first.")

    def _relax(self):
        try:
            self.hw.buzzer(False)
            self.hw.relax()
        except Exception as exc:
            log("Relax failed: %s" % exc)
        self.relaxed = True

    def _watchdog(self):
        while not self._closed:
            time.sleep(B.WATCHDOG_SECONDS)
            if self.patrol_running():
                continue                     # the patrol checks the battery itself
            self.check_battery()
            if not self.relaxed and time.time() - self.last_activity > B.IDLE_RELAX_SECONDS:
                log("Idle for %ds, relaxing legs." % B.IDLE_RELAX_SECONDS)
                self._relax()

    # ---- movement --------------------------------------------------------
    def _walk_lidar(self):
        """The lidar for watching a walk, spun up (waits ~3 s the first time), or None if there isn't one."""
        if self.sim:
            return None
        try:
            lidar = self.get_lidar()
            lidar.ensure_spinning()
            return lidar
        except Exception as exc:
            log("Walking without the lidar: %s" % exc)
            return None

    def _walk_imu(self):
        """The IMU for watching a move, or None if there isn't one (the move goes ahead without it)."""
        if self.sim:
            return None
        try:
            return self.get_imu()
        except Exception as exc:
            log("Moving without the IMU: %s" % exc)
            return None

    def _gait(self, cycles, forward=0, sideways=0, turn=0, speed=B.DEFAULT_SPEED, follow_walls=False,
              max_seconds=None, ahead_stop_cm=None, require_lidar=False, gait="tripod"):
        """Run gait cycles with all the checks. Returns (cycles done, reason, notes).
        While walking, the lidar streams: before every cycle the newest readings are checked for
        anything in his path or brushing his sides, and with follow_walls the turn is set each cycle
        to keep him parallel to and centred between the walls."""
        self.require_can_move()
        lidar = self._walk_lidar() if (forward or sideways) else None
        imu = self._walk_imu()
        yaw_start = None
        if imu is not None:
            try:
                imu.watch()
                yaw_start = imu.mark()
            except Exception as exc:
                self.imu_error = str(exc)
                imu = None
        if require_lidar and lidar is None:
            raise Refused("this needs the lidar, which isn't working (%s)" % (self.lidar_error or "not available"))
        max_seconds = max_seconds or B.MAX_MOVE_SECONDS
        notes = {"seconds": 0.0, "volts_before": self.last_volts[0],
                 "lidar": "on" if lidar else "off (%s)" % (self.lidar_error or "not available"),
                 "imu": "on" if imu else "off (%s)" % (self.imu_error or "not available"),
                 "steered_deg": 0.0, "held_deg": 0.0}
        # degrees one unit of gait turn rotates per cycle
        per_value = getattr(C, "DEG_PER_TURN_CYCLE", 31) / 10.0 * gait_scale(gait)
        # heading hold: walking (not turning) with the IMU, steer each cycle back to the heading he started on
        hold = imu is not None and turn == 0 and (forward or sideways) and getattr(B, "HEADING_HOLD", True)
        yaw_now = None
        tip_deg = getattr(B, "TIP_STOP_DEG", 30)
        self.stop_event.clear()
        self.voice_stopped = False
        self.touch()
        self.relaxed = False
        start = time.time()
        done, reason = 0, "done"
        for step in range(cycles):
            if self.stop_event.is_set():
                reason = "stopped by voice (someone said stop)" if self.voice_stopped else "stopped by stop command"
                break
            if time.time() - start > max_seconds:
                reason = "time limit (%ds) reached" % max_seconds
                break
            if forward > 0:
                distance = self.hw.distance()
                if distance is not None and 0 < distance < B.MIN_OBSTACLE_CM:
                    reason = "obstacle %.0f cm ahead, stopped (limit %d cm)" % (distance, B.MIN_OBSTACLE_CM)
                    break
            if imu is not None:
                try:
                    yaw_now, peak, now_tilt = imu.watch()
                    if step and peak >= tip_deg:   # the first step may start from sitting, so not before it
                        reason = "stopped: body tilted %.0f deg during the last step (now %.0f deg); he may be tipping or stuck" % (
                            peak, now_tilt)
                        break
                except Exception:
                    yaw_now = None
                    notes["imu_gaps"] = notes.get("imu_gaps", 0) + 1
            if step and step % 3 == 0 and not self.check_battery(force=True):
                reason = "battery low, stopped"
                break
            cycle_turn = turn
            if lidar is not None:
                import bert_lidar
                points = lidar.latest()
                if points is None:
                    notes["lidar_gaps"] = notes.get("lidar_gaps", 0) + 1
                    if require_lidar and notes["lidar_gaps"] >= 2:
                        reason = "stopped: lost the lidar readings"
                        break
                else:
                    blocked = bert_lidar.in_the_way(points, forward, sideways, ahead_stop_cm)
                    if blocked:
                        reason = "stopped: " + blocked
                        break
                    if follow_walls and forward > 0:
                        degrees, why = bert_lidar.wall_steer(points)
                        cycle_turn = int(round(degrees / per_value))
                        notes["steered_deg"] += cycle_turn * per_value
                        if why:
                            notes["walls"] = why
            if hold and cycle_turn == 0 and yaw_now is not None:
                # target = start heading plus whatever wall-following deliberately steered
                error = yaw_now - (yaw_start + notes["steered_deg"])      # + = drifted left
                limit = getattr(B, "HEADING_HOLD_MAX_TURN", 3)
                cycle_turn = int(clamp(round(-error * getattr(B, "HEADING_HOLD_GAIN", 0.8) / per_value),
                                       -limit, limit))
                notes["held_deg"] += cycle_turn * per_value
            self.hw.walk(forward, sideways, cycle_turn, speed, gait)
            done += 1
            self.touch()
        notes["seconds"] = time.time() - start
        if not self.battery_low:
            self.hw.stand()
        self.touch()
        if imu is not None:
            try:
                yaw_now, peak, now_tilt = imu.watch()
                notes["yaw_change_deg"] = yaw_now - yaw_start
                notes["max_tilt_deg"] = peak
                notes["tilt_now_deg"] = now_tilt
            except Exception:
                pass
        if lidar is not None:
            try:
                import bert_lidar
                points = lidar.latest()
                if points:
                    notes["around_cm"] = bert_lidar.compact(bert_lidar.summarise(points))
            except Exception:
                pass
        return done, reason, notes

    def walk(self, forward, sideways, steps, speed, follow_walls=False, gait="tripod"):
        gait = check_gait(gait)
        steps = int(clamp(steps, 1, B.MAX_STEPS_PER_CALL))
        speed = int(clamp(speed, 2, B.MAX_SPEED))
        forward = int(clamp(forward, -B.MAX_STEP_LENGTH, B.MAX_STEP_LENGTH))
        sideways = int(clamp(sideways, -B.MAX_STEP_LENGTH, B.MAX_STEP_LENGTH))
        if forward == 0 and sideways == 0:
            return "Nothing to do: forward and sideways are both 0."
        done, reason, notes = self._gait(steps, forward=forward, sideways=sideways, speed=speed,
                                         follow_walls=follow_walls, gait=gait)
        length = math.hypot(forward, sideways)
        cm = done * length / 30.0 * getattr(B, "CM_PER_STEP_AT_LENGTH_30", 10.4) * gait_scale(gait)
        text = "Walked %d of %d steps (forward %d, sideways %d, speed %d), about %.0f cm (%.0f in): %s." % (
            done, steps, forward, sideways, speed, cm, cm / 2.54, reason)
        text += self._gait_text(gait, notes)
        if follow_walls:
            text += " Steered %+.0f deg to follow the walls (%s)." % (
                notes["steered_deg"], notes.get("walls", "no straight walls in reach, went straight"))
        text += self._imu_text(notes)
        if "around_cm" in notes:
            text += " Lidar now, closest cm: %s." % notes["around_cm"]
        if self.battery_getting_low():
            text += " Battery getting low (%s V): expect weaker steps and more drift; charge soon." % self.last_volts[0]
        if "around_cm" not in notes and notes["lidar"] != "on":
            text += " Lidar %s, only the ultrasonic watched." % notes["lidar"]
        if notes.get("lidar_gaps"):
            text += " (lidar had no fresh readings for %d step(s))" % notes["lidar_gaps"]
        return text

    def cruise(self, distance_cm, speed, stop_cm, follow_walls, gait="tripod"):
        """One long walk forward, guided by the lidar every step."""
        gait = check_gait(gait)
        length = getattr(B, "CRUISE_STEP_LENGTH", 30)
        per_step = getattr(B, "CM_PER_STEP_AT_LENGTH_30", 10.4) * length / 30.0 * gait_scale(gait)
        distance_cm = clamp(distance_cm, per_step, B.CRUISE_MAX_CM)
        steps = int(min(math.ceil(distance_cm / per_step), B.CRUISE_MAX_STEPS))
        stop_cm = clamp(stop_cm, B.CRUISE_MIN_STOP_CM, 300)
        speed = int(clamp(speed, 2, B.MAX_SPEED))
        done, reason, notes = self._gait(steps, forward=length, speed=speed, follow_walls=follow_walls,
                                         max_seconds=B.CRUISE_MAX_SECONDS, ahead_stop_cm=stop_cm, require_lidar=True,
                                         gait=gait)
        cm = done * per_step
        text = "Cruised %d of %d steps, about %.0f cm (%.0f in): %s." % (done, steps, cm, cm / 2.54, reason)
        text += self._gait_text(gait, notes)
        if follow_walls:
            text += " Steered %+.0f deg in all to follow the walls (%s)." % (
                notes["steered_deg"], notes.get("walls", "no straight walls in reach, went straight"))
        text += self._imu_text(notes)
        if "around_cm" in notes:
            text += " Lidar now, closest cm: %s." % notes["around_cm"]
        if self.battery_getting_low():
            text += " Battery getting low (%s V): expect weaker steps and more drift; charge soon." % self.last_volts[0]
        if notes.get("lidar_gaps"):
            text += " (lidar had no fresh readings for %d step(s))" % notes["lidar_gaps"]
        return text

    def _gait_text(self, gait, notes):
        """For the wave-gait experiment: which gait, how long it took and the servo battery before/after."""
        if gait == "tripod":
            return ""
        self.check_battery(force=True)
        return " Gait %s: %.1f s of walking, servo battery %s V before, %s V after." % (
            gait, notes.get("seconds", 0.0), notes.get("volts_before"), self.last_volts[0])

    @staticmethod
    def _imu_text(notes):
        if "yaw_change_deg" not in notes:
            return " IMU %s." % notes["imu"] if notes["imu"] != "on" else ""
        text = " IMU: heading actually changed %+.0f deg (positive = left), most tilt %.0f deg." % (
            notes["yaw_change_deg"], notes["max_tilt_deg"])
        if notes.get("held_deg"):
            text += " Heading hold steered %+.0f deg in all to keep him straight." % notes["held_deg"]
        if notes.get("imu_gaps"):
            text += " (IMU had no fresh readings for %d step(s))" % notes["imu_gaps"]
        return text

    def turn(self, degrees, speed, gait="tripod"):
        gait = check_gait(gait)
        degrees = clamp(degrees, -B.MAX_TURN_DEGREES, B.MAX_TURN_DEGREES)
        per_cycle = getattr(C, "DEG_PER_TURN_CYCLE", 10) * gait_scale(gait)
        cycles = min(int(math.ceil(abs(degrees) / per_cycle)), B.MAX_STEPS_PER_CALL * 2)
        # a full cycle is gait turn value 10; spread the angle evenly so small turns are possible
        value = int(round(10 * abs(degrees) / (cycles * per_cycle))) if cycles else 0
        if value < 1:
            return "Turn too small (the smallest turn is about %d degrees)." % max(1, round(per_cycle / 10))
        value = min(value, 10)
        # cat_patrol.Robot.walk applies TURN_SIGN; FakeHardware just logs
        done, reason, notes = self._gait(cycles, turn=value if degrees > 0 else -value, speed=int(clamp(speed, 2, B.MAX_SPEED)),
                                         gait=gait)
        return "Turned about %+d degrees by step count (%d of %d cycles, positive = left): %s.%s" % (
            int(round(done * per_cycle * value / 10.0)) * (1 if degrees > 0 else -1), done, cycles, reason,
            self._imu_text(notes))

    def stop(self):
        self.stop_event.set()
        stopped_patrol = False
        if self.patrol_running():
            self.patrol.running = False
            self.patrol_thread.join(timeout=15)
            stopped_patrol = True
        with self.hw.lock:                   # waits for the current gait cycle to finish
            self.hw.buzzer(False)
            if not self.relaxed and not self.battery_low:
                self.hw.stand()
        self.touch()
        return "Stopped%s. Standing still." % (" (patrol ended)" if stopped_patrol else "")

    def stand_up(self):
        self.require_can_move()
        self.hw.stand()
        self.relaxed = False
        self.touch()
        return "Standing."

    def relax(self):
        self.require_no_patrol()
        self.stop_event.set()
        self._relax()
        return "Legs relaxed (servos off). Call stand_up or walk to get up again."

    def body_tilt(self, roll, pitch, yaw):
        self.require_can_move()
        lim = B.BODY_TILT_LIMIT
        roll, pitch, yaw = (int(clamp(v, -lim, lim)) for v in (roll, pitch, yaw))
        self.hw.body_tilt(roll, pitch, yaw)
        self.relaxed = False
        self.touch()
        return "Body tilted roll %d, pitch %d, yaw %d. The next step or stand_up levels it again." % (roll, pitch, yaw)

    def set_height(self, height_mm, lift_mm):
        self.require_can_move()
        lo, hi = getattr(B, "BODY_HEIGHT_RANGE_MM", (25, 70))
        llo, lhi = getattr(B, "STEP_LIFT_RANGE_MM", (20, 60))
        height_mm = int(clamp(height_mm, lo, hi))
        lift_mm = int(clamp(lift_mm, llo, lhi))
        self.hw.set_height(height_mm, lift_mm)
        self.height_mm, self.lift_mm = height_mm, lift_mm
        self.relaxed = False
        self.touch()
        return ("Standing with the feet %d mm below the body (normal is %d) and lifting each foot %d mm "
                "when walking. Walks, turns and stand_up keep this until it's changed or the server restarts."
                % (height_mm, getattr(B, "BODY_HEIGHT_MM", 25), lift_mm))

    # ---- head, camera, vision -------------------------------------------
    def look(self, pan, tilt):
        self.require_no_patrol()
        pan_offset = clamp(pan, -90, 90) * getattr(C, "PAN_SIGN", 1)
        self.pan = int(clamp(C.PAN_CENTER + pan_offset, *B.PAN_RANGE))
        self.tilt = int(clamp(C.TILT_CENTER + clamp(tilt, -90, 90), *B.TILT_RANGE))
        self.hw.look(self.pan, self.tilt)
        self.touch()
        return "Head at pan servo %d, tilt servo %d (90/90 = straight ahead)." % (self.pan, self.tilt)

    def get_camera(self):
        if self.camera is None:
            if self.sim:
                self.camera = FakeCamera()
            else:
                from cat_vision import PiCameraSource
                self.camera = PiCameraSource()
                time.sleep(0.5)
        return self.camera

    def frame(self):
        frame = self.get_camera().read(newer_than=time.time() - 0.05, timeout=3.0)
        if frame is None:
            raise RuntimeError("No picture from the camera.")
        return frame

    def get_vision(self):
        if self.vision is None:
            from cat_vision import CatDetector, CatIdentifier, Vision
            self.vision = Vision(self.get_camera(), CatDetector(), CatIdentifier())
        return self.vision

    def who_is_there(self):
        self.require_no_patrol()
        vision = self.get_vision()
        from cat_vision import annotate
        _, _, _, detections = vision.look()
        frame = vision.last_frame
        if frame is None:
            raise RuntimeError("No picture from the camera.")
        found = []
        for d in detections:
            item = {"what": d.label, "confidence": round(d.score, 2),
                    "position": "left" if d.cx < -0.33 else ("right" if d.cx > 0.33 else "centre"),
                    "fills_percent_of_view": round(d.area * 100, 1)}
            if d.label == "cat":
                item["verdict"] = d.verdict
                item["orange_percent"] = round(d.orange * 100)
            found.append(item)
        self.touch()
        return found, encode_jpeg(annotate(frame, detections))

    # ---- lidar ---------------------------------------------------------------
    def get_lidar(self):
        if self.lidar is None:
            if self.sim:
                raise Refused("there is no lidar in the simulator")
            try:
                import bert_lidar
                self.lidar = bert_lidar.Lidar(log=lambda m: log("[lidar] " + m))
                self.lidar_error = None
            except Exception as exc:
                self.lidar_error = str(exc)
                raise Refused("the lidar isn't working: %s" % exc)
        return self.lidar

    # ---- IMU (BNO085) ---------------------------------------------------------
    def get_imu(self):
        if self.imu is not None and not self.imu.running:   # unplugged: reopen
            self.imu_error = self.imu.error
            self.imu.close()
            self.imu = None
        if self.imu is None:
            if self.sim:
                raise Refused("there is no IMU in the simulator")
            try:
                import bert_imu
                self.imu = bert_imu.Imu(log=lambda m: log("[imu] " + m))
                self.imu_error = None
                time.sleep(0.3)           # let the first packets arrive
            except Exception as exc:
                self.imu_error = str(exc)
                raise Refused("the IMU isn't working: %s" % exc)
        return self.imu

    def orientation(self):
        imu = self.get_imu()
        try:
            reading = imu.read()
        except Exception as exc:
            self.imu_error = str(exc)
            raise Refused("the IMU isn't sending readings: %s" % exc)
        reading["legs"] = "relaxed" if self.relaxed else "standing"
        return reading

    def lidar_scan(self, picture):
        import bert_lidar
        lidar = self.get_lidar()
        try:
            points = lidar.read_points()
        except Exception as exc:              # unplugged mid-run etc.: reopen next time
            try:
                lidar.close()
            except Exception:
                pass
            self.lidar = None
            self.lidar_error = str(exc)
            raise Refused("the lidar scan failed: %s" % exc)
        summary = bert_lidar.summarise(points, legs_relaxed=self.relaxed)
        if not self.patrol_running():
            summary["ultrasonic_ahead_cm"] = self.hw.distance()
        self.touch()
        png = bert_lidar.render_png(points) if picture else None
        return summary, png

    # ---- lights and buzzer ------------------------------------------------
    def lights(self, colour, brightness):
        self.require_no_patrol()
        r, g, b = parse_colour(colour)
        scale = clamp(brightness, 0, 100) / 100.0 * B.LED_MAX_BRIGHTNESS / 255.0
        rgb = tuple(int(v * scale) for v in (r, g, b))
        ok = self.hw.lights(rgb)
        self.touch()
        return "Lights set to %s." % (rgb,) if ok else "This robot's board doesn't support the LEDs."

    def beep(self, count, seconds):
        self.require_no_patrol()
        if in_quiet_hours():
            return "Quiet hours (%s:00 to %s:00), not beeping." % C.QUIET_HOURS
        count = int(clamp(count, 1, B.MAX_BEEPS))
        seconds = clamp(seconds, 0.05, B.MAX_BEEP_SECONDS)
        try:
            for _ in range(count):
                self.hw.buzzer(True)
                time.sleep(seconds)
                self.hw.buzzer(False)
                time.sleep(0.12)
        finally:
            self.hw.buzzer(False)
        self.touch()
        return "Beeped %d time(s)." % count

    # ---- status ------------------------------------------------------------
    def status(self):
        self.check_battery(force=not self.patrol_running())
        return {
            "name": "Large Bert",
            "simulated": self.sim,
            "battery_volts": {"load": self.last_volts[0], "pi": self.last_volts[1]},
            "battery_cutoffs_volts": {"load": MIN_LOAD_V, "pi": MIN_PI_V},
            "battery_low": self.battery_low,
            "battery_getting_low": self.battery_getting_low(),
            "distance_ahead_cm": None if self.patrol_running() else self.hw.distance(),
            "legs": "relaxed" if self.relaxed else "standing",
            "body_height_mm": getattr(self, "height_mm", getattr(B, "BODY_HEIGHT_MM", 25)),
            "step_lift_mm": getattr(self, "lift_mm", getattr(B, "STEP_LIFT_MM", 40)),
            "head_servos": {"pan": self.pan, "tilt": self.tilt},
            "patrol_running": self.patrol_running(),
            "movement_allowed": not self.no_move,
            "quiet_hours_now": in_quiet_hours(),
            "ears": self.ears.status() if self.ears else {"mic_ok": False, "error": self.ears_error},
            "lidar": self.lidar.status() if self.lidar else {"open": False, "error": self.lidar_error},
            "imu": self.imu.status() if self.imu else {"open": False, "error": self.imu_error},
            "limits": {"max_steps_per_call": B.MAX_STEPS_PER_CALL, "max_move_seconds": B.MAX_MOVE_SECONDS,
                       "max_speed": B.MAX_SPEED, "max_step_length": B.MAX_STEP_LENGTH,
                       "min_obstacle_cm": B.MIN_OBSTACLE_CM, "idle_relax_seconds": B.IDLE_RELAX_SECONDS},
        }

    # ---- patrol ------------------------------------------------------------
    def patrol_start(self, mode, dry_run):
        if self.patrol_running():
            return "The patrol is already running."
        if mode not in ("sentry", "roam"):
            raise Refused("mode must be 'sentry' or 'roam'.")
        if not dry_run:
            self.require_can_move()
        import cat_patrol
        cat_patrol.C.MODE = mode
        cat_patrol.log = self._patrol_log          # capture the patrol's messages
        if dry_run or self.sim:
            robot = cat_patrol.DryRunRobot()
        else:
            robot = LockedRobot(self.hw._hw.robot, self.hw.lock)
        vision = self.get_vision()
        vision.reset_votes()
        self.patrol = cat_patrol.Patrol(robot, vision)
        self.patrol_info = {"mode": mode, "dry_run": dry_run or self.sim, "started": time.strftime("%H:%M:%S")}
        self.relaxed = False

        def run():
            try:
                self.patrol.run()
            except Exception as exc:
                self._patrol_log("Patrol crashed: %r" % exc)
            finally:
                try:
                    robot.rest()
                except Exception:
                    pass
                self.relaxed = True
                self._patrol_log("Patrol ended.")
        self.patrol_thread = threading.Thread(target=run, daemon=True)
        self.patrol_thread.start()
        return "Cat patrol started (%s mode%s)." % (mode, ", dry run: watching only" if self.patrol_info["dry_run"] else "")

    def _patrol_log(self, message):
        line = time.strftime("%H:%M:%S ") + message
        self.patrol_log.append(line)
        log("[patrol] " + message)

    def patrol_stop(self):
        if not self.patrol_running():
            return "The patrol isn't running."
        self.patrol.running = False
        self.patrol_thread.join(timeout=20)
        self.touch()
        return "Patrol stopped." if not self.patrol_running() else "Asked the patrol to stop; it is finishing its current step."

    def patrol_status(self, lines):
        return {"running": self.patrol_running(), **self.patrol_info,
                "recent_log": list(self.patrol_log)[-int(clamp(lines, 1, 200)):]}

    # ---- shutdown ------------------------------------------------------------
    def close(self):
        if self._closed:
            return
        self._closed = True
        self.stop_event.set()
        if self.patrol_running():
            self.patrol.running = False
            self.patrol_thread.join(timeout=10)
        try:
            self.hw.lights((0, 0, 0))
        except Exception:
            pass
        self._relax()
        if self.ears is not None:
            self.ears.close()
        if self.lidar is not None:
            try:
                self.lidar.close()
            except Exception:
                pass
        if self.imu is not None:
            try:
                self.imu.close()
            except Exception:
                pass
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass
        log("Large Bert is resting. Bye!")


# ---------------------------------------------------------------------------
# MCP server
# ---------------------------------------------------------------------------
try:                                     # mcp 2.x
    from mcp.server.mcpserver import MCPServer as _Server, Image
    MCP_V2 = True
except ImportError:                      # mcp 1.x
    from mcp.server.fastmcp import FastMCP as _Server, Image
    MCP_V2 = False
import anyio

INSTRUCTIONS = """You are controlling Large Bert, a real six-legged Freenove hexapod robot in Michael's home and fenced yard.
Be careful: it is a physical machine near people and cats.
- Call status first. Take a snapshot before walking so you know what's ahead.
- Move in small steps (a few gait cycles at a time) and look again. One gait cycle is about 10 cm (4 in) at length 30, scaling with length.
- Call stop at once if anything looks wrong. Every walk is capped and will stop on its own.
- Never chase, corner or touch an animal. To shoo the orange intruder cat, use patrol_start, which has humane limits built in.
- Use listen to hear what people near him say. Saying "Bert, stop" stops him on the Pi by itself; if a walk
  ends "stopped by voice", someone told him to stop, so don't just carry on.
- For long straight runs (a hallway) use cruise: one call walks up to 4 m, steering along the walls
  and stopping by itself near anything ahead.
- Walks are watched by the lidar every step and report the distances all round at the end, so chain
  walks of up to 10 steps without stopping to scan. In hallways use follow_walls=True.
- lidar_scan shows how far things are in every direction (his head hides the first ~25 cm straight
  ahead; the ultrasonic covers that). Use it before turning, crab-walking or going through doorways,
  since the camera and sonar only see ahead.
- Walks, cruises and turns also watch the IMU through every step: they stop if his body tilts past
  30 degrees, and report how far his heading actually changed (better than step counting).
- orientation reads the IMU: pitch, roll, tilt from level, whether he's upright, and yaw (degrees turned
  since power-on, not a compass). Use it to check a turn really went as far as asked, or if he may have tipped.
- When you're done, call relax so the servos rest."""

mcp = _Server("Large Bert", instructions=INSTRUCTIONS)
BERT = None


async def run_blocking(fn, *args):
    """Run hardware work in a thread. If Claude cancels the call (Esc), stop the robot."""
    try:
        return await anyio.to_thread.run_sync(fn, *args, abandon_on_cancel=True)
    except anyio.get_cancelled_exc_class():
        BERT.stop_event.set()
        raise


async def call(fn, *args):
    try:
        return await run_blocking(fn, *args)
    except (Refused, ValueError) as exc:
        return "Refused: %s" % exc


@mcp.tool()
async def status() -> dict:
    """Battery voltages, distance ahead (ultrasonic), legs standing or relaxed, head angles, patrol state and safety limits."""
    return await run_blocking(BERT.status)


@mcp.tool()
async def walk(forward: int = 0, sideways: int = 0, steps: int = 2, speed: int = B.DEFAULT_SPEED,
               follow_walls: bool = False, gait: str = "tripod") -> str:
    """Walk a few gait cycles. forward: step length -30..30 (negative = backwards).
    sideways: -30..30 (positive = crab-walk right, negative = left). steps: 1..10 gait cycles
    (each about 10 cm (4 in) at length 30, scaling with length). speed: 2 (slow) .. 10.
    The lidar watches all the way: before every step it checks his path and both sides, and stops him
    if something is in the way or about to brush his legs. Also stops for the ultrasonic under 25 cm
    going forward, a stop command, low battery, or the time limit. The result ends with the closest
    distances all round, so there's no need for a lidar_scan between walks.
    follow_walls=True (hallways, going forward): each step steers a little to stay parallel to the
    walls and drift back to the middle, which corrects his habit of drifting left.
    gait: "tripod" (normal: three legs swing together) or "wave" (experimental: one leg at a time,
    five always down; steadier and easier on the battery but each step covers less ground; the result
    adds the walking time and servo battery before/after)."""
    return await call(BERT.walk, forward, sideways, steps, speed, follow_walls, gait)


@mcp.tool()
async def cruise(distance_cm: int = 300, speed: int = B.CRUISE_SPEED, stop_cm: int = B.CRUISE_STOP_CM,
                 follow_walls: bool = True, gait: str = "tripod") -> str:
    """Walk forward in one go, up to 400 cm (40 steps, 120 s). speed 2..10, default 10, with his longest stride, with the lidar checking before every step.
    Use it for long straight runs such as a hallway, instead of many short walks. follow_walls=True steers
    each step to stay parallel to and centred between straight walls (fixes his left drift).
    Stops by itself when something in his path is closer than stop_cm (30..300, default 45, measured from
    the lidar on his back), something comes within a few cm of his side feet, the ultrasonic sees
    something under 25 cm, someone says "Bert, stop" or stop is called, or the battery is low.
    Distance is estimated from step count and can be off on rugs. Needs the lidar.
    gait: "tripod" (normal) or "wave" (experimental, one leg at a time; see walk)."""
    return await call(BERT.cruise, distance_cm, speed, stop_cm, follow_walls, gait)


@mcp.tool()
async def turn(degrees: int, speed: int = B.DEFAULT_SPEED, gait: str = "tripod") -> str:
    """Turn on the spot. Positive = left (anticlockwise seen from above), negative = right. Up to ±180, approximate.
    gait: "tripod" (normal) or "wave" (experimental, one leg at a time; turn amounts not calibrated yet)."""
    return await call(BERT.turn, degrees, speed, gait)


@mcp.tool()
async def stop() -> str:
    """Emergency stop: halt walking (and the cat patrol, if running), buzzer off, stand still."""
    BERT.stop_event.set()
    return await run_blocking(BERT.stop)


@mcp.tool()
async def stand_up() -> str:
    """Stand up straight with the body level (also undoes body_tilt)."""
    return await call(BERT.stand_up)


@mcp.tool()
async def set_height(height_mm: int = 25, step_lift_mm: int = 40) -> str:
    """Stand taller or lower. height_mm: how far the feet sit below the body, 25 (normal) .. 70.
    Use about 50-60 outside on grass or gravel for more ground clearance; taller is a little less stable.
    step_lift_mm: how high each foot lifts while walking, 20..60 (normal 40). The setting stays for later
    walks, turns and stand_up until changed. set_height(25, 40) goes back to normal."""
    return await call(BERT.set_height, height_mm, step_lift_mm)


@mcp.tool()
async def relax() -> str:
    """Let the legs go limp (servos off) so Large Bert sits down and rests. Do this when finished."""
    return await call(BERT.relax)


@mcp.tool()
async def look(pan: int = 0, tilt: int = 0) -> str:
    """Point the head/camera. pan: degrees, positive = left, -40..90. tilt: degrees, positive = up. 0, 0 = straight ahead."""
    return await call(BERT.look, pan, tilt)


@mcp.tool()
async def snapshot() -> list:
    """Take a photo with the head camera and return it (works during the patrol too)."""
    jpeg = await run_blocking(lambda: encode_jpeg(BERT.frame()))
    BERT.touch()
    return ["Camera view (head pan servo %d, tilt servo %d)." % (BERT.pan, BERT.tilt), Image(data=jpeg, format="jpeg")]


@mcp.tool()
async def who_is_there() -> list:
    """Run the cat patrol's detector on one camera frame: finds cats and people and says whether each cat
    looks like the orange intruder, one of Michael's cats, or unsure. Returns the list and an annotated photo."""
    try:
        found, jpeg = await run_blocking(BERT.who_is_there)
    except Refused as exc:
        return ["Refused: %s" % exc]
    except (FileNotFoundError, ImportError) as exc:
        return ["Cat detector isn't set up (needs cat_vision.py, OpenCV and ./get_cat_model.sh): %s" % exc]
    summary = "Nothing detected." if not found else "Detected: %s" % found
    return [summary, Image(data=jpeg, format="jpeg")]


@mcp.tool()
async def body_tilt(roll: int = 0, pitch: int = 0, yaw: int = 0) -> str:
    """Lean the body while the feet stay put, each -15..15 degrees. Good for expressive poses."""
    return await call(BERT.body_tilt, roll, pitch, yaw)


@mcp.tool()
async def lights(colour: str = "white", brightness: int = 30) -> str:
    """Set all the LEDs. colour: a name (red, green, blue, amber, orange, purple, pink, cyan, white, off) or hex like #ff8800. brightness 0..100."""
    return await call(BERT.lights, colour, brightness)


@mcp.tool()
async def beep(count: int = 1, seconds: float = 0.1) -> str:
    """Short beeps on the buzzer, 1..5 beeps of at most 0.5 s each. Silent during quiet hours."""
    return await call(BERT.beep, count, seconds)


@mcp.tool()
async def patrol_start(mode: str = "sentry", dry_run: bool = True) -> str:
    """Start the cat patrol (watches for the orange intruder and shoos it away humanely).
    mode: 'sentry' (stays put, sweeps the camera) or 'roam' (wanders the yard).
    dry_run=True only watches and logs; set False to let it move, flash and chirp."""
    try:
        return await call(BERT.patrol_start, mode, dry_run)
    except (FileNotFoundError, ImportError) as exc:
        return "Cat patrol isn't set up (needs cat_patrol.py, OpenCV and ./get_cat_model.sh): %s" % exc


@mcp.tool()
async def patrol_stop() -> str:
    """Stop the cat patrol."""
    return await run_blocking(BERT.patrol_stop)


@mcp.tool()
async def listen(seconds: float = 6) -> dict:
    """Listen through Large Bert's microphone and return what was said (offline speech-to-text on the Pi,
    rough on unusual words). seconds: 1..30, the longest to wait; it returns soon after a sentence ends.
    Anyone saying "Bert, stop" stops him on the Pi at any time, whether or not you are listening."""
    return await call(BERT.listen, seconds)


@mcp.tool()
async def lidar_scan(picture: bool = False) -> list:
    """Read the lidar on Large Bert's back (about half a second; the first scan takes ~4 s to spin up) and report the closest thing in 8 directions
    (ahead, ahead_left, left, behind_left, behind, behind_right, right, ahead_right), in cm, plus the overall
    closest point. Angles: 0 = ahead, positive = left. His head hides the first ~25 cm straight ahead
    (beyond that the lidar sees past it); ultrasonic_ahead_cm covers that gap. picture=True adds a top-down map
    (Bert in the middle facing up, rings every 50 cm, red = obstacles, grey = ignored)."""
    try:
        summary, png = await run_blocking(BERT.lidar_scan, picture)
    except Refused as exc:
        return ["Refused: %s" % exc]
    import json
    out = [json.dumps(summary)]
    if png:
        out.append(Image(data=png, format="png"))
    return out


@mcp.tool()
async def orientation() -> dict:
    """Read Large Bert's IMU (a BNO085, ~100 readings a second): pitch_deg and roll_deg (body tilt),
    tilt_from_level_deg, upright (False = probably fallen over), yaw_deg (how far he has turned since
    the IMU powered up, positive = left; not a compass heading, and it drifts slowly), and accel_g (about 1 g total when still).
    Instant; works whether he's standing or relaxed."""
    try:
        return await run_blocking(BERT.orientation)
    except Refused as exc:
        return {"refused": str(exc)}


@mcp.tool()
async def patrol_status(lines: int = 20) -> dict:
    """Is the patrol running, and its most recent log lines."""
    return await run_blocking(BERT.patrol_status, lines)


# ---------------------------------------------------------------------------
# Running it
# ---------------------------------------------------------------------------
def load_token():
    path = os.path.expanduser(B.TOKEN_FILE)
    if os.environ.get("LARGE_BERT_TOKEN"):
        return os.environ["LARGE_BERT_TOKEN"]
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write(secrets.token_urlsafe(32))
        os.chmod(path, 0o600)
    with open(path) as f:
        return f.read().strip()


class BearerAuth:
    """Tiny ASGI wrapper: every HTTP request needs 'Authorization: Bearer <token>'."""
    def __init__(self, app, token):
        self.app = app
        self.expected = ("Bearer " + token).encode()

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            auth = dict(scope.get("headers") or []).get(b"authorization", b"")
            if not secrets.compare_digest(auth, self.expected):
                await send({"type": "http.response.start", "status": 401,
                            "headers": [(b"content-type", b"text/plain")]})
                await send({"type": "http.response.body", "body": b"Unauthorized\n"})
                return
        await self.app(scope, receive, send)


def run_http(port):
    import uvicorn
    token = load_token()
    if MCP_V2:
        app = mcp.streamable_http_app(host=B.HTTP_HOST)
    else:
        mcp.settings.host = B.HTTP_HOST
        try:
            from mcp.server.transport_security import TransportSecuritySettings
            mcp.settings.transport_security = TransportSecuritySettings(enable_dns_rebinding_protection=False)
        except ImportError:
            pass
        app = mcp.streamable_http_app()
    log("Large Bert MCP on http://<pi-address>:%d/mcp  (token in %s)" % (port, B.TOKEN_FILE))
    uvicorn.run(BearerAuth(app, token), host=B.HTTP_HOST, port=port, log_level="warning")


def _use_private_stdout():
    """Make the MCP SDK talk over the private stdout copy made by _protect_stdout."""
    import io
    import mcp.server.stdio as sdk_stdio
    original = sdk_stdio.stdio_server
    protocol_in = sys.stdin.buffer

    def stdio_server(*_args, **_kwargs):
        return original(stdin=anyio.wrap_file(io.TextIOWrapper(protocol_in, encoding="utf-8", errors="replace")),
                        stdout=anyio.wrap_file(io.TextIOWrapper(PROTOCOL_OUT, encoding="utf-8", write_through=True)))
    sys.modules[_Server.__module__].stdio_server = stdio_server


def self_check(bert):
    print("Status:", bert.status())
    print(bert.look(30, 0)); time.sleep(0.8)
    print(bert.look(0, 0))
    print(bert.lights("blue", 30)); time.sleep(0.5)
    print(bert.lights("off", 0))
    print(bert.beep(1, 0.1))
    if not bert.no_move:
        print(bert.stand_up())
        print(bert.walk(20, 0, 1, 6))
        print(bert.turn(20, 6))
    jpeg = encode_jpeg(bert.frame())
    with open("large_bert_check.jpg", "wb") as f:
        f.write(jpeg)
    print("Saved a camera picture to large_bert_check.jpg")


def stop_old_servers(timeout=5.0):
    """End any other large_bert_mcp.py left over from an earlier connection: it holds the servos and camera,
    and the new server would hang waiting for them. Linux only (reads /proc); does nothing elsewhere."""
    if not os.path.isdir("/proc"):
        return
    me = {os.getpid(), os.getppid()}
    old = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in me:
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as f:
                args = f.read().split(b"\0")
        except OSError:
            continue
        # the python process itself, not the sudo or bash wrapped around it
        if len(args) > 1 and b"python" in os.path.basename(args[0]) and \
                any(os.path.basename(a) == b"large_bert_mcp.py" for a in args[1:]) and b"--sim" not in args:
            old.append(int(entry))
    if not old:
        return
    log("Stopping an old Large Bert server left running (pid %s)." % ", ".join(map(str, old)))
    for pid in old:
        try:
            os.kill(pid, signal.SIGTERM)       # its handler relaxes the legs and turns the lights off
        except OSError:
            pass
    deadline = time.time() + timeout
    while time.time() < deadline and any(os.path.exists("/proc/%d" % pid) for pid in old):
        time.sleep(0.1)
    for pid in old:
        if os.path.exists("/proc/%d" % pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    time.sleep(0.5)                            # let the camera and GPIO be released


def main():
    global BERT
    if not ARGS.sim:
        stop_old_servers()
    BERT = Bert(sim=ARGS.sim, no_move=ARGS.no_move, use_ears=not ARGS.sim and not ARGS.no_ears)
    atexit.register(BERT.close)

    def on_signal(signum, _frame):
        BERT.close()
        os._exit(0)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, on_signal)

    if ARGS.check:
        try:
            self_check(BERT)
        finally:
            BERT.close()
        return
    log("Large Bert MCP server ready (%s%s)." % ("simulated" if ARGS.sim else "real robot",
                                               ", movement off" if ARGS.no_move else ""))
    try:
        if ARGS.http:
            run_http(ARGS.port or B.HTTP_PORT)
        else:
            _use_private_stdout()
            mcp.run("stdio")
    finally:
        BERT.close()


if __name__ == "__main__":
    main()
