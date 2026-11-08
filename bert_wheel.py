# -*- coding: utf-8 -*-
"""Drive Large Bert from a steering wheel and pedals (or any joystick) on the PC.

Starts bert_drive.py on the Pi over SSH and streams the wheel 20 times a second.
Steering turns him, the accelerator walks forward, the brake walks backwards,
optional paddles crab-walk sideways, and a horn button beeps.

    pip install pygame
    python bert_wheel.py               # drive (calibrates the first time)
    python bert_wheel.py --calibrate   # redo the wheel/pedal setup
    python bert_wheel.py --sim         # drive a pretend Bert on the Pi (nothing moves)

In the window:  S = stand up   R = relax legs   Esc or close = stop and quit.
Let go of both pedals and he stops. If this program, the wheel or Wi-Fi stops
for half a second, Bert stops by himself; if the connection drops he relaxes.
"""
import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame

PI = "pi@192.168.0.119"
PI_DIR = "~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_pi/Code/Server"
PI_PYTHON = "/home/pi/bert-venv/bin/python"
SEND_HZ = 20
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bert_wheel.json")


# ---------------------------------------------------------------------------
# Input mapping
# ---------------------------------------------------------------------------
def all_sticks():
    return [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]


def snapshot(sticks):
    pygame.event.pump()
    return {(s.get_instance_id(), a): s.get_axis(a) for s in sticks for a in range(s.get_numaxes())}


def pressed_buttons(sticks):
    pygame.event.pump()
    return [(s.get_instance_id(), b) for s in sticks for b in range(s.get_numbuttons()) if s.get_button(b)]


class Mapping:
    """Which axis is steering/throttle/brake and their rest and full positions, saved by name."""
    def __init__(self, data):
        self.d = data

    def save(self):
        with open(CONFIG, "w") as f:
            json.dump(self.d, f, indent=2)

    @staticmethod
    def load():
        try:
            with open(CONFIG) as f:
                return Mapping(json.load(f))
        except (OSError, ValueError):
            return None

    def resolve(self, sticks):
        """Turn saved stick names into live joystick objects. Returns False if a device is missing."""
        by_name = {s.get_name(): s for s in sticks}
        self.live = {}
        for role in ("steer", "throttle", "brake"):
            entry = self.d.get(role)
            if entry and entry["stick"] not in by_name:
                return False
        for role, entry in self.d.items():
            if isinstance(entry, dict) and entry.get("stick") in by_name:
                self.live[role] = (by_name[entry["stick"]], entry)
        return True

    def pedal(self, role):
        if role not in self.live:
            return 0.0
        stick, e = self.live[role]
        v = stick.get_axis(e["axis"])
        span = e["full"] - e["rest"]
        if abs(span) < 0.2:
            return 0.0
        return max(0.0, min(1.0, (v - e["rest"]) / span))

    def steer(self):
        if "steer" not in self.live:
            return 0.0
        stick, e = self.live["steer"]
        v = stick.get_axis(e["axis"]) - e.get("center", 0.0)
        v *= e.get("sign", 1)
        lock = e.get("lock", 1.0)          # how far the wheel turns for a full turn rate
        return max(-1.0, min(1.0, v / lock))

    def button(self, role):
        if role not in self.live:
            return False
        stick, e = self.live[role]
        return bool(stick.get_button(e["button"]))


# ---------------------------------------------------------------------------
# Window
# ---------------------------------------------------------------------------
class Screen:
    def __init__(self):
        self.win = pygame.display.set_mode((640, 360))
        pygame.display.set_caption("Large Bert - wheel drive")
        self.big = pygame.font.SysFont("segoeui", 30, bold=True)
        self.font = pygame.font.SysFont("segoeui", 20)

    def draw(self, title, lines, colour=(230, 230, 230), bars=None):
        self.win.fill((22, 26, 32))
        self.win.blit(self.big.render(title, True, colour), (24, 18))
        y = 72
        for line in lines:
            self.win.blit(self.font.render(line, True, (210, 214, 220)), (24, y))
            y += 28
        if bars:
            y = 250
            for name, value, lo in bars:
                self.win.blit(self.font.render(name, True, (160, 166, 176)), (24, y))
                pygame.draw.rect(self.win, (60, 66, 76), (140, y + 6, 460, 14), border_radius=7)
                if lo < 0:      # centred bar
                    w = int(230 * value)
                    x = 370 + min(0, w)
                    pygame.draw.rect(self.win, (90, 170, 255), (x, y + 6, abs(w), 14), border_radius=7)
                else:
                    pygame.draw.rect(self.win, (90, 210, 130), (140, y + 6, int(460 * value), 14), border_radius=7)
                y += 30
        pygame.display.flip()


def wait_step(screen, sticks, title, lines, seconds=None):
    """Show a prompt and record axis extremes until Enter (or a timeout). Returns (min, max, last) per axis."""
    lo, hi = {}, {}
    start = time.time()
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                sys.exit("Calibration cancelled.")
            if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_SPACE):
                return lo, hi, snapshot(sticks)
        now = snapshot(sticks)
        for k, v in now.items():
            lo[k] = min(lo.get(k, v), v)
            hi[k] = max(hi.get(k, v), v)
        if seconds and time.time() - start > seconds:
            return lo, hi, now
        screen.draw(title, lines + ["", "Press Enter when done (Esc cancels)."])
        time.sleep(0.02)


def wait_button(screen, sticks, title, lines):
    """Wait for a new button press, or Enter to skip. Returns (stick, button) or None."""
    held = set(pressed_buttons(sticks))
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                sys.exit("Calibration cancelled.")
            if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                return None
        now = set(pressed_buttons(sticks))
        new = now - held
        held = now
        if new:
            return sorted(new)[0]
        screen.draw(title, lines + ["", "Press Enter to skip."])
        time.sleep(0.02)


def calibrate(screen, sticks):
    names = {s.get_instance_id(): s.get_name() for s in sticks}

    def biggest(lo, hi, exclude=()):
        best, best_range = None, 0.0
        for k in lo:
            if k in exclude:
                continue
            r = hi[k] - lo[k]
            if r > best_range:
                best, best_range = k, r
        return best, best_range

    wait_step(screen, sticks, "Setup 1/6: hands off", [
        "Let go of the wheel and all pedals.",
        "(Some wheels report 0 until each control has moved once,",
        "so the next steps move everything.)"])
    lo, hi, last = wait_step(screen, sticks, "Setup 2/6: steering", [
        "Turn the wheel all the way LEFT, then all the way RIGHT,",
        "then back to the CENTRE and hold it there."])
    steer, r = biggest(lo, hi)
    if steer is None or r < 0.5:
        sys.exit("Didn't see the wheel move. Is it plugged in and switched on?")
    centre = last[steer]
    data = {"steer": {"stick": names[steer[0]], "axis": steer[1], "center": centre,
                      "sign": 1, "lock": 0.3}}

    lo, hi, last = wait_step(screen, sticks, "Setup 3/6: accelerator", [
        "Press the ACCELERATOR all the way down, then let it go."])
    thr, r = biggest(lo, hi, exclude=(steer,))
    if thr is None or r < 0.5:
        sys.exit("Didn't see the accelerator move.")
    rest = last[thr]
    full = lo[thr] if abs(lo[thr] - rest) > abs(hi[thr] - rest) else hi[thr]
    data["throttle"] = {"stick": names[thr[0]], "axis": thr[1], "rest": rest, "full": full}

    lo, hi, last = wait_step(screen, sticks, "Setup 4/6: brake (reverse)", [
        "Press the BRAKE all the way down, then let it go.",
        "(Press Enter without touching it to drive without reverse.)"])
    brk, r = biggest(lo, hi, exclude=(steer, thr))
    if brk is not None and r >= 0.5:
        rest = last[brk]
        full = lo[brk] if abs(lo[brk] - rest) > abs(hi[brk] - rest) else hi[brk]
        data["brake"] = {"stick": names[brk[0]], "axis": brk[1], "rest": rest, "full": full}

    # which way is left? ask for a gentle left turn
    _, _, last = wait_step(screen, sticks, "Setup 5/6: direction", [
        "Hold the wheel turned a little to the LEFT, then press Enter."])
    data["steer"]["sign"] = 1 if last[steer] - centre > 0 else -1     # + means left for Bert

    for role, prompt in (("crab_left", "Press the button or paddle for CRAB-WALK LEFT."),
                         ("crab_right", "Press the button or paddle for CRAB-WALK RIGHT."),
                         ("horn", "Press the button for the HORN (a short beep).")):
        got = wait_button(screen, sticks, "Setup 6/6: buttons", [prompt])
        if got:
            data[role] = {"stick": names[got[0]], "button": got[1]}
    m = Mapping(data)
    m.save()
    return m


# ---------------------------------------------------------------------------
# Link to the Pi
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, sim):
        remote = "cd %s && exec %s%s bert_drive.py%s" % (
            PI_DIR, "" if sim else "sudo -n ", PI_PYTHON, " --sim" if sim else "")
        self.proc = subprocess.Popen(
            ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
             "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", PI, remote],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.status = {"state": "connecting..."}
        self.obstacle_cm = 25
        self.errors = []
        self.inbox = queue.Queue()
        threading.Thread(target=self._read, args=(self.proc.stdout, True), daemon=True).start()
        threading.Thread(target=self._read, args=(self.proc.stderr, False), daemon=True).start()

    def _read(self, pipe, is_status):
        for raw in iter(pipe.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            if is_status:
                try:
                    self.status = json.loads(line)
                    if "obstacle_cm" in self.status:
                        self.obstacle_cm = self.status["obstacle_cm"]
                    if self.status.get("msg"):
                        self.inbox.put(self.status["msg"])
                except ValueError:
                    pass
            else:
                self.errors.append(line)
                del self.errors[:-6]
                print("[pi]", line)

    def alive(self):
        return self.proc.poll() is None

    def send(self, **msg):
        try:
            self.proc.stdin.write((json.dumps(msg) + "\n").encode())
            self.proc.stdin.flush()
            return True
        except OSError:
            return False

    def close(self):
        try:
            self.send(cmd="quit")
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Drive Large Bert from a steering wheel.")
    parser.add_argument("--calibrate", action="store_true", help="redo the wheel and pedal setup")
    parser.add_argument("--sim", action="store_true", help="drive a pretend Bert (nothing moves)")
    parser.add_argument("--no-robot", action="store_true", help="just show the wheel inputs, don't connect")
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    sticks = all_sticks()
    for s in sticks:
        s.init()
    if not sticks:
        sys.exit("No wheel or joystick found. Plug it in and try again.")
    screen = Screen()

    mapping = None if args.calibrate else Mapping.load()
    if mapping is None or not mapping.resolve(sticks):
        mapping = calibrate(screen, sticks)
        mapping.resolve(sticks)

    # Safety: don't connect until both pedals are up (avoids a surprise lunge).
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                return
        pygame.event.pump()
        t, b = mapping.pedal("throttle"), mapping.pedal("brake")
        if t < 0.05 and b < 0.05:
            break
        screen.draw("Let go of the pedals", ["Bert won't connect while a pedal reads as pressed.",
                                             "If you aren't touching them, tap each pedal once",
                                             "(the wheel reports 0 until a pedal first moves)."],
                    bars=[("Accel", t, 0), ("Brake", b, 0)])
        time.sleep(0.05)

    link = None if args.no_robot else Link(args.sim)
    clock = pygame.time.Clock()
    note, note_until = "", 0
    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    return
                if ev.type == pygame.KEYDOWN and link:
                    if ev.key == pygame.K_s:
                        link.send(cmd="stand")
                    elif ev.key == pygame.K_r:
                        link.send(cmd="relax")
            throttle, brake, steer = mapping.pedal("throttle"), mapping.pedal("brake"), mapping.steer()
            fwd = throttle - brake
            side = (1.0 if mapping.button("crab_right") else 0.0) - (1.0 if mapping.button("crab_left") else 0.0)
            horn = 1 if mapping.button("horn") else 0
            title, colour = "Wheel test (not connected)", (230, 230, 230)
            lines = ["Steering %+.2f   accel %.2f   brake %.2f" % (steer, throttle, brake)]
            if link:
                if not link.alive():
                    msg = link.status.get("msg") or (link.errors[-1] if link.errors else "connection closed")
                    screen.draw("Disconnected", [msg[:70], msg[70:140], "", "Bert relaxes by himself. Esc to quit."],
                                colour=(255, 120, 100))
                    while True:
                        for ev in pygame.event.get():
                            if ev.type in (pygame.QUIT, pygame.KEYDOWN):
                                return
                        time.sleep(0.05)
                if link.status.get("state") != "connecting...":   # don't queue input while he boots
                    link.send(fwd=round(fwd, 3), turn=round(steer, 3), side=side, horn=horn)
                st = link.status
                state = st.get("state", "?")
                title = "Bert: " + state
                colour = (255, 120, 100) if any(w in state for w in ("blocked", "refused", "low", "busy")) else \
                         (120, 220, 150) if "driving" in state else (230, 230, 230)
                lines.append("Distance ahead: %s cm   (stops forward under %s cm)" % (
                    st.get("dist", "?"), link.obstacle_cm))
                lines.append("Battery: load %s V   pi %s V%s" % (st.get("load_v", "?"), st.get("pi_v", "?"),
                                                                    "   LOW" if st.get("low") else ""))
                try:
                    note, note_until = link.inbox.get_nowait(), time.time() + 6
                except queue.Empty:
                    pass
                if time.time() < note_until:
                    lines.append(note[:72])
            lines.append("S stand   R relax   Esc quit" + ("   (SIMULATED)" if args.sim else ""))
            screen.draw(title, lines, colour,
                        bars=[("Steer", steer, -1), ("Accel", throttle, 0), ("Brake", brake, 0)])
            clock.tick(SEND_HZ)
    finally:
        if link:
            link.close()
        pygame.quit()


if __name__ == "__main__":
    main()
