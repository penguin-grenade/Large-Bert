# -*- coding: utf-8 -*-
"""Large Bert's lidar: an RPLIDAR on the top plate, plugged into the Pi over USB.

The motor starts on the first scan and keeps spinning for KEEP_SPINNING_SECONDS after the last
one, so a run of scans is quick (the C1 takes about 3 s to spin up) but it rests when not used. It speaks the plain SCAN command, which
every RPLIDAR (A1, A2, A3, C1, S1, S2...) understands at its own baud rate, so the model
is found by asking it, not by settings.

Angles everywhere outside this file are Bert's: 0 = straight ahead, positive = to his LEFT
(anticlockwise seen from above, the same as turn() and look()), -90 = right, +/-180 = behind.

Things it ignores, so they aren't reported as obstacles:
  * the head: it sits just in front of the lidar, so straight ahead (+/-HEAD_MASK_DEG) the lidar
    sees the head at 12-19 cm. Points there closer than HEAD_MASK_CM are dropped; the lidar still
    sees past the head further out, and the ultrasonic sensor covers it too.
  * Bert himself: anything closer than SELF_RADIUS_CM. His legs stay below the scan plane even
    sitting (checked 2026-11-16), so apart from the head nothing of him shows up.

Try it on its own (stop the MCP server first: only one program can open the lidar):
    python3 bert_lidar.py                  # model, health and one scan summary
    python3 bert_lidar.py --raw            # every point, Bert's angle and cm, nothing masked
    python3 bert_lidar.py --png scan.png   # also save a top-down picture
"""
import glob
import math
import os
import struct
import threading
import time

# ---- settings ---------------------------------------------------------------
PORT = os.environ.get("BERT_LIDAR_PORT", "")   # empty: find it (see find_port)
YAW_OFFSET_DEG = 0.0      # lidar angle (clockwise) that points straight ahead of Bert
HEAD_MASK_DEG = 15.0      # the head shows up within +/- this many degrees of straight ahead...
HEAD_MASK_CM = 25.0       # ...closer than this (measured 2026-11-16: +/-10 deg, 12-19 cm)
SELF_RADIUS_CM = 12.0     # ignore points closer than this (Bert's own body)
MAX_RANGE_CM = 1200.0     # ignore anything further (A1 is 12 m, C1 12 m)
MIN_QUALITY = 1           # A-series reports 0 for a failed reading
SCAN_SECONDS = 0.6        # how long one scan collects; the C1 does ~10 turns a second
SPIN_UP_TIMEOUT = 6.0     # the C1 sends nothing for ~3 s after SCAN while the motor gets up to speed
KEEP_SPINNING_SECONDS = 30  # stop the motor this long after the last scan
MIN_COVERAGE = 0.9        # a scan must sweep this much of the circle (empty readings count) to be trusted
LATEST_SECONDS = 0.3      # latest(): how much of the newest stream to use (~3 turns on a C1)
MIN_COVERAGE_LATEST = 0.8

# ---- walking guard (used by the MCP server's walks) ---------------------------
# Positions are from the lidar: x = cm ahead, y = cm to the left. Measure Bert standing and set these.
BODY_HALF_LENGTH_CM = 22  # lidar to the tip of his front (and back) feet
BODY_HALF_WIDTH_CM = 24   # lidar to the tip of his side feet
LOOKAHEAD_CM = 12         # stop if something is closer than this past his feet, in the way he's going
FOLLOW_MAX_TURN_DEG = 9   # follow_walls: most he steers in one gait cycle
SECTORS = (               # name, centre angle (Bert's, + = left), half width
    ("ahead", 0, 22.5), ("ahead_left", 45, 22.5), ("left", 90, 22.5), ("behind_left", 135, 22.5),
    ("behind", 180, 22.5), ("behind_right", -135, 22.5), ("right", -90, 22.5), ("ahead_right", -45, 22.5),
)
BAUDS = (115200, 460800, 256000, 1000000)   # A1/A2M8 | C1, A2M12 | A3, S1 | S2
MODELS = {0x18: "A1", 0x28: "A2", 0x29: "A2", 0x2C: "A2", 0x31: "A3", 0x41: "C1",
          0x61: "S1", 0x71: "S2", 0x72: "S2", 0x81: "S3"}


def _log(message):
    import sys
    sys.stderr.write("[lidar] %s\n" % message)
    sys.stderr.flush()


def find_port():
    """BERT_LIDAR_PORT if set, else a USB serial port whose name looks like a lidar's adapter.
    Uses /dev/serial/by-id so it stays right when other USB serial things (an IMU adapter) are added."""
    if PORT:
        return PORT
    ids = sorted(glob.glob("/dev/serial/by-id/*"))
    for want in ("RPLIDAR", "Silicon_Labs_CP210", "1a86", "CH340", "CH341", "CH343", "FTDI"):
        for path in ids:
            if want.lower() in os.path.basename(path).lower():
                return path
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    raise RuntimeError("no lidar found: no USB serial port (/dev/ttyUSB*, /dev/ttyACM*). Is it plugged in with a data cable?")


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class Lidar:
    def __init__(self, port=None, log=_log):
        import serial
        self.serial_mod = serial
        self.log = log
        self.port = port or find_port()
        self.lock = threading.Lock()
        self.ser = None
        self.baud = None
        self.info = None
        self.last_scan_time = None
        self.spinning = False
        self._reader = None
        self._buf_lock = threading.Lock()
        self._chunks = []
        self._connect()
        threading.Thread(target=self._idle_stopper, daemon=True).start()

    # ---- low level ------------------------------------------------------------
    def _cmd(self, cmd, payload=b""):
        if payload:
            body = bytes((0xA5, cmd, len(payload))) + payload
            chk = 0
            for b in body:
                chk ^= b
            self.ser.write(body + bytes((chk,)))
        else:
            self.ser.write(bytes((0xA5, cmd)))

    def _descriptor(self):
        d = self.ser.read(7)
        if len(d) != 7 or d[:2] != b"\xa5\x5a":
            raise IOError("no reply from the lidar (got %r)" % d.hex())
        size = struct.unpack("<I", d[2:6])[0] & 0x3FFFFFFF
        return size, d[6]

    def _motor(self, on):
        # A1 (and the C1's adapter) use DTR for the motor: low = spin. A2/A3 take a PWM command.
        self.ser.dtr = not on
        if self.info and self.info["model_id"] >> 4 in (2, 3):
            self._cmd(0xF0, struct.pack("<H", 660 if on else 0))

    def _stop(self):
        self._cmd(0x25)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def _connect(self):
        last = None
        for baud in BAUDS:
            try:
                ser = self.serial_mod.Serial(self.port, baud, timeout=0.6)
            except Exception as exc:
                raise RuntimeError("can't open the lidar at %s: %s" % (self.port, exc))
            self.ser = ser
            try:
                ser.dtr = True
                self._stop()
                self._cmd(0x50)                       # GET_INFO
                size, _ = self._descriptor()
                data = ser.read(size)
                if len(data) < 20:
                    raise IOError("short info reply")
                self._cmd(0x52)                       # GET_HEALTH
                self._descriptor()
                health = ser.read(3)
                status = {0: "good", 1: "warning", 2: "error"}.get(health[0] if health else -1, "unknown")
                self.baud = baud
                self.info = {"model_id": data[0], "model": MODELS.get(data[0], "RPLIDAR model 0x%02X" % data[0]),
                             "firmware": "%d.%02d" % (data[2], data[1]), "hardware": data[3],
                             "serial": data[4:20][::-1].hex().upper(), "health": status,
                             "port": self.port, "baud": baud}
                if status == "error":
                    self._cmd(0x40)                   # RESET clears a latched error
                    time.sleep(1.0)
                    ser.reset_input_buffer()
                self.log("found %s (fw %s) at %s, %d baud, health %s" % (
                    self.info["model"], self.info["firmware"], self.port, baud, status))
                return
            except Exception as exc:
                last = exc
                ser.close()
                self.ser = None
        raise RuntimeError("the lidar at %s didn't answer at any speed (%s). Try unplugging it and plugging it back in."
                           % (self.port, last))

    # ---- scanning ------------------------------------------------------------
    # While the motor spins, a reader thread keeps the last couple of seconds of raw readings.
    # read_points() waits for fresh ones; latest() hands back the newest without waiting, so a
    # walk can check its surroundings every step without stopping.
    def _start_scan(self):
        self._motor(True)
        self.ser.reset_input_buffer()
        self._cmd(0x20)                               # SCAN: 5-byte nodes
        size, _ = self._descriptor()
        if size != 5:
            raise IOError("unexpected scan format (%d-byte nodes)" % size)
        deadline = time.time() + SPIN_UP_TIMEOUT      # wait for the motor to reach speed
        while time.time() < deadline and self.ser.in_waiting < 500:
            time.sleep(0.05)
        if self.ser.in_waiting < 500:
            raise IOError("the lidar sent no readings within %.0f s of starting; is the motor turning?" % SPIN_UP_TIMEOUT)
        self.ser.reset_input_buffer()
        with self._buf_lock:
            self._chunks = []
        self.spinning = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        ser = self.ser
        while self.spinning and ser is not None:
            try:
                data = ser.read(max(1, ser.in_waiting or 1))
            except Exception as exc:
                self.log("lidar read failed: %s" % exc)
                self.spinning = False
                break
            if data:
                now = time.time()
                with self._buf_lock:
                    self._chunks.append((now, data))
                    while self._chunks and now - self._chunks[0][0] > 2.5:
                        self._chunks.pop(0)

    def _since(self, t0):
        with self._buf_lock:
            return b"".join(d for t, d in self._chunks if t > t0)

    def _halt(self):
        self.spinning = False
        reader = getattr(self, "_reader", None)
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=1.5)
        self._reader = None
        try:
            self._stop()
            self._motor(False)
        except Exception:
            pass

    def ensure_spinning(self):
        """Start the motor and the reader if they aren't running (takes ~3 s on a C1)."""
        with self.lock:
            if not self.spinning:
                try:
                    self._start_scan()
                except Exception:
                    self._halt()
                    raise
            self.last_scan_time = time.time()

    def _idle_stopper(self):
        while self.ser is not None:
            time.sleep(2)
            if self.spinning and self.last_scan_time and time.time() - self.last_scan_time > KEEP_SPINNING_SECONDS:
                with self.lock:
                    if self.spinning and time.time() - self.last_scan_time > KEEP_SPINNING_SECONDS:
                        self._halt()

    def read_points(self, seconds=SCAN_SECONDS, _retry=False):
        """Collect fresh readings for `seconds` (spinning up first if needed).
        Returns [(lidar_angle_deg_clockwise, distance_cm, quality)]."""
        seconds = max(0.2, min(float(seconds), 2.0))
        self.ensure_spinning()
        t0 = time.time()
        time.sleep(seconds)
        raw = self._since(t0)
        self.last_scan_time = time.time()
        points, coverage = parse_nodes(raw, with_coverage=True)
        if coverage < MIN_COVERAGE:                   # stream went bad: start it again, once
            self.log("only %d%% of the circle came back, restarting the scan" % (coverage * 100))
            with self.lock:
                self._halt()
            if not _retry:
                return self.read_points(seconds, _retry=True)
            raise IOError("the lidar only swept %d%% of the circle; is something stopping it turning?" % (coverage * 100))
        return points

    def latest(self, seconds=LATEST_SECONDS):
        """The newest readings (the last `seconds`) without waiting, or None if the lidar isn't spinning
        or the stream looks broken. Keeps the motor spinning."""
        if not self.spinning:
            return None
        self.last_scan_time = time.time()
        points, coverage = parse_nodes(self._since(time.time() - seconds), with_coverage=True)
        return points if coverage >= MIN_COVERAGE_LATEST else None

    def scan(self, seconds=SCAN_SECONDS, legs_relaxed=None):
        points = self.read_points(seconds)
        return summarise(points, legs_relaxed=legs_relaxed)

    def status(self):
        out = dict(self.info or {})
        out["spinning"] = self.spinning
        out["last_scan"] = time.strftime("%H:%M:%S", time.localtime(self.last_scan_time)) if self.last_scan_time else None
        return out

    def close(self):
        if self.ser is not None:
            with self.lock:
                self._halt()
            self.ser.close()
            self.ser = None


def _node_angle(raw, i):
    """Angle (q6) of the node at byte i, or None if its check bits are wrong."""
    b0, b1, b2 = raw[i], raw[i + 1], raw[i + 2]
    if not (((b0 & 1) ^ ((b0 >> 1) & 1)) and (b1 & 1)):
        return None
    angle = (b1 >> 1) | (b2 << 7)
    return angle if angle < 360 * 64 else None


def parse_nodes(raw, with_coverage=False):
    """Standard SCAN nodes, 5 bytes each. The check bits are only 2 bits, so a node is only trusted when
    it and the next two nodes all pass them AND their angles step forward by a little (the lidar turns
    under 1 degree per reading). After a dropped byte it slides along one byte at a time until it locks
    on again, so a gap in the stream costs a few readings, not the rest of the scan.
    Returns [(lidar_angle_deg_clockwise, distance_cm, quality)], plus the fraction of the circle that
    was swept (in 5 degree slices) when with_coverage is set."""
    def close(a, b):
        return (b - a) % (360 * 64) < 5 * 64
    points, swept = [], set()
    i, n = 0, len(raw)
    while i + 15 <= n:
        a0 = _node_angle(raw, i)
        a1 = _node_angle(raw, i + 5) if a0 is not None else None
        a2 = _node_angle(raw, i + 10) if a1 is not None else None
        if a2 is None or not (close(a0, a1) and close(a1, a2)):
            i += 1
            continue
        angle = a0 / 64.0
        swept.add(int(angle // 5))
        dist_mm = (raw[i + 3] | (raw[i + 4] << 8)) / 4.0
        if dist_mm > 0:
            points.append((angle, dist_mm / 10.0, raw[i] >> 2))
        i += 5
    if with_coverage:
        return points, len(swept) / 72.0
    return points


def to_bert(points):
    """Lidar angles (clockwise) -> Bert's angles (0 ahead, + left)."""
    return [(wrap180(-(a - YAW_OFFSET_DEG)), d, q) for a, d, q in points]


def is_masked(bert_angle, dist_cm):
    if abs(bert_angle) <= HEAD_MASK_DEG and dist_cm < HEAD_MASK_CM:
        return "head"
    if dist_cm < SELF_RADIUS_CM:
        return "self"
    if dist_cm > MAX_RANGE_CM:
        return "far"
    return None


def summarise(points, legs_relaxed=None):
    """Closest thing in each of 8 directions, with Bert's own head and body left out."""
    bert = to_bert(points)
    good = despeckle([(a, d) for a, d, q in bert if q >= MIN_QUALITY and not is_masked(a, d)])
    sectors = {}
    for name, centre, half in SECTORS:
        hits = [d for a, d in good if abs(wrap180(a - centre)) <= half]
        sectors[name] = {"closest_cm": round(min(hits)) if hits else None, "points": len(hits)}
    closest = min(good, key=lambda p: p[1]) if good else None
    out = {
        "points": len(points), "used": len(good),
        "directions": sectors,
        "closest": {"cm": round(closest[1]), "angle_deg": round(closest[0]),
                    "where": direction_name(closest[0])} if closest else None,
        "angles": "0 = straight ahead, positive = left, negative = right, 180 = behind",
        "ignored": "within %d cm straight ahead (+/-%d deg: his head) and within %d cm all round (Bert himself)"
                   % (HEAD_MASK_CM, HEAD_MASK_DEG, SELF_RADIUS_CM),
    }
    if len(points) < 100:
        out["warning"] = (out.get("warning", "") + " Very few points came back; the lidar may not be spinning.").strip()
    return out


def despeckle(points):
    """Drop lone readings: a real object gives several neighbouring readings at about the same range,
    while a glitch (a byte lost on USB) gives one odd point. Keeps a point if another point within
    2 degrees is within 10% (at least 5 cm) of its distance."""
    pts = sorted(points)
    keep = []
    for k, (a, d) in enumerate(pts):
        for j in (k - 2, k - 1, k + 1, k + 2):
            if 0 <= j < len(pts):
                a2, d2 = pts[j]
                if abs(wrap180(a2 - a)) <= 2.0 and abs(d2 - d) <= max(5.0, 0.1 * d):
                    keep.append((a, d))
                    break
    return keep


def to_xy(points):
    """Good points as (x cm ahead, y cm left), with the head, Bert himself and glitches left out."""
    bert = [(a, d) for a, d, q in to_bert(points) if q >= MIN_QUALITY and not is_masked(a, d)]
    out = []
    for a, d in despeckle(bert):
        r = math.radians(a)
        out.append((d * math.cos(r), d * math.sin(r)))
    return out


def in_the_way(points, forward, sideways, ahead_stop_cm=None):
    """For a walk step (forward/sideways step lengths, + = ahead / right), return a short reason if
    something is about to touch him, else None. Checks the strip he is moving into (his width going
    forward or back, his length going sideways) and right beside his feet (the door-frame case).
    ahead_stop_cm (going forward): stop once something in his path is closer than this to the lidar."""
    xy = to_xy(points)
    hl, hw, la = BODY_HALF_LENGTH_CM, BODY_HALF_WIDTH_CM, LOOKAHEAD_CM
    ahead_limit = max(hl + la, ahead_stop_cm or 0)
    for x, y in xy:
        if forward > 0 and hl * 0.5 < x < ahead_limit and abs(y) < hw:
            return "something %.0f cm ahead in his path (lidar)" % x
        if forward < 0 and -(hl + la) < x < -hl * 0.5 and abs(y) < hw:
            return "something %.0f cm behind in his path (lidar)" % -x
        if sideways > 0 and -(hw + la) < y < -hw * 0.5 and abs(x) < hl:
            return "something %.0f cm to his right (lidar)" % -y
        if sideways < 0 and hw * 0.5 < y < hw + la and abs(x) < hl:
            return "something %.0f cm to his left (lidar)" % y
        if (forward or sideways) and abs(x) < hl and abs(y) < hw + 3:
            return "something brushing his %s side, %.0f cm out (lidar)" % ("left" if y > 0 else "right", abs(y))
    return None


def _fit_wall(pts):
    """Least-squares y = a + b*x through one side's points; returns (a, b) or None."""
    if len(pts) < 12:
        return None
    n = float(len(pts))
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx < 200:                                    # points bunched up: not a wall
        return None
    b = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
    a = my - b * mx
    spread = sum((p[1] - a - b * p[0]) ** 2 for p in pts) / n
    return (a, b) if spread < 36 else None           # more than ~6 cm scatter: furniture, not a wall


def wall_steer(points):
    """In a hallway, how much to turn (degrees, + = left) to run parallel to the walls and drift back
    to the middle. Returns (degrees, note) or (0, None) when no straight wall is in reach."""
    xy = to_xy(points)
    near = [(x, y) for x, y in xy if -40 < x < 80 and BODY_HALF_WIDTH_CM * 0.8 < abs(y) < 130]
    left = _fit_wall([p for p in near if p[1] > 0])
    right = _fit_wall([p for p in near if p[1] < 0])
    walls = [w for w in (left, right) if w]
    if not walls:
        return 0.0, None
    heading = math.degrees(math.atan(sum(w[1] for w in walls) / len(walls)))  # wall direction, + = left
    offset = 0.0
    if left and right:
        offset = (left[0] + right[0]) / 2.0          # cm the middle of the hall is to his left
    turn = heading + 0.25 * offset
    turn = max(-FOLLOW_MAX_TURN_DEG, min(FOLLOW_MAX_TURN_DEG, turn))
    if left and right:
        note = "walls both sides, last step %.0f deg off parallel and %.0f cm %s of the middle" % (
            abs(heading), abs(offset), "right" if offset > 0 else "left")
    else:
        note = "wall on the %s only, last step %.0f deg off parallel" % ("left" if left else "right", abs(heading))
    return turn, note


def compact(summary):
    """One line for walk results: closest cm in each direction."""
    d = summary["directions"]
    short = {"ahead": "ahead", "ahead_left": "ahead-L", "left": "L", "behind_left": "behind-L", "behind": "behind",
             "behind_right": "behind-R", "right": "R", "ahead_right": "ahead-R"}
    return ", ".join("%s %s" % (short[k], d[k]["closest_cm"] if d[k]["closest_cm"] is not None else "-") for k in short)


def direction_name(angle):
    for name, centre, half in SECTORS:
        if abs(wrap180(angle - centre)) <= half:
            return name
    return "?"


def render_png(points, size=480, max_cm=300):
    """Top-down picture: Bert in the middle facing up, grey = ignored points, red = used points."""
    import cv2
    import numpy as np
    img = np.full((size, size, 3), 255, np.uint8)
    c = size // 2
    scale = (size / 2 - 10) / float(max_cm)
    for r in range(50, max_cm + 1, 50):
        cv2.circle(img, (c, c), int(r * scale), (225, 225, 225), 1)
        cv2.putText(img, "%dcm" % r, (c + 3, c - int(r * scale) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 160, 160), 1)
    mask_px = int(HEAD_MASK_CM * scale)
    for sign in (-1, 1):
        a = math.radians(HEAD_MASK_DEG * sign)
        cv2.line(img, (c, c), (int(c - mask_px * math.sin(a)), int(c - mask_px * math.cos(a))), (200, 200, 255), 1)
    for a, d, q in to_bert(points):
        if d > max_cm:
            continue
        rad = math.radians(a)
        x, y = int(c - d * scale * math.sin(rad)), int(c - d * scale * math.cos(rad))
        colour = (170, 170, 170) if (q < MIN_QUALITY or is_masked(a, d)) else (0, 0, 220)
        cv2.circle(img, (x, y), 2, colour, -1)
    cv2.arrowedLine(img, (c, c + 12), (c, c - 14), (0, 140, 0), 2, tipLength=0.4)
    cv2.putText(img, "ahead", (c - 18, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 0), 1)
    ok, buf = cv2.imencode(".png", img)
    return buf.tobytes()


if __name__ == "__main__":
    import argparse
    import json
    p = argparse.ArgumentParser(description="Try Large Bert's lidar on its own.")
    p.add_argument("--port", help="serial port (default: find it)")
    p.add_argument("--seconds", type=float, default=SCAN_SECONDS)
    p.add_argument("--raw", action="store_true", help="print every point: Bert's angle, cm, quality")
    p.add_argument("--png", help="save a top-down picture here")
    args = p.parse_args()
    lidar = Lidar(args.port)
    try:
        print(json.dumps(lidar.info, indent=1))
        pts = lidar.read_points(args.seconds)
        if args.raw:
            for a, d, q in sorted(to_bert(pts)):
                print("%7.1f %7.1f %3d%s" % (a, d, q, "  (%s)" % is_masked(a, d) if is_masked(a, d) else ""))
        print(json.dumps(summarise(pts), indent=1))
        if args.png:
            with open(args.png, "wb") as f:
                f.write(render_png(pts))
            print("saved", args.png)
    finally:
        lidar.close()
