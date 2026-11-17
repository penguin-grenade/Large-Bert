# -*- coding: utf-8 -*-
"""Large Bert's lidar: an RPLIDAR on the top plate, plugged into the Pi over USB.

Scans are taken on demand (the motor only spins while a scan runs), which keeps the Pi 3's
CPU and the lidar's motor free the rest of the time. It speaks the plain SCAN command, which
every RPLIDAR (A1, A2, A3, C1, S1, S2...) understands at its own baud rate, so the model
is found by asking it, not by settings.

Angles everywhere outside this file are Bert's: 0 = straight ahead, positive = to his LEFT
(anticlockwise seen from above, the same as turn() and look()), -90 = right, +/-180 = behind.

Things it ignores, so they aren't reported as obstacles:
  * the head: it sits in front of the lidar, so the sector straight ahead is blocked
    (HEAD_MASK_DEG). The ultrasonic sensor covers that direction.
  * Bert himself: anything closer than SELF_RADIUS_CM (legs, cables, the top plate).
    Sitting, his legs reach further out than this, so scans taken while relaxed say so.

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
HEAD_MASK_DEG = 30.0      # ignore +/- this many degrees around straight ahead (the head is there)
SELF_RADIUS_CM = 12.0     # ignore points closer than this (Bert's own body)
MAX_RANGE_CM = 1200.0     # ignore anything further (A1 is 12 m, C1 12 m)
MIN_QUALITY = 1           # A-series reports 0 for a failed reading
SCAN_SECONDS = 1.2        # how long one scan() collects; the lidar does ~5-10 turns a second
SPIN_UP_SECONDS = 1.0     # A1/A2 need the motor up to speed before readings are good
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
        self._connect()

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
    def read_points(self, seconds=SCAN_SECONDS):
        """Spin up, collect for `seconds`, stop. Returns [(lidar_angle_deg_clockwise, distance_cm, quality)]."""
        seconds = max(0.3, min(float(seconds), 5.0))
        with self.lock:
            try:
                self._motor(True)
                if self.info["model"] in ("A1", "A2", "A3"):
                    time.sleep(SPIN_UP_SECONDS)
                self.ser.reset_input_buffer()
                self._cmd(0x20)                       # SCAN: 5-byte nodes
                size, _ = self._descriptor()
                if size != 5:
                    raise IOError("unexpected scan format (%d-byte nodes)" % size)
                raw = bytearray()
                end = time.time() + seconds
                while time.time() < end:
                    raw += self.ser.read(max(1, self.ser.in_waiting or 1))
            finally:
                try:
                    self._stop()
                    self._motor(False)
                except Exception:
                    pass
        self.last_scan_time = time.time()
        return parse_nodes(bytes(raw))

    def scan(self, seconds=SCAN_SECONDS, legs_relaxed=None):
        points = self.read_points(seconds)
        return summarise(points, legs_relaxed=legs_relaxed)

    def status(self):
        out = dict(self.info or {})
        out["last_scan"] = time.strftime("%H:%M:%S", time.localtime(self.last_scan_time)) if self.last_scan_time else None
        return out

    def close(self):
        if self.ser is not None:
            try:
                with self.lock:
                    self._stop()
                    self._motor(False)
            except Exception:
                pass
            self.ser.close()
            self.ser = None


def parse_nodes(raw):
    """Standard SCAN nodes, 5 bytes each. Finds the byte alignment by the check bits, drops bad nodes."""
    best, best_ok = 0, -1
    for off in range(5):
        ok = 0
        for i in range(off, min(len(raw) - 4, off + 500), 5):
            b0, b1 = raw[i], raw[i + 1]
            if ((b0 & 1) ^ ((b0 >> 1) & 1)) and (b1 & 1):
                ok += 1
        if ok > best_ok:
            best, best_ok = off, ok
    points = []
    for i in range(best, len(raw) - 4, 5):
        b0, b1, b2, b3, b4 = raw[i:i + 5]
        if not (((b0 & 1) ^ ((b0 >> 1) & 1)) and (b1 & 1)):
            continue
        dist_mm = (b3 | (b4 << 8)) / 4.0
        if dist_mm <= 0:
            continue
        angle = ((b1 >> 1) | (b2 << 7)) / 64.0
        points.append((angle % 360.0, dist_mm / 10.0, b0 >> 2))
    return points


def to_bert(points):
    """Lidar angles (clockwise) -> Bert's angles (0 ahead, + left)."""
    return [(wrap180(-(a - YAW_OFFSET_DEG)), d, q) for a, d, q in points]


def is_masked(bert_angle, dist_cm):
    if abs(bert_angle) <= HEAD_MASK_DEG:
        return "head"
    if dist_cm < SELF_RADIUS_CM:
        return "self"
    if dist_cm > MAX_RANGE_CM:
        return "far"
    return None


def summarise(points, legs_relaxed=None):
    """Closest thing in each of 8 directions, with Bert's own head and body left out."""
    bert = to_bert(points)
    good = [(a, d) for a, d, q in bert if q >= MIN_QUALITY and not is_masked(a, d)]
    sectors = {}
    for name, centre, half in SECTORS:
        hits = [d for a, d in good if abs(wrap180(a - centre)) <= half]
        if abs(centre) < half:                            # "ahead" is mostly head: say what it covers
            lo = HEAD_MASK_DEG
            if lo >= half:
                sectors[name] = {"closest_cm": None, "note": "blocked by the head, use the ultrasonic (status)"}
                continue
        sectors[name] = {"closest_cm": round(min(hits)) if hits else None, "points": len(hits)}
    closest = min(good, key=lambda p: p[1]) if good else None
    out = {
        "points": len(points), "used": len(good),
        "directions": sectors,
        "closest": {"cm": round(closest[1]), "angle_deg": round(closest[0]),
                    "where": direction_name(closest[0])} if closest else None,
        "angles": "0 = straight ahead, positive = left, negative = right, 180 = behind",
        "ignored": "+/-%d deg ahead (the head; the ultrasonic covers it) and anything within %d cm (Bert himself)"
                   % (HEAD_MASK_DEG, SELF_RADIUS_CM),
    }
    if legs_relaxed:
        out["warning"] = ("Bert is sitting, so his legs stick out and may show up as close obstacles. "
                          "Stand up before trusting short readings.")
    if len(points) < 100:
        out["warning"] = (out.get("warning", "") + " Very few points came back; the lidar may not be spinning.").strip()
    return out


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
    mask_px = int(max_cm * scale)
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
