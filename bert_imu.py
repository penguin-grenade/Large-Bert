# -*- coding: utf-8 -*-
"""Large Bert's IMU: a BNO085 in UART-RVC mode, on a CP2102 USB serial adapter plugged into the Pi.

Wiring (2026-11-20): BNO VIN <- adapter +5V, GND <- GND, BNO SDA (= TX in RVC mode) -> adapter RXD,
and BNO P0 jumpered to BNO 3Vo (that picks RVC mode at power-up). Nothing on the Pi's GPIO header.

In RVC mode the BNO streams ~100 packets a second without being asked: 0xAA 0xAA, then index,
yaw, pitch, roll (int16, 0.01 degree), x/y/z acceleration (int16, mg), 3 spare bytes and a checksum.
Yaw is positive = LEFT (anticlockwise seen from above, like turn()), relative to where he faced at power-up (no magnetometer in RVC mode), so it's good for
"how far have I turned" but not for compass headings.

Try it on its own (stop the MCP server first: only one program can open the port):
    python3 bert_imu.py            # a few seconds of readings
"""
import glob
import math
import os
import struct
import threading
import time

# ---- settings ---------------------------------------------------------------
PORT = os.environ.get("BERT_IMU_PORT", "")   # empty: find it (see find_port)
# The adapter's fixed name (a plain CP2102 with the generic serial 0001). /dev/ttyUSB0/1 can swap
# between boots with the lidar's adapter, so always open it by this name.
BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
BAUD = 115200
YAW_SIGN = -1             # the BNO counts left turns as negative (checked 2026-11-20: a hand turn left read
                          # -44 deg); flipped so yaw is Bert's convention everywhere: positive = LEFT
STALE_SECONDS = 0.5       # a reading older than this means the stream stopped
SMOOTH_SECONDS = 0.1      # average this much of the newest stream (~10 packets) for pitch/roll/accel
UPRIGHT_MAX_TILT_DEG = 45 # more tilt than this from level and he's probably fallen over


def _log(message):
    import sys
    sys.stderr.write("[imu] %s\n" % message)
    sys.stderr.flush()


def find_port():
    if PORT:
        return PORT
    if os.path.exists(BY_ID):
        return BY_ID
    seen = ", ".join(os.path.basename(p) for p in sorted(glob.glob("/dev/serial/by-id/*"))) or "none"
    raise RuntimeError("no IMU adapter at %s (USB serial devices: %s). Is it plugged in?" % (BY_ID, seen))


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def body_tilt(pitch, roll):
    """Degrees between the body's up and true up, from the BNO's fused pitch and roll (so walking
    jolts, which spike the raw accelerometer, don't count as tilt)."""
    c = math.cos(math.radians(pitch)) * math.cos(math.radians(roll))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def parse_packet(body):
    """body: the 17 bytes after 0xAA 0xAA. Returns (yaw, pitch, roll, ax, ay, az) in degrees and g, or None."""
    if len(body) != 17 or sum(body[:16]) & 0xFF != body[16]:
        return None
    _, yaw, pitch, roll, ax, ay, az = struct.unpack("<Bhhhhhh", body[:13])
    return yaw / 100.0, pitch / 100.0, roll / 100.0, ax / 1000.0, ay / 1000.0, az / 1000.0


class Imu:
    def __init__(self, port=None, log=_log):
        import serial
        self.log = log
        self.port = port or find_port()
        self.ser = serial.Serial(self.port, BAUD, timeout=0.2)
        self.lock = threading.Lock()
        self.history = []          # (time, yaw, pitch, roll, ax, ay, az), newest last, ~1 s kept
        self.yaw_total = 0.0       # yaw unwrapped (keeps counting past +/-180), for "how far turned"
        self.peak_tilt = 0.0       # most tilt from level since mark(), from every packet
        self._last_yaw = None
        self.packets = 0
        self.bad = 0
        self.running = True
        self.error = None
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        log("IMU on %s" % self.port)

    def _reader(self):
        buf = b""
        while self.running:
            try:
                # whatever has arrived (at least 1 byte, waiting up to the 0.2 s timeout): a fixed
                # read(256) would wait for ~13 packets and timestamp them in bursts
                chunk = self.ser.read(max(1, self.ser.in_waiting))
            except Exception as exc:        # unplugged
                self.error = str(exc)
                self.running = False
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b"\xaa\xaa")
                if start < 0:
                    buf = buf[-1:]
                    break
                if len(buf) < start + 19:
                    buf = buf[start:]
                    break
                reading = parse_packet(buf[start + 2:start + 19])
                if reading is None:         # a stray 0xAA 0xAA inside a packet: slide on by one
                    self.bad += 1
                    buf = buf[start + 1:]
                    continue
                buf = buf[start + 19:]
                reading = (YAW_SIGN * reading[0],) + reading[1:]
                now = time.monotonic()
                yaw, pitch, roll = reading[0], reading[1], reading[2]
                tilt = body_tilt(pitch, roll)
                with self.lock:
                    if self._last_yaw is not None:
                        self.yaw_total += wrap180(yaw - self._last_yaw)
                    self._last_yaw = yaw
                    if tilt > self.peak_tilt:
                        self.peak_tilt = tilt
                    self.packets += 1
                    self.history.append((now,) + reading)
                    while self.history and now - self.history[0][0] > 1.0:
                        self.history.pop(0)

    def read(self):
        """The newest orientation, smoothed over SMOOTH_SECONDS. Raises if the stream has stopped."""
        with self.lock:
            hist = list(self.history)
        if not hist or time.monotonic() - hist[-1][0] > STALE_SECONDS:
            raise RuntimeError(self.error or "no readings from the IMU for over %.1f s" % STALE_SECONDS)
        newest = hist[-1][0]
        recent = [h for h in hist if newest - h[0] <= SMOOTH_SECONDS]
        n = float(len(recent))
        pitch, roll, ax, ay, az = (sum(h[i] for h in recent) / n for i in (2, 3, 4, 5, 6))
        g = math.sqrt(ax * ax + ay * ay + az * az)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, az / g)))) if g > 0.1 else None
        span = hist[-1][0] - hist[0][0]
        return {
            "yaw_deg": round(hist[-1][1], 1),
            "pitch_deg": round(pitch, 1),
            "roll_deg": round(roll, 1),
            "tilt_from_level_deg": None if tilt is None else round(tilt, 1),
            "upright": tilt is not None and tilt < UPRIGHT_MAX_TILT_DEG,
            "accel_g": {"x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3), "total": round(g, 3)},
            "rate_hz": round((len(hist) - 1) / span) if span > 0 else None,
            "age_ms": int((time.monotonic() - newest) * 1000),
        }

    def mark(self):
        """Start watching a move: returns the unwrapped yaw now, and resets the peak tilt."""
        with self.lock:
            self.peak_tilt = 0.0
            return self.yaw_total

    def watch(self):
        """(unwrapped yaw, peak tilt since mark(), tilt now) while moving. Raises if the stream stopped."""
        with self.lock:
            fresh = self.history and time.monotonic() - self.history[-1][0] <= STALE_SECONDS
            if not fresh:
                raise RuntimeError(self.error or "no readings from the IMU for over %.1f s" % STALE_SECONDS)
            newest = self.history[-1]
            return self.yaw_total, self.peak_tilt, body_tilt(newest[2], newest[3])

    def status(self):
        return {"open": self.running, "port": self.port, "packets": self.packets,
                "bad_packets": self.bad, "error": self.error}

    def close(self):
        self.running = False
        try:
            self.thread.join(timeout=1)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    imu = Imu()
    time.sleep(0.5)
    try:
        for _ in range(10):
            print(imu.read())
            time.sleep(0.3)
        print(imu.status())
    finally:
        imu.close()
