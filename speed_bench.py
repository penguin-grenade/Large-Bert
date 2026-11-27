# -*- coding: utf-8 -*-
"""Compare Freenove's one-byte servo writes with bert_fastservo's block writes on the real robot.

Run on the Pi (nothing else may be driving Bert), from the Code/Server folder:
    sudo /home/pi/bert-venv/bin/python speed_bench.py old:10 fast:10 old:6 fast:6 --steps 4 --back
Each run takes a lidar scan, walks `steps` tripod cycles forward with all the
normal safety checks (lidar guard, ultrasonic, IMU, voice stop), and scans
again; the distance is how far the second scan has to slide to line up with
the first. Between gait frames the servo battery is sampled, so the dip under
load is caught. Prints one JSON line per run and a table at the end.
"""
import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import large_bert_mcp as L


class Sampler:
    """Stands in for control.py's time module: every few sleeps, read the battery."""
    def __init__(self, real, adc, every=3):
        self._real, self._adc, self.every = real, adc, every
        self.on = False
        self.n = 0
        self.volts = []
        self.first = self.last = None

    def __getattr__(self, name):
        return getattr(self._real, name)

    def sleep(self, seconds):
        self._real.sleep(seconds)
        if self.on:
            self.last = time.time()
            if self.first is None:
                self.first = self.last
            self.n += 1
            if self.n % self.every == 0:
                try:
                    self.volts.append(float(self._adc.read_battery_voltage()[0]))
                except Exception:
                    pass


def cloud(bert, seconds=1.5):
    """Lidar points (x right, y ahead, cm) around Bert, without his own body and head."""
    import math
    import bert_lidar as BL
    pts = bert.get_lidar().read_points(seconds)
    out = []
    for a, d, q in BL.to_bert(pts):
        if q >= BL.MIN_QUALITY and not BL.is_masked(a, d) and d < 450:
            r = math.radians(a)
            out.append((-d * math.sin(r), d * math.cos(r)))
    return out


def moved(before, after, max_cm=100.0, cell=2.0):
    """How far forward he went between two scans: slide the second scan forward until it best
    overlaps the first (also trying +-2 deg of rotation). Returns cm, or None if nothing matched."""
    import math
    if not before or not after:
        return None
    grid = set((int(round(x / cell)), int(round(y / cell))) for x, y in before)
    best = (0, None)
    for rot in (-2, -1, 0, 1, 2):
        c, s_ = math.cos(math.radians(rot)), math.sin(math.radians(rot))
        turned = [(x * c - y * s_, x * s_ + y * c) for x, y in after]
        shift = -10.0
        while shift <= max_cm:
            hits = 0
            for x, y in turned:
                gx, gy = int(round(x / cell)), int(round((y + shift) / cell))
                if (gx, gy) in grid or (gx, gy + 1) in grid or (gx, gy - 1) in grid:
                    hits += 1
            if hits > best[0]:
                best = (hits, shift)
            shift += 1.0
    return best[1] if best[0] >= 20 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="writes:speed, e.g. old:10 fast:10")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--length", type=int, default=30)
    ap.add_argument("--back", action="store_true", help="walk back (speed 6) after each run")
    ap.add_argument("--rest", type=float, default=8.0, help="seconds to rest between runs (battery recovers)")
    args = ap.parse_args()

    bert = L.Bert(sim=False, use_ears=True)
    fast = bert.hw._hw.fast_servo
    if fast is None:
        sys.exit("bert_fastservo isn't installed (FAST_SERVO_WRITES off?)")
    import control
    sampler = Sampler(control.time, bert.hw._hw.robot.adc)
    control.time = sampler
    results = []
    try:
        bert.stand_up()
        time.sleep(2)
        for run in args.runs:
            writes, speed = run.split(":")
            speed = int(speed)
            fast.enabled = writes == "fast"
            bert.check_battery(force=True)
            rest_v = bert.last_volts[0]
            before = cloud(bert)
            sampler.volts, sampler.n, sampler.first, sampler.last, sampler.on = [], 0, None, None, True
            text = bert.walk(args.length, 0, args.steps, speed)
            sampler.on = False
            time.sleep(1.5)
            after = cloud(bert)
            v = sampler.volts
            secs = round(sampler.last - sampler.first, 2) if sampler.first else None
            r = {"writes": writes, "speed": speed, "steps": args.steps, "length": args.length,
                 "result": text, "rest_v": rest_v, "seconds": secs, "frames": sampler.n,
                 "ms_per_frame": round(1000 * secs / max(1, sampler.n - 1), 1) if secs else None,
                 "min_v": round(min(v), 2) if v else None,
                 "mean_v": round(statistics.mean(v), 2) if v else None,
                 "samples": len(v), "moved_cm": moved(before, after)}
            if secs and r["moved_cm"]:
                r["cm_per_s"] = round(r["moved_cm"] / secs, 2)
            print(json.dumps(r), flush=True)
            results.append(r)
            if ": done." not in text:
                print("run did not finish, stopping the bench", flush=True)
                break
            if args.back:
                fast.enabled = True
                bert.walk(-args.length, 0, args.steps, 6)
            time.sleep(args.rest)
    finally:
        fast.enabled = True
        bert.stand_up()
        print("\n%-5s %5s %6s %8s %7s %7s %7s %7s %6s" % (
            "write", "speed", "secs", "ms/frame", "cm", "cm/s", "rest V", "min V", "dip V"))
        for r in results:
            dip = round(r["rest_v"] - r["min_v"], 2) if r["rest_v"] and r["min_v"] else None
            print("%-5s %5s %6s %8s %7s %7s %7s %7s %6s" % (
                r["writes"], r["speed"], r["seconds"], r["ms_per_frame"], r["moved_cm"], r.get("cm_per_s"),
                r["rest_v"], r["min_v"], dip))
        bert.close()


if __name__ == "__main__":
    main()
