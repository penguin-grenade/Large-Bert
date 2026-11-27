# -*- coding: utf-8 -*-
"""Compare gaits on the real robot: time, distance and servo battery sag.

Run on the Pi (nothing else may be driving Bert), from the Code/Server folder:
    sudo /home/pi/bert-venv/bin/python gait_bench.py tripod:10 wave:10 wave:6 --steps 4
Each run: measure where the walls are (lidar), walk `steps` cycles forward with
all the normal safety checks (lidar guard, ultrasonic, IMU, voice stop), then
measure again. Between every gait frame the servo battery is sampled, so the
sag under load is caught, not just the resting voltage. Prints one JSON line
per run and a table at the end. --back walks back afterwards to the start.
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


def walls(bert, seconds=1.0):
    """Median lidar distance (cm) ahead, behind, left and right."""
    import bert_lidar as BL
    pts = bert.get_lidar().read_points(seconds)
    out = {}
    for name, centre in (("ahead", 0), ("left", 90), ("behind", 180), ("right", -90)):
        ds = [d for a, d, q in BL.to_bert(pts)
              if q >= BL.MIN_QUALITY and not BL.is_masked(a, d) and abs(BL.wrap180(a - centre)) <= 4]
        out[name] = round(statistics.median(ds), 1) if ds else None
    return out


def moved(before, after):
    """Forward travel from the walls ahead/behind (whichever both scans saw)."""
    vals = []
    if before.get("ahead") and after.get("ahead"):
        vals.append(before["ahead"] - after["ahead"])
    if before.get("behind") and after.get("behind"):
        vals.append(after["behind"] - before["behind"])
    return round(sum(vals) / len(vals), 1) if vals else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="gait:speed, e.g. tripod:10 wave:10")
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--length", type=int, default=30)
    ap.add_argument("--back", action="store_true", help="walk back with the tripod gait after each run")
    ap.add_argument("--rest", type=float, default=8.0, help="seconds to rest between runs (battery recovers)")
    args = ap.parse_args()

    bert = L.Bert(sim=False, use_ears=True)
    import control
    sampler = Sampler(control.time, bert.hw._hw.robot.adc)
    control.time = sampler
    import bert_gait
    results = []
    try:
        bert.stand_up()
        time.sleep(2)
        for run in args.runs:
            gait, speed = run.split(":")
            speed = int(speed)
            bert.check_battery(force=True)
            rest_v = bert.last_volts[0]
            before = walls(bert)
            sampler.volts, sampler.n, sampler.first, sampler.last, sampler.on = [], 0, None, None, True
            text = bert.walk(args.length, 0, args.steps, speed, gait=gait)
            sampler.on = False
            time.sleep(1.5)
            after = walls(bert)
            v = sampler.volts
            r = {"gait": gait, "speed": speed, "steps": args.steps, "length": args.length,
                 "result": text, "rest_v": rest_v,
                 "min_v": round(min(v), 2) if v else None,
                 "mean_v": round(statistics.mean(v), 2) if v else None,
                 "samples": len(v), "before": before, "after": after, "moved_cm": moved(before, after)}
            # first to last gait frame (both gaits sleep 10 ms per frame)
            secs = round(sampler.last - sampler.first, 2) if sampler.first else None
            r["seconds"] = secs
            if secs and r["moved_cm"]:
                r["cm_per_s"] = round(r["moved_cm"] / secs, 2)
            if gait == "wave":
                r["frames_per_leg"] = bert_gait.frames_for(speed)
            print(json.dumps(r), flush=True)
            results.append(r)
            if ": done." not in text:
                print("run did not finish, stopping the bench", flush=True)
                break
            if args.back:
                bert.walk(-args.length, 0, args.steps, 6)
            time.sleep(args.rest)
    finally:
        bert.stand_up()
        print("\n%-7s %5s %6s %7s %7s %7s %7s %6s" % ("gait", "speed", "secs", "cm", "cm/s", "rest V", "min V", "sag V"))
        for r in results:
            sag = round(r["rest_v"] - r["min_v"], 2) if r["rest_v"] and r["min_v"] else None
            print("%-7s %5s %6s %7s %7s %7s %7s %6s" % (r["gait"], r["speed"], r.get("seconds"), r["moved_cm"],
                                                     r.get("cm_per_s"), r["rest_v"], r["min_v"], sag))
        bert.close()


if __name__ == "__main__":
    main()
