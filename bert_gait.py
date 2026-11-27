# -*- coding: utf-8 -*-
"""Experimental wave gait for Large Bert: one leg in the air at a time.

Freenove's own gait (control.run_gait gait "1") is a tripod: three legs swing
while the other three carry the whole robot. In a wave gait five legs always
carry him and only one swings, so each servo works less hard and the battery
should sag less. The price is the duty factor: every leg spends 5/6 of the
cycle on the ground (tripod: 1/2), so a cycle covers less ground per second
unless the single swinging leg moves very fast.

Freenove has a wave gait too (run_gait gait "2"), but its frame counts are
rounded independently, so a cycle doesn't always come back to the stance it
started from and the next cycle starts with a jolt. This one uses exact frame
counts, so every cycle ends on the home stance, and it lets the swing be much
quicker at high speed.

One cycle, for each leg in turn (order WAVE_ORDER):
    lift  - the leg rises `lift` mm             (lift_frames)
    swing - it moves forward by the full stride (swing_frames)
    lower - it comes back down                  (lift_frames)
while the five legs on the ground slide back by 1/5 of the stride each phase,
which pushes the body forward. A leg's foot starts at home, goes back by the
share of stride its earlier neighbours pushed, swings, and is pushed back to
home again by the end of the cycle.
"""
import copy
import math

try:
    import bert_config as B
except Exception:                      # works on its own (tests) too
    B = None


def setting(name, default):
    return getattr(B, name, default) if B is not None else default


# Leg indexes as in control.body_points (0 = front right, 1 = middle right, 2 = back right,
# 3 = back left, 4 = middle left, 5 = front left). Freenove's wave order: alternate sides,
# so the legs either side of a lifted leg are always down.
WAVE_ORDER = (5, 2, 1, 0, 3, 4)


def frames_for(speed):
    """Frames per leg phase for gait speed 2 (slow) .. 10 (fast)."""
    slow = setting("WAVE_FRAMES_SLOW", 24)
    fast = setting("WAVE_FRAMES_FAST", 4)
    speed = max(2, min(10, speed))
    return max(3, int(round(slow + (fast - slow) * (speed - 2) / 8.0)))


def plan(speed):
    """(lift_frames, swing_frames) for one leg's phase."""
    n = frames_for(speed)
    lift = max(1, n // 4)
    swing = max(1, n - 2 * lift)
    return lift, swing


def stride_vectors(body_points, x, y, angle):
    """Per-leg stride (dx, dy) in mm for one cycle, like run_gait's xy but for the whole cycle.
    x = sideways, y = forward (step length, -35..35), angle = turn value (-10..10, + = Freenove's sign)."""
    factor = setting("WAVE_STRIDE_FACTOR", 2.5)   # foot travel per cycle, in step lengths
    a = angle / 180.0 * math.pi
    out = []
    for px, py, _ in body_points:
        dx = (px * math.cos(a) + py * math.sin(a) - px) + x
        dy = (-px * math.sin(a) + py * math.cos(a) - py) + y
        out.append((factor * dx, factor * dy))
    return out


def wave_cycle(control, x, y, speed, angle, lift=40, sleep=None, frame_hook=None):
    """One wave-gait cycle on a Freenove control.Control. Same arguments as run_gait:
    x sideways, y forward (-35..35), speed 2..10, angle turn (-10..10).
    sleep defaults to control.py's own time.sleep (so bert_drive's interruptible
    sleep works here too). frame_hook(frame_index) is called after every frame."""
    import sys
    if sleep is None:
        ctl_module = sys.modules.get(type(control).__module__)
        sleep = getattr(getattr(ctl_module, "time", None), "sleep", None)
        if sleep is None:
            import time
            sleep = time.sleep
    delay = setting("WAVE_FRAME_DELAY", 0.01)
    x = max(-35, min(35, int(x)))
    y = max(-35, min(35, int(y)))
    home = copy.deepcopy(control.body_points)
    points = copy.deepcopy(home)
    if x == 0 and y == 0 and angle == 0:
        control.transform_coordinates(points)
        control.set_leg_angles()
        return
    stride = stride_vectors(home, x, y, angle)
    lift_frames, swing_frames = plan(speed)
    phase_frames = 2 * lift_frames + swing_frames
    push = 5 * phase_frames               # frames each leg spends pushing per cycle
    ground = home[0][2]
    frame = 0
    for leg in WAVE_ORDER:
        for j in range(phase_frames):
            for k in range(6):
                if k == leg:
                    if j < lift_frames:
                        points[k][2] = ground + lift * (j + 1) / lift_frames
                    elif j < lift_frames + swing_frames:
                        points[k][0] += stride[k][0] / swing_frames
                        points[k][1] += stride[k][1] / swing_frames
                    else:
                        points[k][2] = ground + lift * (phase_frames - 1 - j) / lift_frames
                else:
                    points[k][0] -= stride[k][0] / push
                    points[k][1] -= stride[k][1] / push
            control.transform_coordinates(points)
            control.set_leg_angles()
            if frame_hook is not None:
                frame_hook(frame)
            frame += 1
            sleep(delay)
    # rounding can leave a hair of drift: finish exactly on the home stance
    control.transform_coordinates(home)
    control.set_leg_angles()

