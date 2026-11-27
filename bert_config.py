# -*- coding: utf-8 -*-
"""Safety limits and settings for large_bert_mcp.py (Large Bert's MCP server).

Battery cut-offs, quiet hours, turn/pan directions and gait settings are read
from patrol_config.py so the patrol and the MCP server always agree.
"""

# ---- Walking limits -------------------------------------------------------
MAX_STEPS_PER_CALL = 10      # one walk/turn call never does more gait cycles than this
MAX_MOVE_SECONDS = 25        # ...and never runs longer than this
DEFAULT_SPEED = 6            # Freenove speed 2 (slow) .. 10 (fast)
MAX_SPEED = 10               # Freenove's top speed (the wheel drive has used it since PR #4)
MAX_STEP_LENGTH = 30         # Freenove allows 35; a bit less is kinder to the legs
MAX_TURN_DEGREES = 180       # biggest single turn() call
CM_PER_STEP_AT_LENGTH_30 = 10.4  # measured: 5 cycles at length 30 went 52 cm (20.5 in); scales with length
MIN_OBSTACLE_CM = 25         # refuse to step forward when the ultrasonic sees something closer
TIP_STOP_DEG = 30            # IMU: stop a walk if the body tilts this far from level (he sits ~2.5 deg nose-down,
                             # body_tilt allows 15); the IMU reads all through every step, not just between steps
HEADING_HOLD = True          # IMU: walk and cruise steer back to the heading they started on (he drifts ~1.5 deg/step left)
HEADING_HOLD_GAIN = 0.8      # share of the heading error corrected on the next step
HEADING_HOLD_MAX_TURN = 3    # most gait turn per step for the correction (1 = ~3 deg)

# ---- Wave gait (experimental, gait="wave"): one leg in the air at a time, see bert_gait.py ----
WAVE_FRAMES_SLOW = 24        # frames per leg at speed 2 (lift, swing, lower); about 10 ms + servo writes each
WAVE_FRAMES_FAST = 4         # ...at speed 10 (1 lift, 2 swing, 1 lower). Patio 2026-11-27: 4 beat 6
WAVE_STRIDE_FACTOR = 2.5     # foot travel per cycle in step lengths (Freenove's wave: 5/3, tripod: 2). The foot
                             # furthest back reaches stride * factor (75 mm at 30) behind home; 2.5 ran fine
WAVE_DISTANCE_SCALE = 0.55   # measured: a wave cycle goes this share of a tripod cycle's distance (patio,
                             # 2026-11-27, at the settings above); used for the distance and turn estimates
WAVE_FRAME_DELAY = 0.01      # pause after each frame, like Freenove's gait

# ---- cruise: one long lidar-guided walk forward ------------------------------
CRUISE_MAX_CM = 400          # longest single cruise
CRUISE_MAX_STEPS = 40        # ...and never more gait cycles than this
CRUISE_SPEED = 10            # default cruise gait speed (2..MAX_SPEED)
CRUISE_STEP_LENGTH = 35      # cruise stride: Freenove's longest (walk stays at MAX_STEP_LENGTH)
CRUISE_MAX_SECONDS = 120     # ...or longer than this
CRUISE_STOP_CM = 45          # default: stop when something in his path is this close (from the lidar)
CRUISE_MIN_STOP_CM = 30      # Claude can't ask for less than this

# ---- Idle and battery watchdog --------------------------------------------
IDLE_RELAX_SECONDS = 120     # no command for this long: legs relax (saves battery, cools servos)
WATCHDOG_SECONDS = 5         # how often the watchdog wakes up
BATTERY_CHECK_SECONDS = 30   # how often the battery is read while idle
BATTERY_RECOVER_MARGIN_V = 0.2  # after a low-battery stop, need this much above the cut-off to move again
BATTERY_WARN_V = 7.0         # servo battery below this: status and walks say it's getting low (weaker, more drift)

# ---- Head, body, lights, buzzer -------------------------------------------
PAN_RANGE = (50, 180)        # same limits as the Freenove server (90 = straight ahead)
TILT_RANGE = (50, 180)       # tilt servo is channel 0, which Freenove limits to 50..180
BODY_HEIGHT_MM = 25          # normal standing height: how far the feet sit below the body (Freenove default)
BODY_HEIGHT_RANGE_MM = (25, 70)  # set_height limits; 50-60 gives more ground clearance outside
STEP_LIFT_MM = 40            # how high each foot lifts while walking (Freenove default)
STEP_LIFT_RANGE_MM = (20, 60)
BODY_TILT_LIMIT = 15         # degrees of roll/pitch/yaw for body_tilt (Freenove's own limit)
MAX_BEEPS = 5
MAX_BEEP_SECONDS = 0.5       # longest single beep
LED_MAX_BRIGHTNESS = 255

# ---- Network ---------------------------------------------------------------
HTTP_HOST = "0.0.0.0"        # only used with --http
HTTP_PORT = 8765
TOKEN_FILE = "~/.large_bert_token"   # bearer token for --http, created on first run
