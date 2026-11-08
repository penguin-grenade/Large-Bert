# -*- coding: utf-8 -*-
"""Safety limits and settings for large_bert_mcp.py (Large Bert's MCP server).

Battery cut-offs, quiet hours, turn/pan directions and gait settings are read
from patrol_config.py so the patrol and the MCP server always agree.
"""

# ---- Walking limits -------------------------------------------------------
MAX_STEPS_PER_CALL = 10      # one walk/turn call never does more gait cycles than this
MAX_MOVE_SECONDS = 25        # ...and never runs longer than this
DEFAULT_SPEED = 6            # Freenove speed 2 (slow) .. 10 (fast)
MAX_SPEED = 8                # Claude can't ask for more than this
MAX_STEP_LENGTH = 30         # Freenove allows 35; a bit less is kinder to the legs
MAX_TURN_DEGREES = 180       # biggest single turn() call
CM_PER_STEP_AT_LENGTH_30 = 10.4  # measured: 5 cycles at length 30 went 52 cm (20.5 in); scales with length
MIN_OBSTACLE_CM = 25         # refuse to step forward when the ultrasonic sees something closer

# ---- Idle and battery watchdog --------------------------------------------
IDLE_RELAX_SECONDS = 120     # no command for this long: legs relax (saves battery, cools servos)
WATCHDOG_SECONDS = 5         # how often the watchdog wakes up
BATTERY_CHECK_SECONDS = 30   # how often the battery is read while idle
BATTERY_RECOVER_MARGIN_V = 0.2  # after a low-battery stop, need this much above the cut-off to move again

# ---- Head, body, lights, buzzer -------------------------------------------
PAN_RANGE = (50, 180)        # same limits as the Freenove server (90 = straight ahead)
TILT_RANGE = (50, 180)       # tilt servo is channel 0, which Freenove limits to 50..180
BODY_TILT_LIMIT = 15         # degrees of roll/pitch/yaw for body_tilt (Freenove's own limit)
MAX_BEEPS = 5
MAX_BEEP_SECONDS = 0.5       # longest single beep
LED_MAX_BRIGHTNESS = 255

# ---- Network ---------------------------------------------------------------
HTTP_HOST = "0.0.0.0"        # only used with --http
HTTP_PORT = 8765
TOKEN_FILE = "~/.large_bert_token"   # bearer token for --http, created on first run
