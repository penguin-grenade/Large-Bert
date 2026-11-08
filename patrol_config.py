# -*- coding: utf-8 -*-
"""Settings for cat_patrol.py. Edit these to tune the patrol for your yard."""

# ---- Patrol style -------------------------------------------------------
# "sentry": stay in one spot and sweep the camera (saves battery, easier on legs)
# "roam":   wander the yard, avoiding obstacles with the ultrasonic sensor
MODE = "sentry"

# Head (camera) servos. On Large Bert channel 1 turns the head (pan) and channel 0 nods it (tilt).
HEAD_PAN_CHANNEL = 1
HEAD_TILT_CHANNEL = 0
PAN_CENTER = 108    # servo angle that looks straight ahead (calibrated by eye, was 90)
TILT_CENTER = 105   # servo angle that looks level (was 90)
SCAN_ANGLES = [PAN_CENTER + a for a in (0, -25, -40, -25, 0, 25, 40, 25)]   # pan positions while watching
SCAN_DWELL_SECONDS = 1.5                             # how long to look at each position
PAN_SIGN = 1        # set to -1 if the robot turns the wrong way after spotting a cat mid-sweep

# Walking. Forward is +y in the Freenove gait. Speed 2 (slow) .. 10 (fast).
PATROL_SPEED = 6
APPROACH_SPEED = 8
STEP_LENGTH = 30            # 0..35
TURN_SIGN = -1               # set to -1 if "--test-hardware" shows a positive turn going right
DEG_PER_TURN_CYCLE = 31     # roughly how far one turning gait cycle rotates the body
ROAM_STEPS_BETWEEN_SCANS = 6
OBSTACLE_CM = 45            # roam mode: turn away from anything closer than this

# ---- Vision -------------------------------------------------------------
MODEL_DIR = "cat_model"     # created by get_cat_model.sh
REFERENCE_DIR = "cats"      # cats/intruder/*.jpg and cats/mine/*.jpg
CAMERA_SIZE = (640, 480)
CAMERA_HFOV_DEG = 60        # horizontal field of view of the Pi camera
CAT_MIN_SCORE = 0.45        # detector confidence needed to count something as a cat
PERSON_MIN_SCORE = 0.5

# Who is who. With cat_model/cat_embedder.onnx and photos in cats/intruder/ and
# cats/mine/<name>/, each cat is compared with those photos by a small neural net.
# Tested on your 37 photos (each against the rest): none of your cats came out as "intruder"
# (26 "mine", 4 "unsure"), and 5 of 7 orange boy photos did (2 "unsure").
EMBEDDER_FILE = "cat_model/cat_embedder.onnx"
MATCH_TOP_K = 2             # average the best K photo matches per cat
INTRUDER_MARGIN = 0.02      # intruder photos must match at least this much better than any of yours
MINE_MARGIN = 0.05          # one of yours must match at least this much better to count as "mine"

# Orange-coat check (OpenCV hue runs 0..180; orange/ginger is roughly 5..28).
ORANGE_HUE_RANGE = (5, 28)
ORANGE_MIN_SAT = 50
ORANGE_MIN_VAL = 80
ORANGE_GATE = 0.25          # photo match says intruder: also needs this much orange (orange boy's photos: 0.34+)
ORANGE_ONLY_MIN = 0.65      # no photos/model: orange alone must be this high (Jimb's tabby coat reached 0.57 in sun)
MINE_MAX_ORANGE = 0.12      # no photos/model: below this it's treated as one of yours

# A cat is only confirmed as the intruder when this many of the last N looks agree.
VOTE_WINDOW = 6
VOTES_NEEDED = 4
INVESTIGATE_SECONDS = 6

# ---- Deterrence (humane limits) -----------------------------------------
STANDOFF_CM = 60            # never walk closer than this (ultrasonic)
CLOSE_AREA_FRACTION = 0.35  # or if the cat fills this much of the picture
CHASE_SECONDS = 20          # give up after this long
LOST_SECONDS = 3            # cat out of sight this long = it left, success
CORNER_SECONDS = 5          # cat stays put inside the standoff this long = back off, don't corner it
COOLDOWN_SECONDS = 120      # rest between encounters so no cat gets harassed
MAX_ENCOUNTERS_PER_HOUR = 6 # after this many chases in an hour, stand down until the hour is up
IGNORE_SECONDS = 20         # after deciding to leave a cat alone, don't re-check it for this long
HOLD_OFF_FOR_PEOPLE = True  # stop if a person is in view
HOLD_OFF_FOR_MY_CATS = True # stop if one of your cats is in view
CHIRP_EVERY_SECONDS = 2.0
QUIET_HOURS = (22, 7)       # no buzzer between these local hours (lights still work); None to disable

# ---- Safety -------------------------------------------------------------
MIN_LOAD_BATTERY_V = 5.0    # servo battery (2S Li-ion); below this the robot sits down and stops
MIN_PI_BATTERY_V = 6.4
BATTERY_CHECK_SECONDS = 60

# ---- Logging ------------------------------------------------------------
LOG_DIR = "cat_log"         # snapshots and cat crops, handy for building reference photos
SAVE_CROPS = True
MIN_SECONDS_BETWEEN_SAVES = 2.0
