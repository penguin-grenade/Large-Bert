# Large Bert's lidar

Large Bert has a **Slamtec RPLIDAR C1** on his top plate, plugged into the Pi over USB (its
CP2102N adapter shows up as `/dev/ttyUSB0`, 460800 baud). It sees all the way round him out
to about 12 m, so it covers his sides and back, which the camera and the ultrasonic sensor
can't see.

The code is `bert_lidar.py`. The MCP server opens the lidar the first time `lidar_scan` is
called. If it's missing or unplugged, the tool says so and everything else still works
(`status` shows `lidar`).

## The `lidar_scan` tool

It returns the closest thing in 8 directions (ahead, ahead_left, left, behind_left, behind,
behind_right, right, ahead_right) in cm, the single closest point, and the ultrasonic reading
straight ahead. Angles are Bert's: 0 = ahead, positive = left, negative = right, 180 = behind,
the same as `turn` and `look`. `picture=true` adds a top-down map: Bert in the middle facing up,
rings every 50 cm, red = obstacles, grey = ignored points.

- The first scan takes about 3 s while the C1's motor spins up. The motor then keeps spinning
  for 30 s after the last scan, so later scans take about 0.6 s. After that it stops to rest.
- **The head**: it sits just in front of the lidar and shows up 12 to 19 cm away, within about
  10 degrees of straight ahead. Points there closer than 25 cm (within 15 degrees) are dropped.
  Further out the lidar sees past the head, and the ultrasonic covers the gap.
- **His legs** stay below the scan plane, even sitting, so nothing else of Bert shows up.
  Anything closer than 12 cm all round is dropped anyway.

Measured 2026-11-16: the lidar's 0 degrees already points straight ahead (`YAW_OFFSET_DEG = 0`).

## Watching while he walks

Every `walk` uses the lidar too (no separate scan, no stopping). While the motor spins, a reader
thread keeps the newest readings, and before each gait cycle the walk checks the last 0.3 s:

- **In his path**: anything within `LOOKAHEAD_CM` past his feet, in the strip as wide as he is
  (going forward or back) or as long as he is (crab-walking), stops the walk.
- **Brushing his sides**: anything within 3 cm of his side feet stops it too (the door-frame case).
- The result ends with the closest distance in each direction, so Claude can chain walks.
- **`follow_walls=True`** (hallways): each step adds a small turn (at most 9 degrees) to stay
  parallel to the walls and drift back to the middle. It finds the straightest line through the wall points on
  each side; furniture that isn't a straight line is ignored. This replaces stopping to correct
  his drift to the left.

**`cruise(distance_cm, speed, stop_cm, follow_walls)`** is one long walk forward (up to 400 cm,
40 steps, 120 s, default speed 10 with a 35 stride) with all of the above every step and wall-following on by default. It stops by
itself when something in his path is closer than `stop_cm` to the lidar (default 45, at least 30).
Use it for a hallway instead of many short walks. Its limits are in `bert_config.py`.

Bert's size is set by `BODY_HALF_LENGTH_CM` (22) and `BODY_HALF_WIDTH_CM` (24), measured from the
lidar to his foot tips while standing. They're estimates: measure him and adjust.

The first walk waits about 3 s for the lidar to spin up. If the lidar is missing, walks still
work with only the ultrasonic, and the result says so.

## Try it on its own

Only one program can open the lidar, so stop the MCP server first (or let it be: it only
holds the port after the first `lidar_scan`).
```
~/bert-venv/bin/python bert_lidar.py                  # model, health and one scan summary
~/bert-venv/bin/python bert_lidar.py --raw            # every point: Bert's angle, cm, quality
~/bert-venv/bin/python bert_lidar.py --png scan.png   # also save the top-down picture
```

## Settings (top of `bert_lidar.py`)

`BERT_LIDAR_PORT` (environment) picks the port by hand. Otherwise it uses the first entry in
`/dev/serial/by-id` that looks like a lidar adapter. **If another CP210x USB serial adapter is
added later (for example for the BNO085 IMU), set `BERT_LIDAR_PORT` to the lidar's by-id path**:
`/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_6c9d5bfce731f1118411935f30d20014-if00-port0`.

`YAW_OFFSET_DEG`, `HEAD_MASK_DEG`, `HEAD_MASK_CM`, `SELF_RADIUS_CM`, `SCAN_SECONDS` and
`KEEP_SPINNING_SECONDS` are there too. It uses the plain SCAN command, so other RPLIDARs
(A1, A2, S-series) should work as well; it finds the baud rate by asking.

It needs only `pyserial` (already in `~/bert-venv`) and OpenCV for the picture. No ROS:
the Pi 3 doesn't have the memory for it.
