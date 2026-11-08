# Driving Large Bert with the steering wheel

Drive Bert live from the Thrustmaster T500 RS (or any wheel, gamepad or joystick)
plugged into the Windows PC.

| File | Where it runs | What it does |
|---|---|---|
| `bert_wheel.py` | PC | Reads the wheel and pedals, shows Bert's head camera with a dashboard on top, streams 20 updates a second |
| `bert_drive.py` | Pi (`Code/Server`) | Started by `bert_wheel.py` over SSH. Turns the input into gait cycles with safety checks |

There's no open port. The PC starts the Pi side over the same passwordless SSH that
the MCP server uses (`LARGE_BERT_MCP.md`, step 4).

## Controls

| Control | Bert does |
|---|---|
| Accelerator | Walks forward. Further down means longer, faster steps |
| Brake | Walks backwards, at most 60% speed (he has no rear sensor) |
| Wheel | Turns. With no pedal pressed he turns on the spot; with a pedal he curves |
| Paddles (if set up) | Crab-walks left or right at up to 60% speed |
| Horn button (if set up) | A short beep. Silent during quiet hours (22:00 to 07:00) |
| D-pad on the wheel (or arrow keys) | Turns his head to look around. `C` points it straight ahead again |
| `S` / `R` key | Stand up / relax the legs |
| `Esc` or close the window | Stop, relax, disconnect |

Let go of both pedals and he stops mid-stride, within about half a second, and puts his feet back under him.
A step takes 1 to 3 seconds, so changing direction also cuts the current step short and starts a new one.

## Video

The window shows Bert's head camera live, about 12 frames a second, over the same SSH
link (about 300 KB/s on the home Wi-Fi). Drag the window bigger or maximise it; the picture
keeps its shape. If Wi-Fi slows down, frames are dropped rather than delaying the controls.
The dashboard shows a "Video paused" line if no picture has arrived for over 1.5 s.
Run the Pi side with `--no-video` to turn it off.

## Setup

1. Copy `bert_drive.py` to the Pi, next to `large_bert_mcp.py`:
   ```powershell
   scp bert_drive.py pi@192.168.0.119:~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_pi/Code/Server/
   ```
2. On the PC: `pip install pygame`
3. Make sure nothing else is using Bert: close any Claude session that's connected to the
   `large-bert` MCP server, and stop `main.py` or `cat_patrol.py`. If something still has him,
   the window shows "Bert is busy" and says which program has him. The drive program never
   stops another program by itself.
4. Run `python bert_wheel.py`. The first time, it walks you through setup in the window:
   turn the wheel lock to lock, press each pedal, show it which way is left, then pick the
   paddles and horn (press Enter to skip any). The setup is saved in `bert_wheel.json`.
   Run `python bert_wheel.py --calibrate` to do it again.

The T500 reports 0 for each pedal until that pedal first moves. So if the window says
"Let go of the pedals" when you aren't touching them, tap each pedal once.

`python bert_wheel.py --sim` drives a pretend Bert on the Pi (nothing moves, the MCP server
can keep running). `--no-robot` only shows the wheel inputs.

## Safety

- **Deadman:** if the Pi gets no input for 0.5 s (program frozen, wheel unplugged, Wi-Fi
  hiccup), he stops mid-stride and stands. Tested on the real robot: stopped 0.5 s after input ended.
- **Connection lost or window closed:** he relaxes and the lights go off.
- **Obstacle:** checked 10 times a second while stepping forward. Forward is blocked while the ultrasonic sees something closer than
  `MIN_OBSTACLE_CM` (25 cm). Turning, reversing and crab-walking still work so you can
  back out. The dashboard turns red and shows the distance.
- **Speed caps:** with the pedal floored, step length 35 (about 12 cm a step) and gait speed 10,
  Freenove's own maximums (`DRIVE_MAX_STEP`, `DRIVE_MAX_SPEED` in `bert_drive.py`). That's
  roughly twice as fast as Claude's MCP limits in `bert_config.py`, which stay as they were.
  Reverse and sideways are capped at 60%. Fast walking pulls the battery down harder.
- **Battery:** the same cut-offs as the MCP server and patrol (`patrol_config.py`), checked
  every few steps. Below them he relaxes and won't walk until he's charged.
- **Gentle start:** the PC won't connect while a pedal reads as pressed, and small
  wheel and pedal movements are softened so he doesn't lurch.
- **Idle:** after 2 minutes without moving, the legs relax.

Tuning lives at the top of `bert_drive.py` (`DEADMAN_SECONDS`, `REVERSE_FRACTION`,
`DEADZONE`, `MIN_GAIT_SPEED`). How much wheel gives a full-rate turn is `"lock"` under
`"steer"` in `bert_wheel.json` (0.3 means about 30% of the wheel's travel).
