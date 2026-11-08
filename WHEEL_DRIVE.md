# Driving Large Bert with the steering wheel

Drive Bert live from the Thrustmaster T500 RS (or any wheel, gamepad or joystick)
plugged into the Windows PC.

| File | Where it runs | What it does |
|---|---|---|
| `bert_wheel.py` | PC | Reads the wheel and pedals, shows a small dashboard window, streams 20 updates a second |
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
| `S` / `R` key | Stand up / relax the legs |
| `Esc` or close the window | Stop, relax, disconnect |

Let go of both pedals and he stops at the end of the current step (under a second).

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
  hiccup), he stops and stands.
- **Connection lost or window closed:** he relaxes and the lights go off.
- **Obstacle:** forward is blocked while the ultrasonic sees something closer than
  `MIN_OBSTACLE_CM` (25 cm). Turning, reversing and crab-walking still work so you can
  back out. The dashboard turns red and shows the distance.
- **Speed caps:** step length up to `MAX_STEP_LENGTH` (30, about 10 cm a step) and gait speed
  up to `MAX_SPEED` (8), both from `bert_config.py`. Reverse and sideways are capped at 60%.
- **Battery:** the same cut-offs as the MCP server and patrol (`patrol_config.py`), checked
  every few steps. Below them he relaxes and won't walk until he's charged.
- **Gentle start:** the PC won't connect while a pedal reads as pressed, and small
  wheel and pedal movements are softened so he doesn't lurch.
- **Idle:** after 2 minutes without moving, the legs relax.

Tuning lives at the top of `bert_drive.py` (`DEADMAN_SECONDS`, `REVERSE_FRACTION`,
`DEADZONE`, `MIN_GAIT_SPEED`). How much wheel gives a full-rate turn is `"lock"` under
`"steer"` in `bert_wheel.json` (0.3 means about 30% of the wheel's travel).
