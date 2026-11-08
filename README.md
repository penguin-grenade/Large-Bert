# Large Bert

Large Bert is a Freenove Big Hexapod (Raspberry Pi) that Claude can drive through MCP.
He also runs a humane yard patrol that shoos away one orange intruder cat.

These files go in the robot's `Code/Server` folder on the Pi, next to Freenove's own code:
`~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_pi/Code/Server`.

| File | What it is |
|---|---|
| `large_bert_mcp.py` | The MCP server: walk, turn, head, camera, lights, buzzer, battery and patrol tools, all with safety limits |
| `bert_config.py` | The MCP server's safety limits and settings |
| `cat_patrol.py`, `cat_vision.py` | The cat patrol and its camera/cat-recognition code |
| `patrol_config.py` | Patrol settings plus calibration shared with the MCP server: head centre and channels, turn size, battery cut-offs |
| `control.py` | Freenove's leg control, with `np.mat` replaced by `np.asmatrix` so body tilt works on NumPy 2 (same as Freenove's current upstream version) |
| `get_cat_model.sh` | Downloads the cat detector model (once) |
| `mcp.json` | Claude Code config that starts the server over SSH. Save it as `.mcp.json` in the folder you open Claude Code in |
| `LARGE_BERT_MCP.md` | Install and usage guide for the MCP server |
| `CAT_PATROL.md` | Guide for the cat patrol |
| `bert_drive.py` | Live drive listener: lets the PC's steering wheel and pedals walk him, with a deadman stop, speed caps and the ultrasonic obstacle stop |
| `bert_wheel.py` | Runs on the **Windows PC**, not the Pi: reads the wheel/pedals with pygame and drives Bert over SSH |
| `WHEEL_DRIVE.md` | Guide for driving him with the wheel |

Not in the repo: `bert_wheel.json` (the PC's saved wheel setup), the reference cat photos (`cats/`), the models (`cat_model/`), and patrol logs.

## Calibration (measured 2026-11-08)
- Head: pan is servo channel 1, tilt is channel 0. Straight ahead is pan 108, tilt 105.
- Turning: about 31° per gait cycle (`DEG_PER_TURN_CYCLE`).
- Walking: about 10.4 cm (4.1 in) per gait cycle at step length 30 (`CM_PER_STEP_AT_LENGTH_30`).
- Relax switches servo power off (GPIO4), because the servos hold position without a signal.
- Battery cut-offs: 5.0 V load, 6.4 V Pi.
