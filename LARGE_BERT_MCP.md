# Large Bert MCP server

`large_bert_mcp.py` turns Large Bert into an MCP server, so Claude can walk him, turn
his head, take pictures, set his lights, beep, check the battery, and start or stop the
cat patrol.

## How Claude reaches him

The server runs **on the Pi**. Claude Code or Claude Desktop on your Windows PC starts
it over **SSH**. This is the default, and it's what `.mcp.json` sets up.

| Option | Good | Not so good |
|---|---|---|
| **SSH (default)** | No open ports. Uses your SSH key. If the PC or Wi-Fi drops, the server exits and Bert relaxes. | Only works from a computer on your home network. |
| `--http` on the LAN | Any MCP client on the network can connect. Needs a bearer token. | It's another open port, and the traffic isn't encrypted. |
| Public tunnel (cloudflared, Tailscale Funnel) | Works from anywhere. | Puts a walking robot on the internet. Not recommended. |

**To let Claude in the project chat drive him**, ask it to start a Remote Control
session on your PC in the `Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi-master` folder.
That runs Claude Code on your PC, which reads `.mcp.json` and connects to Bert over SSH.
No tunnel is needed.

## Install (about 10 minutes)

### 1. Copy the files to the Pi
`large_bert_mcp.py`, `bert_config.py` and this guide are in your PC's `Code\Server` folder,
next to the cat patrol files. In **PowerShell** on your PC:

```powershell
cd C:\Users\hungr\Downloads\Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi-master\Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi-master\Code\Server
scp large_bert_mcp.py bert_config.py LARGE_BERT_MCP.md pi@192.168.0.119:~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_pi/Code/Server/
```

(The cat patrol files should already be there. The server uses them for `who_is_there`
and `patrol_start`, and reads the battery cut-offs from `patrol_config.py`.)

### 2. Install the MCP library on the Pi
In a terminal on the Pi (over VNC):

```bash
python3 -m venv --system-site-packages ~/bert-venv
~/bert-venv/bin/pip install mcp
```

`--system-site-packages` lets the venv use the Pi's OpenCV and camera libraries. The
`mcp` library needs Python 3.10 or newer. Raspberry Pi OS Bookworm has 3.11.

### 3. Self-test on the Pi
Stop `main.py` and `cat_patrol.py` first, because they use the same servos and camera. Then:

```bash
cd ~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_pi/Code/Server
sudo ~/bert-venv/bin/python large_bert_mcp.py --check
```

Bert prints the battery, looks left, flashes blue, stands, takes one step, turns a little
left, saves `large_bert_check.jpg`, and relaxes. Use `--check --no-move` to skip walking.

### 4. Passwordless SSH from your PC
In PowerShell (press Enter at each question):

```powershell
ssh-keygen -t ed25519
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh pi@192.168.0.119 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
ssh pi@192.168.0.119 echo hello
```

The last line must print `hello` **without asking for a password**. If it asks to trust
the host, type `yes`.

### 5. Connect Claude
- **Claude Code:** `.mcp.json` is already in the outer `Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi-master`
  folder. Open Claude Code there, approve the `large-bert` server when asked, and check `/mcp`.
- **Claude Desktop:** Settings, Developer, Edit Config. Paste the `large-bert` entry from `.mcp.json`
  inside `"mcpServers"`, then restart Claude Desktop.

Then try: *"Check Large Bert's battery, take a picture, and walk him two steps forward."*

## Tools

| Tool | What it does |
|---|---|
| `status` | Battery, distance ahead, legs up or relaxed, head angle, patrol state, limits |
| `walk` | 1 to 10 gait cycles forward, back or sideways |
| `turn` | Turn on the spot, positive is left |
| `stop` | Emergency stop. Also ends the patrol |
| `stand_up` / `relax` | Stand level, or let the servos go limp |
| `look` | Point the head (pan and tilt in degrees from straight ahead) |
| `snapshot` | Photo from the head camera. Works during the patrol |
| `who_is_there` | Runs the cat detector: orange intruder, one of yours, or unsure |
| `body_tilt` | Lean the body up to 15 degrees |
| `set_height` | Stand taller (feet 25 to 70 mm below the body) and lift the feet higher, for grass and gravel outside |
| `lights` / `beep` | LEDs by colour name or hex, and short beeps |
| `patrol_start` / `patrol_stop` / `patrol_status` | The cat patrol. It starts as a dry run unless Claude passes `dry_run=false` |
| `listen` | Hear what people near Bert say (offline speech-to-text on the Pi). See [VOICE.md](VOICE.md) |

## Safety limits (in `bert_config.py`)
- Each walk or turn is capped at **10 gait cycles and 25 seconds**, speed 8, step length 30.
- Going forward, he stops if the ultrasonic sees something **closer than 25 cm**.
- **`stop`** interrupts a walk between steps. Pressing **Esc** in Claude Code does too.
- Saying **"Bert, stop"** near his microphone stops him on the Pi, without waiting on Claude ([VOICE.md](VOICE.md)).
- The battery is checked before every move and every 30 seconds. **Below 5.0 V (load) or
  6.4 V (Pi)** he relaxes and refuses to move until it's 0.2 V above the cut-off. These
  values come from `patrol_config.py`.
- After **2 minutes without a command**, the legs relax.
- If the SSH connection drops or the server quits, he relaxes and the lights turn off.
- The buzzer stays silent during quiet hours (22:00 to 07:00).
- While the patrol runs, Claude can only take snapshots, check status, or stop it.
  The patrol keeps all its humane limits.

## Other ways to run it
- `python3 large_bert_mcp.py --sim` runs a pretend robot on any computer, for trying it out.
- `--no-move` allows looking, lights and status only.
- `--http` serves on port 8765. The bearer token is created in `~/.large_bert_token` on the Pi.
  Clients send it as `Authorization: Bearer <token>`. Example `.mcp.json` entry:
  `{"type": "http", "url": "http://192.168.0.119:8765/mcp", "headers": {"Authorization": "Bearer <token>"}}`

## If something goes wrong
- **`/mcp` shows "failed":** run the `ssh ...` command from `.mcp.json` yourself in PowerShell
  and read the error. Usually SSH is asking for a password (redo step 4), or `sudo` wants
  one (run `sudo visudo -f /etc/sudoers.d/010_pi-nopasswd` and add `pi ALL=(ALL) NOPASSWD: ALL`).
- **"GPIO busy" or camera errors:** `main.py` or `cat_patrol.py` is still running. Stop it.
- **He turns or looks the wrong way:** set `TURN_SIGN` or `PAN_SIGN` in `patrol_config.py`.
  Run `sudo python3 cat_patrol.py --test-hardware` to check.
