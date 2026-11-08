# Cat patrol

Turns the hexapod into a yard guard that shoos away one particular orange cat
and leaves your own cats alone. It only uses lights, chirps and walking
toward the cat. It never touches an animal.

## Files (all in `Code/Server`)

| File | What it does |
|---|---|
| `cat_patrol.py` | The patrol itself: watch, investigate, deter, cool down |
| `cat_vision.py` | Camera, cat/person detector, "is it the orange intruder?" check |
| `patrol_config.py` | Every setting worth tuning, with comments |
| `get_cat_model.sh` | Downloads the detector model (once) |
| `cats/intruder/` | Cropped photos of orange boy |
| `cats/mine/Bingus`, `Jimb`, `Kermit`, `Walter` | Cropped photos of your cats |
| `cat_model/cat_embedder.onnx` | Small network that compares a cat with those photos |

## Setup on the robot's Pi

```bash
cd ~/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi/Code/Server
sudo apt install -y python3-opencv     # if OpenCV isn't installed yet
./get_cat_model.sh
```

Stop the normal robot server (`main.py`) first, since both use the same servos and camera.

## First run, step by step

1. **Check directions:** `sudo python3 cat_patrol.py --test-hardware`.
   It walks forward, turns left, and looks left. If the turn went right, set
   `TURN_SIGN = -1`. If the camera looked right, set `PAN_SIGN = -1`.
2. **Watch without moving:** `python3 cat_patrol.py --dry-run`.
   It logs what it would do and saves pictures to `cat_log/`. Each picture
   shows boxes: red is the intruder, green is yours, and yellow means unsure.
   Cat crops are sorted into `cat_log/crops/intruder|mine|unsure`.
3. **Improve recognition with the robot's own pictures:** your phone photos
   are already set up as references. The robot's camera sees cats from lower
   down and further away, so after a few days of dry runs, copy good crops from
   `cat_log/crops/` into `cats/intruder/` or `cats/mine/<name>/`. Remove any
   that are blurry or show the wrong cat.
4. **Go live:** `sudo python3 cat_patrol.py` (sentry mode) or
   `sudo python3 cat_patrol.py --mode roam`.

You can also test the vision on a PC with a phone video of the cat:
`python cat_patrol.py --video orange_cat.mp4 --show` (needs `pip install opencv-python numpy`).

## How it behaves

1. **Watch:** sweeps the camera across the yard (sentry), or walks a few steps
   and then sweeps, turning away from anything closer than 45 cm (roam).
2. **Investigate:** when any cat appears, it turns to face it, stands still with
   amber lights, and looks several times. It needs 4 of the last 6 looks to say
   "intruder" (and none to say "yours") before doing anything.
3. **Deter:** red and blue flashing lights, a chirp every 2 seconds, and it walks
   toward the cat while steering to keep it centred.
4. **Humane limits:**
   - It never goes closer than 60 cm.
   - If the cat stays put inside that distance for 5 seconds, the robot backs
     off instead of cornering it.
   - It gives up after 20 seconds, then rests for 2 minutes.
   - It does at most 6 chases an hour.
   - It stops at once if a person or one of your cats comes into view.
   - The buzzer is silent between 22:00 and 07:00.
5. **Battery:** below 6.2 V it chirps, sits down and stops.

## How it tells the cats apart

Each cat the detector finds is turned into a "fingerprint" by
`cat_embedder.onnx` and compared with the reference photos. It only counts as
the intruder when it matches orange boy better than any of your four cats,
**and** at least 25% of it looks orange. I tested this on your 37 photos,
checking each one against all the others:

| Cat | Intruder | Mine | Unsure |
|---|---|---|---|
| orange boy | 5 | 0 | 2 |
| Bingus, Jimb, Kermit, Walter | **0** | 26 | 4 |

"Unsure" means it does nothing. On top of that, a chase needs 4 of the last 6
looks to agree. Colour alone wasn't safe for your cats: Jimb's tabby coat in
sunshine looked as orange as orange boy. That's why the colour check is only
a second gate.

## Tuning tips

- **Orange boy not recognised often enough?** Add more crops of him from
  `cat_log/crops/unsure`, or lower `INTRUDER_MARGIN` a little.
- **One of your cats ever flagged as intruder?** Add that cat's crops to its
  `cats/mine/<name>` folder and raise `INTRUDER_MARGIN`.
- **Your gravel yard:** the robot should cope. If it stumbles, lower
  `STEP_LENGTH` to 20 and `APPROACH_SPEED` to 6. Since the yard is fenced,
  `--mode roam` is fine. Sentry mode still saves battery.
- **Too jumpy or too slow?** Adjust `VOTES_NEEDED`, `APPROACH_SPEED` and `STEP_LENGTH`.
- **Turns overshoot or undershoot?** Adjust `DEG_PER_TURN_CYCLE`.
