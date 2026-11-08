# Large Bert's ears

Large Bert has a **ReSpeaker XVF3800 USB 4-mic array** plugged into the Pi. Speech-to-text
runs on the Pi with [Vosk](https://alphacephei.com/vosk/), fully offline.

It does two things:

- **"Bert, stop" freezes him on the Pi**, without waiting on Claude. It works while he walks,
  during the cat patrol, and with the wheel drive (`bert_drive.py` uses the same `Bert` class).
  A walk that is cut short ends with "stopped by voice". A lone "stop" works too.
- **`listen`**, an MCP tool: Claude hears what people near Bert say, for up to 30 seconds. It
  returns shortly after a sentence ends.

The code is `bert_ears.py`. The MCP server loads it by itself. If the mic, Vosk or the model
is missing, voice is off and everything else still works (`status` shows `ears`).

## Install

1. Plug the XVF3800 into one of the Pi's USB-A ports with a **data** cable (a power-only
   cable looks like nothing is plugged in). Check: `arecord -l` lists `reSpeaker XVF3800`.
2. On the Pi:
   ```
   ~/bert-venv/bin/pip install vosk
   mkdir -p ~/vosk-models && cd ~/vosk-models
   wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
   unzip vosk-model-small-en-us-0.15.zip && rm vosk-model-small-en-us-0.15.zip
   ```
3. Copy `bert_ears.py` next to `large_bert_mcp.py` and restart the MCP server.

Try the ears on their own (stop the MCP server first, since only one program can use the mic):
```
~/bert-venv/bin/python bert_ears.py             # prints what it hears and any stops
~/bert-venv/bin/python bert_ears.py --listen 6  # one 6 second transcription
```

## How it works and its limits

- One thread reads the mic all the time (channel 0, the XVF3800's processed beam, 16 kHz).
- A tiny Vosk grammar that only knows "Bert stop", "stop" and some look-alike words checks
  every bit of sound. "Bert, stop" fires as soon as it is heard; a lone "stop" fires at the end
  of the phrase. This costs a small part of one CPU core on the Pi 3.
- `listen` also runs the full small English model, at about half real time on the Pi 3. It
  is rough on names: "Bert" often comes out as "birds" or "birch".
- It leans towards stopping when unsure. A false stop just makes him stand still.
- The model uses about 200 MB of the Pi 3's 1 GB of RAM.
- `--no-ears` turns it all off. `--sim` leaves the mic alone.

Settings (stop words, confidence, mic device) are at the top of `bert_ears.py`.
