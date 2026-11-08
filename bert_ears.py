# -*- coding: utf-8 -*-
"""Large Bert's ears: the ReSpeaker XVF3800 USB mic array + Vosk speech-to-text, all on the Pi.

One background thread reads the mic all the time and does two jobs:
  * safety: it listens for "Bert, stop" (or just "stop") and calls on_stop() at once,
    without waiting on Claude. This runs even while Bert is walking.
  * listen(): when asked, it transcribes the next few seconds of speech for Claude.

The stop-word check uses a tiny Vosk grammar, which is cheap enough for a Pi 3, and runs on
every bit of sound. "Bert, stop" fires as soon as it shows up in the partial result; a lone
"stop" fires when the phrase ends. While listen() runs, the full recogniser also runs and its
text is checked for "stop" as well.

Settings live at the top of this file. Try it on its own:
    python3 bert_ears.py             # prints what it hears and any stop words
    python3 bert_ears.py --listen 6  # one 6 second transcription
"""
import json
import os
import subprocess
import threading
import time

MODEL_DIR = os.path.expanduser(os.environ.get("BERT_VOSK_MODEL", "/home/pi/vosk-models/vosk-model-small-en-us-0.15"))
ALSA_DEVICE = "plughw:Array"          # the XVF3800 shows up as ALSA card "Array"
RATE = 16000
CHANNELS = 2                          # XVF3800 USB gives 2 channels; channel 0 is the processed (beamformed) one
CHUNK_SECONDS = 0.25
# What counts as a stop. Leaning towards stopping when unsure is the safe side.
# A small grammar forces every sound onto one of its phrases, so it also lists look-alike
# "decoy" words (walk, step, top, ...) to soak up speech that isn't a stop.
STOP_WORDS = ("stop",)
STOP_GRAMMAR = ["bert stop", "stop bert", "stop", "stop stop",
                "bert", "walk", "forward", "back", "step", "steps", "top", "stuff", "shop", "start",
                "turn", "three", "okay", "what", "up", "[unk]"]
STOP_PHRASES_FAST = ("bert stop", "stop bert", "stop stop")   # fire on the partial result, no waiting
STOP_MIN_CONF = 0.6                   # word confidence needed in the idle grammar check
STOP_COOLDOWN = 2.0                   # seconds before the same stop can fire again


def _log(message):
    import sys
    sys.stderr.write("[ears] %s\n" % message)
    sys.stderr.flush()


class Ears:
    def __init__(self, on_stop=None, log=_log):
        from vosk import Model, SetLogLevel
        SetLogLevel(-1)
        self.log = log
        self.on_stop = on_stop
        self.model = Model(MODEL_DIR)
        self.lock = threading.Lock()
        self._listener = None            # (recogniser, list of finished text, done event, deadline)
        self.last_stop = None            # (time, what was heard)
        self.stop_count = 0
        self.mic_ok = False
        self.error = None
        self._running = True
        self._proc = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    # ---- public ----------------------------------------------------------
    def listen(self, seconds=5.0, end_on_silence=True):
        """Transcribe up to `seconds` of speech. With end_on_silence it returns soon after a
        sentence ends (once something was heard). Returns {"text", "heard_stop", ...}."""
        from vosk import KaldiRecognizer
        seconds = max(1.0, min(float(seconds), 30.0))
        if not self.mic_ok:
            raise RuntimeError("the microphone isn't working: %s" % (self.error or "not started yet"))
        rec = KaldiRecognizer(self.model, RATE)
        job = {"rec": rec, "texts": [], "done": threading.Event(), "deadline": time.time() + seconds,
               "end_on_silence": end_on_silence, "stop": False}
        with self.lock:
            if self._listener is not None:
                raise RuntimeError("already listening")
            self._listener = job
        try:
            job["done"].wait(seconds + 5)
        finally:
            with self.lock:
                self._listener = None
        last = json.loads(rec.FinalResult()).get("text", "")
        if last:
            job["texts"].append(last)
        return {"text": " ".join(t for t in job["texts"] if t).strip(), "heard_stop": job["stop"],
                "seconds": seconds}

    def status(self):
        return {"mic_ok": self.mic_ok, "error": self.error, "voice_stops": self.stop_count,
                "last_voice_stop": None if not self.last_stop else
                {"seconds_ago": round(time.time() - self.last_stop[0]), "heard": self.last_stop[1]}}

    def close(self):
        self._running = False
        if self._proc:
            try:
                self._proc.kill()
            except Exception:
                pass

    # ---- internals ------------------------------------------------------
    def _fire_stop(self, heard):
        now = time.time()
        if self.last_stop and now - self.last_stop[0] < STOP_COOLDOWN:
            return
        self.last_stop = (now, heard)
        self.stop_count += 1
        self.log("heard a stop: %r" % heard)
        if self.on_stop:
            try:
                self.on_stop(heard)
            except Exception as exc:
                self.log("on_stop failed: %s" % exc)

    @staticmethod
    def _has_stop(text):
        return any(w in STOP_WORDS for w in text.split())

    def _check_grammar_result(self, result):
        words = result.get("result") or []
        hits = [w for w in words if w.get("word") in STOP_WORDS and w.get("conf", 0) >= STOP_MIN_CONF]
        if hits:
            self._fire_stop(result.get("text", ""))
            return True
        return False

    def _check_grammar_partial(self, partial):
        if any(p in partial for p in STOP_PHRASES_FAST):
            self._fire_stop(partial)
            return True
        return False

    def _run(self):
        from vosk import KaldiRecognizer
        stop_rec = None
        bytes_per_chunk = int(RATE * CHUNK_SECONDS) * 2 * CHANNELS
        while self._running:
            try:
                self._proc = subprocess.Popen(
                    ["arecord", "-q", "-D", ALSA_DEVICE, "-f", "S16_LE", "-r", str(RATE),
                     "-c", str(CHANNELS), "-t", "raw"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                stop_rec = KaldiRecognizer(self.model, RATE, json.dumps(STOP_GRAMMAR))
                stop_rec.SetWords(True)
                while self._running:
                    raw = self._proc.stdout.read(bytes_per_chunk)
                    if not raw:
                        err = self._proc.stderr.read().decode(errors="replace").strip()
                        raise RuntimeError(err or "microphone stream ended")
                    if not self.mic_ok:
                        self.mic_ok, self.error = True, None
                        self.log("microphone open")
                    mono = _channel0(raw, CHANNELS)
                    with self.lock:
                        job = self._listener
                    if stop_rec.AcceptWaveform(mono):
                        fired = self._check_grammar_result(json.loads(stop_rec.Result()))
                        stop_rec.Reset()
                    else:
                        fired = self._check_grammar_partial(json.loads(stop_rec.PartialResult()).get("partial", ""))
                        if fired:
                            stop_rec.Reset()         # don't fire again on the same words
                    if job is None:
                        continue
                    if fired:
                        job["stop"] = True
                    # listen() is running: feed the full recogniser and watch it for stop words
                    rec = job["rec"]
                    if rec.AcceptWaveform(mono):
                        text = json.loads(rec.Result()).get("text", "")
                        if text:
                            job["texts"].append(text)
                            if self._has_stop(text):
                                job["stop"] = True
                                self._fire_stop(text)
                            if job["end_on_silence"]:
                                job["done"].set()
                    else:
                        partial = json.loads(rec.PartialResult()).get("partial", "")
                        if partial and self._has_stop(partial) and not job["stop"]:
                            job["stop"] = True
                            self._fire_stop(partial)
                    if time.time() >= job["deadline"]:
                        job["done"].set()
            except Exception as exc:
                self.mic_ok, self.error = False, str(exc)
                self.log("microphone problem (%s); retrying in 5 s" % exc)
                with self.lock:
                    if self._listener is not None:
                        self._listener["done"].set()
                try:
                    if self._proc:
                        self._proc.kill()
                except Exception:
                    pass
                time.sleep(5)


def _channel0(raw, channels):
    if channels == 1:
        return raw
    from array import array
    samples = array("h")
    samples.frombytes(raw[:len(raw) - len(raw) % (2 * channels)])
    return samples[::channels].tobytes()


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--listen", type=float, default=0, help="transcribe this many seconds and quit")
    a = p.parse_args()
    ears = Ears(on_stop=lambda heard: print("STOP! (heard %r)" % heard, flush=True))
    time.sleep(1.5)
    if a.listen:
        print(ears.listen(a.listen, end_on_silence=False))
    else:
        print("Listening for stop words; Ctrl+C to quit.")
        try:
            while True:
                print(ears.listen(8))
        except KeyboardInterrupt:
            pass
    ears.close()
