# -*- coding: utf-8 -*-
"""Camera input, cat/person detection and "is this the orange intruder?" checks."""
import glob
import os
import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import patrol_config as C

# Class ids of the TensorFlow COCO SSD model
COCO_PERSON = 1
COCO_CAT = 17
COCO_DOG = 18
LABELS = {COCO_PERSON: "person", COCO_CAT: "cat", COCO_DOG: "dog"}

INTRUDER = "intruder"
MINE = "mine"
UNSURE = "unsure"


@dataclass
class Detection:
    label: str
    score: float
    box: tuple          # x, y, w, h in pixels
    cx: float           # horizontal centre, -1 (left edge) .. 1 (right edge)
    area: float         # fraction of the frame the box covers
    verdict: str = UNSURE
    orange: float = 0.0


class PiCameraSource:
    """Grabs frames from the robot's Pi camera on a background thread."""
    def __init__(self, size=C.CAMERA_SIZE):
        from picamera2 import Picamera2
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(main={"size": size, "format": "RGB888"})
        self.camera.configure(config)
        self.camera.start()
        self._frame, self._stamp = None, 0.0
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _grab(self):
        return self.camera.capture_array()  # RGB888 in picamera2 is BGR byte order, as OpenCV expects

    def _loop(self):
        while self._running:
            frame = self._grab()
            if frame is None:
                time.sleep(0.05)
                continue
            with self._lock:
                self._frame, self._stamp = frame, time.time()

    def read(self, newer_than=0.0, timeout=2.0):
        """Return the latest frame taken after `newer_than`, or None on timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._frame is not None and self._stamp > newer_than:
                    return self._frame.copy()
            time.sleep(0.02)
        return None

    def close(self):
        self._running = False
        self.camera.stop()
        self.camera.close()


class VideoSource(PiCameraSource):
    """A webcam index or video file, for testing detection on a PC."""
    def __init__(self, path):
        self.cap = cv2.VideoCapture(int(path) if str(path).isdigit() else path)
        self.is_file = not str(path).isdigit()
        self._frame, self._stamp = None, 0.0
        self._lock = threading.Lock()
        self._running = True
        self.ended = False
        threading.Thread(target=self._loop, daemon=True).start()

    def _grab(self):
        ok, frame = self.cap.read()
        if not ok:
            self.ended = True
            self._running = False
            return None
        if self.is_file:
            time.sleep(1 / 15)  # play files at roughly camera speed
        return frame

    def close(self):
        self._running = False
        self.cap.release()


class CatDetector:
    """Finds cats (and people, so we can hold off around them) with MobileNet-SSD."""
    def __init__(self, model_dir=C.MODEL_DIR):
        weights = os.path.join(model_dir, "frozen_inference_graph.pb")
        graph = os.path.join(model_dir, "ssd_mobilenet_v2_coco_2018_03_29.pbtxt")
        if not (os.path.exists(weights) and os.path.exists(graph)):
            raise FileNotFoundError("Detector model missing. Run ./get_cat_model.sh first.")
        self.net = cv2.dnn.readNetFromTensorflow(weights, graph)

    def detect(self, frame):
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True)
        self.net.setInput(blob)
        out = self.net.forward()
        found = []
        for _, class_id, score, x1, y1, x2, y2 in out[0, 0]:
            class_id = int(class_id)
            if class_id not in LABELS:
                continue
            needed = C.PERSON_MIN_SCORE if class_id == COCO_PERSON else C.CAT_MIN_SCORE
            if score < needed:
                continue
            x1, x2 = max(0, int(x1 * w)), min(w, int(x2 * w))
            y1, y2 = max(0, int(y1 * h)), min(h, int(y2 * h))
            if x2 <= x1 or y2 <= y1:
                continue
            cx = ((x1 + x2) / 2 - w / 2) / (w / 2)
            area = (x2 - x1) * (y2 - y1) / float(w * h)
            found.append(Detection(LABELS[class_id], float(score), (x1, y1, x2 - x1, y2 - y1), cx, area))
        return found


def inner_crop(frame, box, shrink=0.15):
    """Crop the middle of a box so grass/fence behind the cat counts less."""
    x, y, w, h = box
    dx, dy = int(w * shrink), int(h * shrink)
    crop = frame[y + dy:y + h - dy, x + dx:x + w - dx]
    return crop if crop.size else frame[y:y + h, x:x + w]


class CatIdentifier:
    """Tells the orange intruder apart from your own cats.

    Main signal: a small image network (cat_embedder.onnx) turns each cat into a
    "fingerprint", which is compared with your reference photos in
    cats/intruder/ and cats/mine/<name>/. The cat must also look orange.
    Without the model or photos it falls back to a strict orange-only check.
    When in doubt it answers UNSURE, and the robot does nothing.
    """
    def __init__(self, reference_dir=C.REFERENCE_DIR, embedder_file=C.EMBEDDER_FILE):
        self.net = cv2.dnn.readNetFromONNX(embedder_file) if os.path.exists(embedder_file) else None
        self.refs = {INTRUDER: [], MINE: []}
        self.ref_names = []
        self.ref_vectors = []
        for kind in self.refs:
            for path in sorted(glob.glob(os.path.join(reference_dir, kind, "**", "*.*"), recursive=True)):
                image = cv2.imread(path)
                if image is None:
                    continue
                self.refs[kind].append(path)
                if self.net is not None:
                    # Each of your cats is its own group, so a near match to any one of them counts
                    folder = os.path.relpath(os.path.dirname(path), os.path.join(reference_dir, kind))
                    name = kind if kind == INTRUDER else "%s/%s" % (kind, folder)
                    for img in (image, cv2.flip(image, 1)):
                        self.ref_names.append(name)
                        self.ref_vectors.append(self.embed(img))
        self.use_refs = self.net is not None and bool(self.refs[INTRUDER]) and bool(self.refs[MINE])
        if self.use_refs:
            self.ref_vectors = np.array(self.ref_vectors)
            self.ref_names = np.array(self.ref_names)

    def embed(self, image):
        self.net.setInput(cv2.dnn.blobFromImage(image, 1 / 255.0, (224, 224), swapRB=True))
        return self.net.forward()[0]

    @staticmethod
    def orange_fraction(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low = np.array([C.ORANGE_HUE_RANGE[0], C.ORANGE_MIN_SAT, C.ORANGE_MIN_VAL])
        high = np.array([C.ORANGE_HUE_RANGE[1], 255, 255])
        return float(np.count_nonzero(cv2.inRange(hsv, low, high))) / max(1, hsv.shape[0] * hsv.shape[1])

    def match_scores(self, image):
        """Best-match score for each reference cat, e.g. {"intruder": .6, "mine/Jimb": .7}."""
        sims = self.ref_vectors @ self.embed(image)
        scores = {}
        for name in set(self.ref_names):
            best = np.sort(sims[self.ref_names == name])[::-1][:C.MATCH_TOP_K]
            scores[name] = float(best.mean())
        return scores

    def classify(self, crop, full_crop=None):
        """Return (verdict, orange_fraction). full_crop is the whole box, used for matching."""
        orange = self.orange_fraction(crop)
        if self.use_refs:
            scores = self.match_scores(full_crop if full_crop is not None else crop)
            intruder = scores.pop(INTRUDER)
            mine = max(scores.values())
            if intruder - mine >= C.INTRUDER_MARGIN and orange >= C.ORANGE_GATE:
                return INTRUDER, orange
            if mine - intruder >= C.MINE_MARGIN:
                return MINE, orange
            return UNSURE, orange
        if orange >= C.ORANGE_ONLY_MIN:
            return INTRUDER, orange
        if orange <= C.MINE_MAX_ORANGE:
            return MINE, orange
        return UNSURE, orange


class Vision:
    """Detector + identifier + a short memory so one odd frame can't trigger a chase."""
    def __init__(self, source, detector=None, identifier=None):
        self.source = source
        self.detector = detector or CatDetector()
        self.identifier = identifier or CatIdentifier()
        self.votes = deque(maxlen=C.VOTE_WINDOW)
        self.last_frame = None
        self.last_stamp = 0.0
        self.last_save = 0.0

    def look(self):
        """Take a fresh frame. Returns (intruder, my_cat_seen, person_seen, detections)."""
        frame = self.source.read(newer_than=self.last_stamp)
        if frame is None:
            return None, False, False, []
        self.last_stamp = time.time()
        self.last_frame = frame
        detections = self.detector.detect(frame)
        intruder, my_cat, person = None, False, False
        for d in detections:
            if d.label == "person":
                person = True
            elif d.label == "cat":
                x, y, w, h = d.box
                d.verdict, d.orange = self.identifier.classify(inner_crop(frame, d.box), frame[y:y + h, x:x + w])
                if d.verdict == MINE:
                    my_cat = True
                elif d.verdict == INTRUDER and (intruder is None or d.area > intruder.area):
                    intruder = d
        cats = [d for d in detections if d.label == "cat"]
        if cats:
            self.votes.append(INTRUDER if intruder else (MINE if my_cat else UNSURE))
        if cats and C.SAVE_CROPS:
            self.save(frame, detections)
        return intruder, my_cat, person, detections

    def confirmed_intruder(self):
        return list(self.votes).count(INTRUDER) >= C.VOTES_NEEDED and MINE not in self.votes

    def reset_votes(self):
        self.votes.clear()

    def save(self, frame, detections):
        now = time.time()
        if now - self.last_save < C.MIN_SECONDS_BETWEEN_SAVES:
            return
        self.last_save = now
        stamp = time.strftime("%Y%m%d-%H%M%S")
        for i, d in enumerate(detections):
            if d.label != "cat":
                continue
            folder = os.path.join(C.LOG_DIR, "crops", d.verdict)
            os.makedirs(folder, exist_ok=True)
            x, y, w, h = d.box
            cv2.imwrite(os.path.join(folder, "%s_%d_orange%02d.jpg" % (stamp, i, int(d.orange * 100))),
                        frame[y:y + h, x:x + w])
        os.makedirs(C.LOG_DIR, exist_ok=True)
        cv2.imwrite(os.path.join(C.LOG_DIR, stamp + ".jpg"), annotate(frame, detections))


def annotate(frame, detections):
    image = frame.copy()
    colours = {INTRUDER: (0, 0, 255), MINE: (0, 200, 0), UNSURE: (0, 200, 255)}
    for d in detections:
        x, y, w, h = d.box
        colour = (255, 128, 0) if d.label == "person" else colours[d.verdict]
        text = d.label if d.label != "cat" else "%s %.0f%% orange" % (d.verdict, d.orange * 100)
        cv2.rectangle(image, (x, y), (x + w, y + h), colour, 2)
        cv2.putText(image, "%s %.2f" % (text, d.score), (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
    return image
