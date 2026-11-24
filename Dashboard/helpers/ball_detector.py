"""The trained paper-ball model (YOLO11n, ONNX) for the dashboard, run with the OpenCV the dashboard already has.

The picture is prepared with the same letterbox as the phone app (resized to fit 640x640, grey 114 around it), the model's
output is decoded into boxes in the original picture's pixels, and overlapping boxes are merged. Frames are RGB, like the
dashboard's camera frames.
"""
import threading
import time
from collections import namedtuple
from pathlib import Path

import cv2
import numpy as np

from helpers.paths import MODEL_PATH       # Training/paper_ball_detector/roboflow-paperball.onnx

MODEL_SIZE = 640
IOU = 0.45                  # boxes overlapping more than this are one ball
GREEN = (0, 255, 0)         # RGB

Detection = namedtuple("Detection", "score x1 y1 x2 y2")      # box corners in picture pixels


def letterbox(frame, size=MODEL_SIZE):
    """Fit the picture into a size x size square (grey around it). Returns (square, scale, pad_x, pad_y)."""
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    pad_x, pad_y = (size - nw) / 2.0, (size - nh) / 2.0
    canvas = np.full((size, size, 3), 114, np.uint8)
    x0, y0 = int(round(pad_x)), int(round(pad_y))
    canvas[y0:y0 + nh, x0:x0 + nw] = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return canvas, scale, pad_x, pad_y


def decode(output, conf, iou, scale, pad_x, pad_y, width, height):
    """The model's output tensor -> [Detection] in the original picture, best first. `output` is (1, 4 + classes, boxes) or the
    transposed layout; each box is (centre x, centre y, width, height, class scores...) in letterbox pixels."""
    out = output.reshape(output.shape[-2], output.shape[-1])
    if out.shape[0] > out.shape[1]:
        out = out.T                                          # -> [4 + classes, boxes]
    score = out[4:].max(axis=0)
    keep = score > conf
    if not keep.any():
        return []
    cx, cy, bw, bh, score = out[0][keep], out[1][keep], out[2][keep], out[3][keep], score[keep]
    left = np.clip((cx - bw / 2 - pad_x) / scale, 0, width)
    top = np.clip((cy - bh / 2 - pad_y) / scale, 0, height)
    right = np.clip((cx + bw / 2 - pad_x) / scale, 0, width)
    bottom = np.clip((cy + bh / 2 - pad_y) / scale, 0, height)
    boxes = [[float(left[i]), float(top[i]), float(right[i] - left[i]), float(bottom[i] - top[i])] for i in range(len(score))]
    found = []
    for k in np.array(cv2.dnn.NMSBoxes(boxes, score.tolist(), conf, iou)).flatten():
        k = int(k)
        if right[k] > left[k] and bottom[k] > top[k]:
            found.append(Detection(float(score[k]), int(left[k]), int(top[k]), int(right[k]), int(bottom[k])))
    return sorted(found, key=lambda d: -d.score)


class BallDetector:
    """The model itself. detect() takes one picture at a time (a lock makes the live worker and a photo test take turns)."""

    def __init__(self, path=MODEL_PATH, size=MODEL_SIZE, iou=IOU):
        if not Path(path).exists():
            raise FileNotFoundError(f"model not found: {path}")
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.size, self.iou = size, iou
        self._lock = threading.Lock()

    def detect(self, rgb, conf=0.5):
        h, w = rgb.shape[:2]
        square, scale, pad_x, pad_y = letterbox(rgb, self.size)
        blob = cv2.dnn.blobFromImage(square, 1 / 255.0, (self.size, self.size), swapRB=False, crop=False)
        with self._lock:
            self.net.setInput(blob)
            output = self.net.forward()
        return decode(output, conf, self.iou, scale, pad_x, pad_y, w, h)


def draw_detections(rgb, detections):
    """A copy of the picture with a green box and the score around every detection (the best one drawn thicker)."""
    if not detections:
        return rgb
    out = rgb.copy()
    thickness = max(2, out.shape[1] // 200)
    font = max(0.5, out.shape[1] / 800)
    for i, d in enumerate(detections):
        cv2.rectangle(out, (d.x1, d.y1), (d.x2, d.y2), GREEN, thickness + 1 if i == 0 else thickness)
        cv2.putText(out, f"{d.score * 100:.0f}%", (d.x1, max(14, d.y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, font, GREEN,
                    max(1, thickness - 1))
    return out


def summarize(detections, ms):
    if not detections:
        return f"no paper balls, {ms:.0f} ms"
    n = len(detections)
    return f"{n} paper ball{'s' if n != 1 else ''}, best {detections[0].score * 100:.0f}%, {ms:.0f} ms"


DetectionResult = namedtuple("DetectionResult", "frame_id detections ms stamp width height")


class DetectorWorker(threading.Thread):
    """Runs the model on the newest camera frame in the background (never on the screen's thread), skipping frames while it is
    busy. The model is loaded the first time it is needed. Set `enabled` and `conf`; read the newest result with latest()."""

    def __init__(self, camera, make_detector=BallDetector):
        super().__init__(daemon=True)
        self.camera, self.make_detector = camera, make_detector
        self.enabled, self.conf = False, 0.5
        self.error = ""                                      # why detection switched itself off, if it did
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        self._load_lock = threading.Lock()
        self._result = None                                  # (frame id, [Detection], milliseconds)
        self._full = None                                    # the same as a DetectionResult
        self._detector = None
        self._last_id = -1

    def _get_detector(self):
        with self._load_lock:
            if self._detector is None:
                self._detector = self.make_detector()
            return self._detector

    def detect_now(self, rgb):
        """One picture right now (for the photo test): ([Detection], milliseconds). Raises if the model cannot be loaded."""
        detector = self._get_detector()
        start = time.perf_counter()
        found = detector.detect(rgb, self.conf)
        return found, (time.perf_counter() - start) * 1000.0

    def step(self):
        """Handle at most one new frame. True when one was processed."""
        if not self.enabled:
            time.sleep(0.05)
            return False
        frame_id, rgb = self.camera.snapshot()
        if rgb is None or frame_id == self._last_id:
            time.sleep(0.02)
            return False
        try:
            found, ms = self.detect_now(rgb)
        except Exception as e:                               # model missing or unreadable: say so, and stop trying
            self.error, self.enabled = f"Cannot run the model: {e}", False
            return False
        self._last_id = frame_id
        height, width = rgb.shape[:2]
        with self._lock:
            self._result = (frame_id, found, ms)
            self._full = DetectionResult(frame_id, found, ms, time.monotonic(), width, height)
        return True

    def run(self):
        while not self.stop_event.is_set():
            self.step()

    def latest(self):
        with self._lock:
            return self._result

    def latest_frame(self):
        """The newest result with the time the model finished and the size of the frame it ran on; None before the first frame."""
        with self._lock:
            return self._full
