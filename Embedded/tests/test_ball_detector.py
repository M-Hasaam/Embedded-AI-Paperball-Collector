import sys
import threading
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ball_detector import (MODEL_PATH, BallDetector, Detection, DetectorWorker, decode, draw_detections,  # noqa: E402
                           letterbox, summarize)

DATASET = Path(__file__).resolve().parents[2] / "Training" / "paper_ball_detector" / "dataset" / "test" / "test2"


def tensor(rows, layout="anchors_last"):
    """A fake model output: rows = [(cx, cy, w, h, score), ...] in 640x640 letterbox pixels -> shape (1, 5, N) or (1, N, 5)."""
    rows = list(rows) + [(0, 0, 0, 0, 0.0)] * (16 - len(rows))      # the real output has 8400 boxes: most of them empty
    out = np.array(rows, dtype=np.float32).T[None]          # (1, 5, N)
    return out if layout == "anchors_last" else out.transpose(0, 2, 1)


class LetterboxTest(unittest.TestCase):
    def test_pads_to_a_square_and_keeps_the_scale(self):
        frame = np.full((240, 320, 3), 200, np.uint8)          # 320 wide, 240 high
        canvas, scale, pad_x, pad_y = letterbox(frame, 640)
        self.assertEqual(canvas.shape, (640, 640, 3))
        self.assertEqual((scale, pad_x, pad_y), (2.0, 0.0, 80.0))
        self.assertEqual(int(canvas[10, 300, 0]), 114)         # the grey bars above and below
        self.assertEqual(int(canvas[320, 300, 0]), 200)        # the picture in the middle


class DecodeTest(unittest.TestCase):
    # a 320x240 picture letterboxed to 640: scale 2, no side padding, 80 px above and below
    ARGS = dict(scale=2.0, pad_x=0.0, pad_y=80.0, width=320, height=240)

    def test_maps_the_box_back_to_the_picture(self):
        dets = decode(tensor([(320, 320, 100, 100, 0.9), (50, 50, 20, 20, 0.1)]), conf=0.5, iou=0.45, **self.ARGS)
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0].score, 0.9, places=5)
        self.assertEqual((dets[0].x1, dets[0].y1, dets[0].x2, dets[0].y2), (135, 95, 185, 145))

    def test_merges_overlapping_boxes_and_keeps_the_best_first(self):
        rows = [(320, 320, 100, 100, 0.8), (324, 322, 100, 100, 0.9), (500, 400, 60, 60, 0.6)]
        dets = decode(tensor(rows), conf=0.5, iou=0.45, **self.ARGS)
        self.assertEqual(len(dets), 2)
        self.assertGreater(dets[0].score, dets[1].score)

    def test_accepts_the_other_layout_of_the_output(self):
        a = decode(tensor([(320, 320, 100, 100, 0.9)]), conf=0.5, iou=0.45, **self.ARGS)
        b = decode(tensor([(320, 320, 100, 100, 0.9)], "anchors_first"), conf=0.5, iou=0.45, **self.ARGS)
        self.assertEqual(a, b)

    def test_nothing_confident_gives_an_empty_list(self):
        self.assertEqual(decode(tensor([(320, 320, 100, 100, 0.2)]), conf=0.5, iou=0.45, **self.ARGS), [])

    def test_boxes_are_clipped_to_the_picture(self):
        dets = decode(tensor([(630, 320, 100, 100, 0.9)]), conf=0.5, iou=0.45, **self.ARGS)
        self.assertLessEqual(dets[0].x2, 320)


class DrawTest(unittest.TestCase):
    def test_marks_the_box_and_leaves_the_input_alone(self):
        frame = np.zeros((100, 100, 3), np.uint8)
        shown = draw_detections(frame, [Detection(0.87, 10, 20, 60, 70)])
        self.assertFalse(frame.any())                           # the original picture is untouched
        self.assertTrue(shown.any())
        self.assertEqual(tuple(int(v) for v in shown[45, 10]), (0, 255, 0))      # on the left edge of the box: green (RGB)

    def test_no_detections_returns_the_same_picture(self):
        frame = np.full((50, 50, 3), 7, np.uint8)
        self.assertTrue((draw_detections(frame, []) == frame).all())


class SummaryTest(unittest.TestCase):
    def test_text(self):
        self.assertEqual(summarize([], 31.4), "no paper balls, 31 ms")
        self.assertEqual(summarize([Detection(0.87, 1, 2, 3, 4)], 31.4), "1 paper ball, best 87%, 31 ms")
        self.assertEqual(summarize([Detection(0.87, 1, 2, 3, 4), Detection(0.5, 1, 2, 3, 4)], 12), "2 paper balls, best 87%, 12 ms")


@unittest.skipUnless(MODEL_PATH.exists() and DATASET.exists(), "the trained model or the test2 pictures are missing")
class RealModelTest(unittest.TestCase):
    """The trained model on real ESP32-CAM pictures that were labelled by hand (YOLO labels: class cx cy w h, 0..1)."""

    @classmethod
    def setUpClass(cls):
        cls.detector = BallDetector()

    @staticmethod
    def iou(a, b):
        ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
        iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
        inter = ix * iy
        union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / union if union else 0.0

    def labelled_pictures(self, count):
        found = []
        for image in sorted((DATASET / "images").glob("*.jpg")):
            label = DATASET / "labels" / (image.stem + ".txt")
            if label.exists() and label.read_text().strip():
                found.append((image, label))
            if len(found) == count:
                break
        return found

    def results(self, conf):
        """For the first 8 labelled pictures: the best overlap (IoU) of any detection with the hand-drawn box (0 = nothing found)."""
        best = []
        for image, label in self.labelled_pictures(8):
            bgr = cv2.imdecode(np.fromfile(str(image), np.uint8), cv2.IMREAD_COLOR)
            h, w = bgr.shape[:2]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            _, cx, cy, bw, bh = (float(v) for v in label.read_text().split()[:5])
            truth = ((cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h)
            best.append(max((self.iou(truth, (d.x1, d.y1, d.x2, d.y2)) for d in self.detector.detect(rgb, conf)), default=0.0))
        return best

    def test_the_boxes_land_on_the_hand_drawn_ball(self):
        # Measured on these 8 pictures: at 0.25 it finds the ball in 7, with overlaps from 0.53 to 0.92. This checks that the
        # picture handling and the mapping of the boxes back to the picture are right.
        best = self.results(0.25)
        found = [b for b in best if b > 0]
        self.assertGreaterEqual(len(found), 6, f"found the ball in {len(found)} of {len(best)} pictures")
        self.assertGreaterEqual(sum(1 for b in found if b >= 0.5), len(found) - 1, f"overlaps {found}")

    def test_the_default_confidence_finds_about_two_thirds_on_this_camera(self):
        # A recorded fact, not a hope: at 0.5 the ESP32-CAM pictures give 5 of 8 (the model is less sure on harder pictures),
        # which is why the dashboard has a confidence slider that goes down to 0.25.
        found = [b for b in self.results(0.5) if b >= 0.3]
        self.assertGreaterEqual(len(found), 4, f"found the ball in {len(found)} of 8 pictures")

    def test_stays_quiet_on_an_empty_picture(self):
        self.assertEqual(self.detector.detect(np.full((240, 320, 3), 128, np.uint8), 0.5), [])


class FakeCamera:
    def __init__(self):
        self.frame_id, self.frame = 0, None

    def snapshot(self):
        return self.frame_id, self.frame


class FakeDetector:
    def __init__(self):
        self.calls = []

    def detect(self, rgb, conf):
        self.calls.append(conf)
        return [Detection(0.9, 1, 2, 3, 4)]


class WorkerTest(unittest.TestCase):
    def make(self, factory=None):
        camera = FakeCamera()
        detector = FakeDetector()
        worker = DetectorWorker(camera, factory or (lambda: detector))
        return camera, detector, worker

    def test_does_nothing_while_switched_off(self):
        camera, detector, worker = self.make()
        camera.frame, camera.frame_id = np.zeros((10, 10, 3), np.uint8), 1
        self.assertFalse(worker.step())
        self.assertEqual(detector.calls, [])

    def test_runs_once_per_new_frame_with_the_current_confidence(self):
        camera, detector, worker = self.make()
        worker.enabled, worker.conf = True, 0.65
        camera.frame, camera.frame_id = np.zeros((10, 10, 3), np.uint8), 1
        self.assertTrue(worker.step())
        self.assertFalse(worker.step())                          # the same frame again: skipped
        camera.frame_id = 2
        self.assertTrue(worker.step())
        self.assertEqual(detector.calls, [0.65, 0.65])
        frame_id, dets, ms = worker.latest()
        self.assertEqual(frame_id, 2)
        self.assertEqual(len(dets), 1)
        self.assertGreaterEqual(ms, 0)

    def test_waits_for_the_first_frame(self):
        camera, detector, worker = self.make()
        worker.enabled = True
        self.assertFalse(worker.step())                          # no picture yet
        self.assertIsNone(worker.latest())

    def test_a_model_that_cannot_load_switches_detection_off_and_says_why(self):
        def broken():
            raise FileNotFoundError("no model here")

        camera, detector, worker = self.make(broken)
        worker.enabled = True
        camera.frame, camera.frame_id = np.zeros((10, 10, 3), np.uint8), 1
        self.assertFalse(worker.step())
        self.assertFalse(worker.enabled)
        self.assertIn("no model here", worker.error)

    def test_detect_now_runs_a_single_picture_and_returns_the_time(self):
        camera, detector, worker = self.make()
        worker.conf = 0.4
        dets, ms = worker.detect_now(np.zeros((10, 10, 3), np.uint8))
        self.assertEqual(len(dets), 1)
        self.assertEqual(detector.calls, [0.4])

    def test_the_model_is_loaded_once_and_only_when_needed(self):
        loads = []
        camera, detector, worker = self.make(lambda: loads.append(1) or FakeDetector())
        self.assertEqual(loads, [])                              # creating the worker does not load it
        worker.detect_now(np.zeros((10, 10, 3), np.uint8))
        worker.detect_now(np.zeros((10, 10, 3), np.uint8))
        self.assertEqual(loads, [1])

    def test_stops_when_asked(self):
        camera, detector, worker = self.make()
        worker.start()
        worker.stop_event.set()
        worker.join(timeout=2)
        self.assertFalse(worker.is_alive())


if __name__ == "__main__":
    unittest.main()
