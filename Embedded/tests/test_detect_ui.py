import sys
import tempfile
import types
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from ball_detector import Detection  # noqa: E402


class Var:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeDetector:
    """What the dashboard's DetectorWorker offers: enabled, conf, error, detect_now()."""

    def __init__(self, found=None, error=None):
        self.enabled, self.conf, self.error = False, 0.5, ""
        self.found = found if found is not None else [Detection(0.87, 10, 20, 60, 70)]
        self.fail = error
        self.seen = []

    def detect_now(self, rgb):
        if self.fail:
            raise RuntimeError(self.fail)
        self.seen.append(rgb.shape)
        return self.found, 12.0


def fake_app(detector=None):
    return types.SimpleNamespace(detector=detector or FakeDetector(), detect_status_var=Var(""), detect_conf_var=Var(""),
                                 detect_var=Var(False), test_photo=None, last_frame_id=5)


def write_picture(folder, name="photo.png", size=(100, 120)):
    path = Path(folder) / name
    ok, data = cv2.imencode(".png", np.zeros((size[1], size[0], 3), np.uint8))
    path.write_bytes(data.tobytes())
    return path


class PhotoTestTest(unittest.TestCase):
    def test_runs_the_model_and_keeps_the_picture_with_its_boxes(self):
        app = fake_app()
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(dashboard.Dashboard.show_test_photo(app, write_picture(tmp)))
        self.assertEqual(app.detector.seen, [(120, 100, 3)])            # the model got the picture, as RGB rows x columns
        self.assertEqual(app.test_photo.shape, (120, 100, 3))
        self.assertTrue(app.test_photo.any())                           # the green box was drawn on it
        self.assertEqual(app.detect_status_var.get(), "photo: 1 paper ball, best 87%, 12 ms")     # short: the row has little room
        self.assertEqual(app.last_frame_id, -1)                         # the camera area is redrawn at once

    def test_a_file_that_is_not_a_picture_is_reported_not_crashed_on(self):
        app = fake_app()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "notes.txt"
            path.write_text("not a picture")
            self.assertFalse(dashboard.Dashboard.show_test_photo(app, path))
        self.assertIsNone(app.test_photo)
        self.assertIn("Cannot read", app.detect_status_var.get())

    def test_a_missing_file_is_reported(self):
        app = fake_app()
        self.assertFalse(dashboard.Dashboard.show_test_photo(app, Path(tempfile.gettempdir()) / "no_such_picture_here.jpg"))
        self.assertIn("Cannot read", app.detect_status_var.get())

    def test_a_model_that_cannot_run_is_reported(self):
        app = fake_app(FakeDetector(error="model not found"))
        with tempfile.TemporaryDirectory() as tmp:
            self.assertFalse(dashboard.Dashboard.show_test_photo(app, write_picture(tmp)))
        self.assertIsNone(app.test_photo)
        self.assertIn("model not found", app.detect_status_var.get())

    def test_a_picture_without_balls_says_so(self):
        app = fake_app(FakeDetector(found=[]))
        with tempfile.TemporaryDirectory() as tmp:
            dashboard.Dashboard.show_test_photo(app, write_picture(tmp))
        self.assertIn("no paper balls", app.detect_status_var.get())

    def test_back_to_live_drops_the_photo(self):
        app = fake_app()
        app.test_photo = np.zeros((10, 10, 3), np.uint8)
        dashboard.Dashboard.back_to_live(app)
        self.assertIsNone(app.test_photo)
        self.assertEqual(app.last_frame_id, -1)


class ControlsTest(unittest.TestCase):
    def test_the_checkbox_switches_the_worker_on_and_off(self):
        app = fake_app()
        app.detector.error = "old problem"
        app.detect_var.set(True)
        dashboard.Dashboard._detect_toggled(app)
        self.assertTrue(app.detector.enabled)
        self.assertEqual(app.detector.error, "")
        app.detect_var.set(False)
        dashboard.Dashboard._detect_toggled(app)
        self.assertFalse(app.detector.enabled)
        self.assertEqual(app.detect_status_var.get(), "")

    def test_the_slider_sets_the_confidence(self):
        app = fake_app()
        dashboard.Dashboard._detect_conf_changed(app, "35")
        self.assertAlmostEqual(app.detector.conf, 0.35)
        self.assertEqual(app.detect_conf_var.get(), "35%")

    def test_a_model_that_failed_switches_the_checkbox_off_and_shows_why(self):
        app = fake_app()
        app.detect_var.set(True)
        app.detector.error = "Cannot run the model: model not found"
        dashboard.Dashboard._show_detector_error(app)
        self.assertFalse(app.detect_var.get())
        self.assertIn("model not found", app.detect_status_var.get())
        self.assertEqual(app.detector.error, "")                         # reported once


if __name__ == "__main__":
    unittest.main()
