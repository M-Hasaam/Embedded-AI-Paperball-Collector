import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.ball_distance import (FOV_DEG, CalibrationError, DistanceModel, ball_angle_deg, draw_guides,  # noqa: E402
                                   draw_spot_marker, load_model, save_model)

# the line used throughout: 1 / distance = 0.0005 * y - 0.1, so y_for(20) = 300, y_for(40) = 250, y_for(80) = 225
SAMPLES = [(20, 300), (40, 250), (80, 225)]


def line_model():
    return DistanceModel.fit(SAMPLES)


class ModelTest(unittest.TestCase):
    def test_fit_recovers_the_line(self):
        model = line_model()
        self.assertAlmostEqual(model.a, 0.0005, places=6)
        self.assertAlmostEqual(model.b, -0.1, places=6)
        self.assertAlmostEqual(model.distance_cm(300), 20.0, places=4)
        self.assertAlmostEqual(model.y_for(40), 250.0, places=3)
        for real, estimated, error in model.errors():
            self.assertAlmostEqual(error, 0.0, places=3)

    def test_a_noisy_fit_reports_each_points_error(self):
        model = DistanceModel.fit([(20, 300), (40, 252), (80, 225)])
        errors = model.errors()
        self.assertEqual(len(errors), 3)
        for real, estimated, error in errors:
            self.assertAlmostEqual(error, estimated - real, places=9)       # (real, estimated, estimated - real)
        self.assertTrue(any(abs(e[2]) > 0.1 for e in errors), "the fit cannot pass through all three points")

    def test_fit_needs_three_samples_and_a_rising_curve(self):
        with self.assertRaises(CalibrationError) as ctx:
            DistanceModel.fit([(20, 300), (40, 250)])
        self.assertIn("3", str(ctx.exception))
        with self.assertRaises(CalibrationError):
            DistanceModel.fit([(20, 200), (40, 250), (80, 300)])           # the ball came closer but its bottom edge went up
        with self.assertRaises(CalibrationError):
            DistanceModel.fit([(0, 300), (40, 250), (80, 225)])             # a distance of zero

    def test_no_distance_above_the_horizon(self):
        self.assertIsNone(line_model().distance_cm(150))

    def test_save_and_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.json"
            save_model(path, line_model())
            loaded = load_model(path)
        self.assertAlmostEqual(loaded.a, 0.0005, places=6)
        self.assertAlmostEqual(loaded.distance_cm(250), 40.0, places=3)
        self.assertEqual(len(loaded.errors()), 3)

    def test_load_missing_file_gives_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(load_model(Path(tmp) / "nothing.json"))

    def test_load_broken_file_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, text in (("a.json", "not json"), ("b.json", '{"a": 1}'), ("c.json", "[1, 2]"), ("d.json", '{"a": "x", "b": 2}')):
                path = Path(tmp) / name
                path.write_text(text)
                with self.assertRaises(CalibrationError, msg=name):
                    load_model(path)


class AngleTest(unittest.TestCase):
    def test_the_field_of_view_matches_what_the_car_measured(self):
        # the car turned 9, 14.6, -9.7 and 4.9 degrees (gyro, accurate to ~1 deg) and the ball moved 1.5-1.7x that much in the
        # picture with a 55 degree assumption: the real horizontal field of view is about 36 degrees
        self.assertTrue(34.0 <= FOV_DEG <= 40.0, FOV_DEG)

    def test_angle(self):
        self.assertEqual(ball_angle_deg(0.5), 0.0)
        self.assertAlmostEqual(ball_angle_deg(0.0), FOV_DEG / 2, places=3)         # the left edge: half the field of view
        self.assertAlmostEqual(ball_angle_deg(1.0), -FOV_DEG / 2, places=3)
        self.assertAlmostEqual(ball_angle_deg(0.0, 55.0), 27.5, places=3)          # another field of view can be given
        self.assertGreater(ball_angle_deg(0.2), ball_angle_deg(0.4))        # more to the left = a larger angle (+ = left)


class DrawTest(unittest.TestCase):
    def black(self, height=480, width=640):
        return np.zeros((height, width, 3), np.uint8)

    def test_guides_are_drawn_at_the_calibrated_rows(self):
        frame = self.black()
        shown = draw_guides(frame, line_model())
        self.assertFalse(frame.any())                                        # the input picture is untouched
        col = 630                                                            # far from the labels on the left
        model = line_model()
        self.assertEqual(tuple(int(v) for v in shown[int(round(model.y_for(20))), col]), (255, 0, 0))     # near: red (RGB)
        self.assertEqual(tuple(int(v) for v in shown[int(round(model.y_for(40))), col]), (255, 255, 0))   # middle: yellow
        self.assertEqual(tuple(int(v) for v in shown[int(round(model.y_for(60))), col]), (0, 255, 0))     # far: green
        self.assertEqual(tuple(int(v) for v in shown[int(round(model.y_for(80))), col]), (0, 255, 0))

    def test_a_line_outside_the_picture_is_skipped(self):
        shown = draw_guides(self.black(height=240), line_model())            # the 20 and 40 cm rows (300, 250) are below the bottom
        self.assertEqual(shown.shape, (240, 640, 3))
        self.assertTrue(shown[int(round(line_model().y_for(60))), 630].any())

    def test_no_model_returns_the_same_picture(self):
        frame = self.black()
        self.assertIs(draw_guides(frame, None), frame)

    def test_spot_marker_marks_the_column(self):
        frame = self.black()
        shown = draw_spot_marker(frame, 0.25)
        self.assertFalse(frame.any())
        self.assertTrue(shown[:, 158:163].any())                             # a dashed vertical line near x = 160


if __name__ == "__main__":
    unittest.main()
