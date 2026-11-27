import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.ball_distance import (CAMERA_AHEAD_CM, FOV_DEG, CalibrationError, DistanceModel, ball_angle_deg, draw_guides, turn_to_ball,  # noqa: E402
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
        self.assertTrue(52.0 <= FOV_DEG <= 58.0, FOV_DEG)

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


def as_seen(forward, left, fov=FOV_DEG, ahead=CAMERA_AHEAD_CM):
    """Where across the picture, and how far from the camera, a ball at (forward, left) cm of the car's turning point appears."""
    x, y = forward - ahead, left
    x_ratio = 0.5 - math.tan(math.atan2(y, x)) * 0.5 / math.tan(math.radians(fov) / 2)
    return x_ratio, math.hypot(x, y)


def turned(forward, left, deg):
    """The same ball after the car has turned `deg` degrees to the left."""
    a = math.radians(deg)
    return forward * math.cos(a) + left * math.sin(a), left * math.cos(a) - forward * math.sin(a)


class TurnToBallTest(unittest.TestCase):
    SPOT_X, SPOT_CM = 0.474, 16.1

    def test_no_turn_for_a_ball_on_the_grab_spot(self):
        self.assertAlmostEqual(turn_to_ball(self.SPOT_X, self.SPOT_CM, self.SPOT_X, self.SPOT_CM), 0.0, places=6)

    def test_after_the_turn_the_ball_is_on_the_line_that_ends_at_the_grab_spot(self):
        goal = math.radians(math.degrees(math.atan((0.5 - self.SPOT_X) * 2 * math.tan(math.radians(FOV_DEG) / 2))))
        goal_left = self.SPOT_CM * math.sin(goal)                       # how far left of straight ahead the grab spot is
        for forward, left in ((35, 6), (35, -6), (60, 15), (140, -30), (32, 1)):
            x_ratio, cm = as_seen(forward, left)
            deg = turn_to_ball(x_ratio, cm, self.SPOT_X, self.SPOT_CM)
            self.assertAlmostEqual(turned(forward, left, deg)[1], goal_left, delta=0.15, msg=f"ball at {forward},{left}")

    def test_close_up_the_car_turns_less_than_the_camera_angle(self):
        near = turn_to_ball(0.3, 17.0, 0.5, 16.0)
        self.assertAlmostEqual(near / ball_angle_deg(0.3), 17.0 / (17.0 + CAMERA_AHEAD_CM), delta=0.05)

    def test_far_away_the_car_turns_about_the_camera_angle(self):
        self.assertAlmostEqual(turn_to_ball(0.3, 5000.0, 0.5, 16.0), ball_angle_deg(0.3), delta=0.2)

    def test_it_predicts_what_the_car_really_did(self):
        # logged on the car: (distance cm, ball before, ball after - both as picture offset from the centre, + = left), gyro turn
        for cm, before, after, gyro in ((127.3, -0.1680, -0.0627, -5.8), (17.1, -0.1436, 0.0233, -4.9),
                                        (19.7, -0.3084, 0.0287, -11.0), (16.5, -0.1409, 0.0153, -4.6),
                                        (18.4, -0.1307, 0.1599, -8.5)):
            deg = turn_to_ball(0.5 - before, cm, 0.5 - after, cm)
            self.assertAlmostEqual(deg, gyro, delta=1.0, msg=f"at {cm} cm")


if __name__ == "__main__":
    unittest.main()
