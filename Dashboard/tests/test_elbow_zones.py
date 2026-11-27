import re
import sys
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers import elbow_limit  # noqa: E402
from helpers.elbow_limit import elbow_window  # noqa: E402

import dashboard  # noqa: E402

FIRMWARE = Path(__file__).resolve().parents[2] / "Embedded" / "esp32-s3" / "AI_Car" / "AI_Car.ino"


class ElbowWindowTest(unittest.TestCase):
    def test_every_shoulder_angle_gives_the_chart_window(self):
        chart = [(40, 60, 135, 180), (61, 70, 120, 180), (71, 80, 105, 180), (81, 119, 100, 180), (120, 120, 100, 170),
                 (121, 139, 95, 170), (140, 149, 95, 160), (150, 159, 100, 150), (160, 170, 120, 145)]
        for first, last, lo, hi in chart:
            for shoulder in range(first, last + 1):
                self.assertEqual(elbow_window(shoulder), (lo, hi), shoulder)

    def test_beyond_the_table_the_end_rows_apply(self):
        self.assertEqual(elbow_window(10), elbow_window(40))
        self.assertEqual(elbow_window(180), elbow_window(170))

    def test_it_is_the_cars_table(self):
        text = FIRMWARE.read_text(encoding="utf-8")

        def numbers(name):
            found = re.search(rf"{name}\[LIMIT_ROWS\]\s*=\s*\{{([^}}]*)\}}", text)
            return [int(v) for v in found.group(1).split(",")]

        self.assertEqual(list(elbow_limit.SHOULDER), numbers("LIMIT_SHOULDER"))
        self.assertEqual(list(elbow_limit.LOWEST), numbers("LIMIT_ELBOW_MIN"))
        self.assertEqual(list(elbow_limit.HIGHEST), numbers("LIMIT_ELBOW_MAX"))
        self.assertEqual(elbow_limit.MARGIN, int(re.search(r"LIMIT_MARGIN\s*=\s*(\d+)", text).group(1)))


ROOT = None


def setUpModule():
    global ROOT
    try:
        ROOT = tk.Tk()
    except tk.TclError:
        ROOT = None
    else:
        ROOT.withdraw()


def tearDownModule():
    if ROOT is not None:
        ROOT.destroy()


class ElbowZonesTest(unittest.TestCase):
    """The red zones on the elbow slider: where the elbow may not go at the shoulder's angle."""

    def setUp(self):
        if ROOT is None:
            self.skipTest("no display for Tk")
        self.win = tk.Toplevel(ROOT)
        self.addCleanup(self.win.destroy)
        self.win.geometry("420x120+-4000+-4000")

        def scale(lo, hi, value):
            made = tk.Scale(self.win, from_=lo, to=hi, orient="horizontal", showvalue=False, sliderlength=16, width=14, bd=0,
                            highlightthickness=0)
            made.pack(fill="x")
            made.set(value)
            return made

        self.elbow, self.shoulder = scale(80, 180, 120), scale(40, 170, 90)
        self.app = types.SimpleNamespace(servo_scales=[None, self.elbow, self.shoulder])
        self.app.limit_zones = dashboard.Dashboard._make_elbow_zones(self.app, self.elbow)

    def draw(self, shoulder):
        self.shoulder.set(shoulder)
        self.win.update()
        dashboard.Dashboard._draw_elbow_zones(self.app)
        self.win.update()
        return self.app.limit_zones

    def x(self, value):
        return self.elbow.coords(value)[0]

    def test_the_forbidden_part_is_marked_whatever_the_elbow_is_asked_to_do(self):
        for asked in (80, 120, 175):                                  # also while nothing is crossed
            self.elbow.set(asked)
            low, high = self.draw(40)
            self.assertTrue(low.winfo_ismapped(), asked)
            self.assertAlmostEqual(low.winfo_x() + low.winfo_width(), self.x(135), delta=2)
            self.assertLessEqual(low.winfo_x(), self.x(80))
            self.assertFalse(high.winfo_ismapped())                   # 180 is the servo's own end: nothing forbidden above

    def test_both_ends_are_marked_when_the_window_is_narrow(self):
        low, high = self.draw(170)                                    # 120 - 145
        self.assertAlmostEqual(low.winfo_x() + low.winfo_width(), self.x(120), delta=2)
        self.assertAlmostEqual(high.winfo_x(), self.x(145), delta=2)
        self.assertGreaterEqual(high.winfo_x() + high.winfo_width(), self.x(180))

    def test_the_zones_follow_the_shoulder_and_the_sliders_range(self):
        low, _ = self.draw(40)
        wide = low.winfo_width()
        low, _ = self.draw(90)                                        # 100 and up
        self.assertLess(low.winfo_width(), wide)
        self.elbow.configure(from_=140)                               # the slider's range starts above every limit
        low, high = self.draw(90)
        self.assertFalse(low.winfo_ismapped())
        self.assertFalse(high.winfo_ismapped())

    def test_the_zones_leave_the_slider_handle_visible(self):
        low, _ = self.draw(40)
        self.assertLess(low.winfo_height(), self.elbow.winfo_height() / 2 + 1)
        self.assertGreaterEqual(low.winfo_y(), self.elbow.winfo_height() / 2 - 1)      # along the bottom of the track

    def test_moving_the_shoulder_or_changing_a_range_redraws(self):
        calls = []
        text = types.SimpleNamespace(set=lambda value: None)
        app = types.SimpleNamespace(servo_values=[text] * 4, dragging=set(), _draw_elbow_zones=lambda: calls.append("draw"),
                                    root=types.SimpleNamespace(after_idle=lambda fn: fn()), servo_scales=[self.elbow] * 4,
                                    _send_servo=lambda index: None)
        for index, expected in ((0, 0), (1, 0), (2, 1), (3, 0)):
            calls.clear()
            dashboard.Dashboard._slider_changed(app, index, "120")
            self.assertEqual(len(calls), expected, index)
        for index, expected in ((0, 0), (1, 1), (2, 1)):
            calls.clear()
            dashboard.Dashboard._range_changed(app, index, 80, 180, text)
            self.assertEqual(len(calls), expected, index)


if __name__ == "__main__":
    unittest.main()
