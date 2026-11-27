import sys
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402

ROOT = None
LONG_CAMERA = "Bridge connected but no frames - close other camera viewers (camera_yolo.py etc.) and check the camera"
LONG_CAR = "AI_Car not advertising - trying its saved connection..."


def setUpModule():
    global ROOT
    try:
        ROOT = tk.Tk()
    except tk.TclError as e:
        raise unittest.SkipTest(f"no display for Tk: {e}")
    ROOT.withdraw()


def tearDownModule():
    if ROOT is not None:
        ROOT.destroy()


class TextFitTest(unittest.TestCase):
    def test_a_text_that_fits_is_left_alone(self):
        self.assertEqual(dashboard.elide("Live", len, 10), "Live")

    def test_a_long_text_is_cut_with_dots_to_fit(self):
        shown = dashboard.elide("abcdefghij klmnop", len, 12)
        self.assertEqual(shown, "abcdefghij\u2026")
        self.assertLessEqual(len(shown), 12)

    def test_the_room_is_shared_and_a_short_text_leaves_the_rest_to_the_long_one(self):
        self.assertEqual(dashboard.share_width(500, 100, 200), (100, 200))
        self.assertEqual(dashboard.share_width(500, 80, 900), (80, 420))
        self.assertEqual(dashboard.share_width(500, 900, 80), (420, 80))
        self.assertEqual(dashboard.share_width(500, 900, 900), (250, 250))
        self.assertEqual(dashboard.share_width(20, 900, 900), (60, 60))          # never shrunk to nothing


class HeaderTest(unittest.TestCase):
    """The top bar: two connection messages and four buttons, which a long message must not push out of the window."""

    def build(self, width):
        win = tk.Toplevel(ROOT)
        self.addCleanup(win.destroy)
        win.geometry(f"{width}x80+-4000+-4000")
        app = types.SimpleNamespace(car_text=tk.StringVar(master=win, value="Searching..."),
                                    cam_text=tk.StringVar(master=win, value="Starting..."))
        for name in ("open_path_window", "open_pulse_window", "open_moves_window", "open_collect_window"):
            setattr(app, name, lambda: None)
        for name in ("_build_header", "_badge", "_button", "_fit_badges"):
            setattr(app, name, getattr(dashboard.Dashboard, name).__get__(app))
        header = tk.Frame(win)
        header.pack(fill="x")
        app._build_header(header)
        win.update()
        return win, header, app

    def check_nothing_is_pushed_out(self, win, header, app):
        win.update()
        items = sorted(header.winfo_children(), key=lambda w: w.winfo_x())
        for widget in items:
            self.assertGreaterEqual(widget.winfo_width(), widget.winfo_reqwidth(), widget)       # not squeezed
            self.assertGreaterEqual(widget.winfo_x(), 0, widget)
            self.assertLessEqual(widget.winfo_x() + widget.winfo_width(), header.winfo_width(), widget)
        for left, right in zip(items, items[1:]):
            self.assertLessEqual(left.winfo_x() + left.winfo_width(), right.winfo_x(), (left, right))

    def test_long_messages_leave_every_button_in_place(self):
        for width in (1200, 1500, 1900):
            win, header, app = self.build(width)
            app.cam_text.set(LONG_CAMERA)
            app.car_text.set(LONG_CAR)
            self.check_nothing_is_pushed_out(win, header, app)

    def test_the_message_is_shortened_on_screen_but_kept_whole(self):
        win, header, app = self.build(1300)
        app.cam_text.set(LONG_CAMERA)
        app.car_text.set(LONG_CAR)
        win.update()
        self.assertTrue(app.cam_badge.shown.get().endswith("\u2026"))
        self.assertTrue(LONG_CAMERA.startswith(app.cam_badge.shown.get()[:-1]))
        self.assertEqual(app.cam_text.get(), LONG_CAMERA)

    def test_short_messages_are_shown_whole_again(self):
        win, header, app = self.build(1300)
        app.cam_text.set(LONG_CAMERA)
        win.update()
        app.cam_text.set("Live")
        app.car_text.set(LONG_CAR)
        win.update()
        self.assertEqual(app.cam_badge.shown.get(), "Live")

    def test_a_wider_window_shows_more(self):
        win, header, app = self.build(1200)
        app.cam_text.set(LONG_CAMERA)
        win.update()
        narrow = len(app.cam_badge.shown.get())
        win.geometry("1900x80+-4000+-4000")
        win.update()
        self.assertGreater(len(app.cam_badge.shown.get()), narrow)


if __name__ == "__main__":
    unittest.main()
