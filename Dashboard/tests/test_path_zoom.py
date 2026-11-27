"""Zooming and moving the Path map: two fingers apart or together zoom, two fingers moving together drag the map."""
import ctypes
import subprocess
import sys
import tempfile
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.path_settings import PathSettings  # noqa: E402

ROOT = None


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


def at(x, y, delta=0):
    return types.SimpleNamespace(x=x, y=y, delta=delta)


class PathZoomTest(unittest.TestCase):
    def setUp(self):
        app = types.SimpleNamespace(root=ROOT, tracker=dashboard.PathTracker(), path_settings=PathSettings(), path_window=None,
                                    runner=types.SimpleNamespace(status="Idle", active=False))
        for name in ("path_return", "path_abort", "path_clear", "path_test_drive"):
            setattr(app, name, lambda: None)
        app.path_run_plan = app.apply_path_settings = lambda *a, **k: None
        app._button = dashboard.Dashboard._button.__get__(app)
        logs = tempfile.TemporaryDirectory()
        self.addCleanup(logs.cleanup)
        self.log_path = Path(logs.name) / "touchpad_log.txt"
        self.window = dashboard.PathWindow(app, touch_log=self.log_path)
        self.addCleanup(self.window.close)
        self.window.win.geometry("660x760+-4000+-4000")
        self.window.win.update()
        self.window.refresh(force=True)

    def world(self, x, y):
        """The map position (cm) under the pixel (x, y)."""
        cx, cy, scale, w, h = self.window.tf
        return cx + (x - w / 2) / scale, cy - (y - h / 2) / scale

    def scale(self):
        return self.window.tf[2]

    def assertSamePlace(self, a, b):
        self.assertAlmostEqual(a[0], b[0], places=3)
        self.assertAlmostEqual(a[1], b[1], places=3)

    def test_a_pinch_zooms_in_and_out_and_keeps_the_place_under_the_pointer(self):
        before, scale = self.world(100, 80), self.scale()
        self.window._pinch(at(100, 80, 120))                          # fingers moving apart
        self.assertGreater(self.scale(), scale * 1.2)
        self.assertSamePlace(self.world(100, 80), before)
        self.window._pinch(at(100, 80, -120))                         # fingers coming closer
        self.assertAlmostEqual(self.scale(), scale, places=4)

    def test_a_pinch_in_small_steps_zooms_a_little_each(self):
        scale = self.scale()
        self.window._pinch(at(200, 200, 15))
        small = self.scale() / scale
        self.assertTrue(1.0 < small < 1.06, small)
        for _ in range(7):                                            # eight small steps make one notch
            self.window._pinch(at(200, 200, 15))
        self.assertAlmostEqual(self.scale() / scale, dashboard.PathWindow.PINCH_STEP, places=4)

    def test_two_fingers_moving_together_drag_the_map_and_never_zoom(self):
        scale, under = self.scale(), self.world(200, 200)
        self.window._two_finger_move(0, 50)                           # downwards: the map goes down with the fingers
        self.assertSamePlace(self.world(200, 250), under)
        self.window._two_finger_move(-40, 0)                          # to the left
        self.assertSamePlace(self.world(160, 250), under)
        self.assertEqual(self.scale(), scale)

    def test_touchpad_movement_arrives_as_scrolling(self):
        scale, under = self.scale(), self.world(200, 200)
        step = dashboard.PathWindow.PAN_PX
        self.window._gesture(False, False, 100, 200, 200)             # vertical scroll "up": the content moves down
        self.assertSamePlace(self.world(200, 200 + 100 * step), under)
        self.window._gesture(True, False, 100, 200, 200)              # horizontal scroll "right": the content moves left
        self.assertSamePlace(self.world(200 - 100 * step, 200 + 100 * step), under)
        self.assertEqual(self.scale(), scale)
        self.window._gesture(False, True, 120, 200, 200)              # with Ctrl: a pinch
        self.assertGreater(self.scale(), scale)

    def test_the_tk_events_do_the_same_where_windows_is_not_asked(self):
        self.window.release_touch()                                   # as on another system
        scale, under = self.scale(), self.world(150, 150)
        self.window.canvas.event_generate("<MouseWheel>", delta=100, x=150, y=150)
        self.window.win.update()
        self.assertEqual(self.scale(), scale)
        self.assertGreater(self.world(150, 150)[1], under[1])         # the content went down: higher up the map is here now
        self.window.canvas.event_generate("<Shift-MouseWheel>", delta=100, x=150, y=150)
        self.window.win.update()
        self.assertEqual(self.scale(), scale)
        self.window.canvas.event_generate("<Control-MouseWheel>", delta=120, x=150, y=150)
        self.window.win.update()
        self.assertGreater(self.scale(), scale)

    def test_zooming_stops_at_sensible_ends(self):
        for _ in range(80):
            self.window._pinch(at(200, 200, 120))
        self.assertAlmostEqual(self.window.view[2], dashboard.PathWindow.CLOSEST_CM)
        for _ in range(80):
            self.window._pinch(at(200, 200, -120))
        self.assertAlmostEqual(self.window.view[2], dashboard.PathWindow.FARTHEST_CM)

    def test_the_buttons_zoom_about_the_middle(self):
        w, h = self.window.tf[3], self.window.tf[4]
        middle, scale = self.world(w / 2, h / 2), self.scale()
        self.window.zoom_in_button.invoke()
        self.assertGreater(self.scale(), scale)
        self.assertSamePlace(self.world(w / 2, h / 2), middle)
        self.window.zoom_out_button.invoke()
        self.assertAlmostEqual(self.scale(), scale, places=4)

    def test_a_zoomed_map_stays_put_while_the_car_moves_and_fit_shows_everything_again(self):
        fitted = self.window.tf
        self.window._pinch(at(100, 80, 240))
        zoomed = self.window.tf
        self.window.app.tracker.x = 400.0
        self.window.refresh(force=True)
        self.assertEqual(self.window.tf, zoomed)
        self.window.fit_button.invoke()
        self.assertIsNone(self.window.view)
        self.assertLess(self.scale(), fitted[2])                      # wider now: the car at 400 cm is in view

    def drag(self, start, end):
        self.window._press(at(*start))
        self.window._drag(at(*end))
        self.window._release(at(*end))

    def test_dragging_moves_the_map_and_places_no_point(self):
        under = self.world(100, 100)
        scale = self.scale()
        self.drag((100, 100), (180, 140))
        self.assertEqual(self.window.waypoints, [])
        self.assertSamePlace(self.world(180, 140), under)             # the map went with the pointer
        self.assertAlmostEqual(self.scale(), scale, places=6)

    def test_a_click_still_places_a_point_where_it_was_clicked_also_when_zoomed(self):
        self.window._pinch(at(300, 200, 360))
        expected = self.world(120, 90)
        self.drag((120, 90), (122, 91))                               # a hand that trembles a little: still a click
        self.assertEqual(len(self.window.waypoints), 1)
        self.assertAlmostEqual(self.window.waypoints[0][0], self.world(122, 91)[0], delta=2.0 / self.scale() + 0.01)
        self.assertAlmostEqual(self.window.waypoints[0][1], expected[1], delta=2.0 / self.scale() + 0.01)

    def post(self, message, delta, flags=0, at_pixel=(150, 150)):
        """Send the map the message Windows sends for a wheel or a two-finger gesture."""
        canvas = self.window.canvas
        x, y = canvas.winfo_rootx() + at_pixel[0], canvas.winfo_rooty() + at_pixel[1]
        wparam = ((delta & 0xFFFF) << 16) | flags
        lparam = ((y & 0xFFFF) << 16) | (x & 0xFFFF)
        user32 = ctypes.WinDLL("user32")
        user32.PostMessageW.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_size_t, ctypes.c_ssize_t]
        user32.PostMessageW(canvas.winfo_id(), message, wparam, ctypes.c_ssize_t(ctypes.c_int32(lparam).value).value)
        self.window.win.update()
        self.window._poll_touch()

    @unittest.skipUnless(sys.platform == "win32", "Windows messages")
    def test_windows_messages_drag_and_zoom_the_map(self):
        WHEEL, HWHEEL, CTRL = 0x020A, 0x020E, 0x0008
        self.assertIsNotNone(self.window.touch)
        scale, under = self.scale(), self.world(150, 150)
        self.post(WHEEL, 100)
        self.assertEqual(self.scale(), scale)
        self.assertGreater(self.world(150, 150)[1], under[1])
        under = self.world(150, 150)
        self.post(HWHEEL, 100)                                        # sideways: Tk alone never passes this on
        self.assertGreater(self.world(150, 150)[0], under[0])
        self.assertEqual(self.scale(), scale)
        place = self.world(150, 150)
        self.post(WHEEL, 120, CTRL)
        self.assertGreater(self.scale(), scale)
        self.assertSamePlace(self.world(150, 150), place)
        self.post(WHEEL, -120, CTRL)
        self.assertAlmostEqual(self.scale(), scale, places=4)

    @unittest.skipUnless(sys.platform == "win32", "Windows messages")
    def test_every_gesture_is_written_to_the_touchpad_log(self):
        self.post(0x020E, -40)
        self.post(0x020A, 30, 0x0008)
        text = self.log_path.read_text(encoding="utf-8")
        for part in ("sideways -40 -> drag", "vertical +30 with Ctrl -> zoom"):
            self.assertIn(part, text)

    @unittest.skipUnless(sys.platform == "win32", "Windows messages")
    def test_gestures_inside_the_running_program_do_not_crash_it(self):
        # the window messages are read by a callback inside Tk's own loop: run that loop for real, in its own process
        script = Path(__file__).with_name("touch_mainloop_check.py")
        done = subprocess.run([sys.executable, str(script)], capture_output=True, text=True, timeout=60)
        self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
        self.assertIn("handled 240 gestures", done.stdout)

    def test_the_zoom_buttons_sit_on_the_map(self):
        self.window.win.update()
        for button in (self.window.zoom_in_button, self.window.zoom_out_button, self.window.fit_button):
            self.assertTrue(button.winfo_ismapped())
            self.assertIs(button.master.master, self.window.canvas)


if __name__ == "__main__":
    unittest.main()
