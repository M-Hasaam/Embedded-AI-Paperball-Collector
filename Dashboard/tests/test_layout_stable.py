import sys
import threading
import tkinter as tk
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402

ROOT = None


def setUpModule():
    global ROOT
    try:
        ROOT = tk.Tk()
    except tk.TclError as e:
        raise unittest.SkipTest(f"no display for Tk: {e}")


def tearDownModule():
    if ROOT is not None:
        ROOT.destroy()


class FakeCamera:
    def __init__(self, *args):
        self.lock = threading.Lock()
        self.last_jpeg, self.connected, self.link_kbps = b"", False, 0.0
        self.stop_event = threading.Event()
        self.rotate = None

    def start(self):
        pass

    def snapshot(self):
        return 0, None


class FakeDetector:
    def __init__(self, camera):
        self.enabled, self.conf, self.error = False, 0.5, ""
        self.stop_event = threading.Event()

    def start(self):
        pass

    def latest(self):
        return None

    def latest_frame(self):
        return None


class LayoutStableTest(unittest.TestCase):
    """The columns must keep their size while the numbers and messages inside the cards change: the whole window used to jump
    sideways (and look like it flickered) whenever a text got longer or shorter."""

    def setUp(self):
        patches = [mock.patch.object(dashboard, "CameraWorker", FakeCamera), mock.patch.object(dashboard, "DetectorWorker", FakeDetector),
                   mock.patch.object(dashboard.Dashboard, "_pump", lambda self: None), mock.patch.object(sys, "argv", ["dashboard.py", "--no-ble"])]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.win = tk.Toplevel(ROOT)
        self.app = dashboard.Dashboard(self.win)
        self.win.geometry("1400x900+3000+3000")
        self.addCleanup(self.win.destroy)
        self.win.update()

    def sizes(self):
        self.win.update()
        return (self.win.grid_bbox(0, 1)[2:], self.win.grid_bbox(1, 1)[2:], self.win.grid_bbox(0, 2)[3], self.app.dist_bar.winfo_width())

    def test_the_columns_do_not_move_when_the_texts_change(self):
        app = self.app
        before = self.sizes()
        app.enc_speed_var.set("132.0 cm/s")
        app.run_status_var.set("Turn -5.90978 deg: done (turn error -0.6 deg), then driving 60 cm straight ahead")
        app.cam_info.configure(text="640×480  ·  1234 KB/frame  ·  link 1957 KB/s")
        app.detect_status_var.set("yolo_20261108_152427.jpg: 2 paper balls, best 87%, 31 ms")
        app.car_text.set("Connected to AI_Car_with_a_very_long_name (3.5 s)")
        app.tof_var.set("Gripper 1200 mm · in jaws · no readings")
        self.assertEqual(self.sizes(), before)


    def test_the_gripper_reading_does_not_make_the_window_taller(self):
        # a block of its own for the gripper sensor made the window 91 px taller (957 instead of 866) and cut off the Drive card
        self.win.update_idletasks()
        self.assertLessEqual(self.win.winfo_reqheight(), 880)

    def test_the_gripper_line_never_covers_the_card_title(self):
        # in a narrower window the full text covered the title ("DISTANCE (ULTRASON"): it must get shorter instead
        app = self.app
        app.tof_var.set("Gripper 54 mm · in jaws")
        for width in (1400, 1100, 900, 760):
            self.win.geometry(f"{width}x900+3000+3000")
            self.win.update()
            title = app.tof_title
            self.assertLessEqual(title.winfo_x() + title.winfo_reqwidth(), app.tof_label.winfo_x(), f"covered at {width} px")
            self.assertIn("54", app.tof_shown.get())                      # still says the number

    def test_the_full_text_is_shown_when_there_is_room(self):
        app = self.app
        app.tof_var.set("Gripper 54 mm · in jaws")
        self.win.geometry("1600x900+3000+3000")
        self.win.update()
        self.assertEqual(app.tof_shown.get(), "Gripper 54 mm · in jaws")


if __name__ == "__main__":
    unittest.main()
