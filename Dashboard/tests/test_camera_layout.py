import functools
import sys
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402

ROOT = None


def setUpModule():
    global ROOT
    try:
        ROOT = tk.Tk()
    except tk.TclError as e:
        raise unittest.SkipTest(f"no display for Tk: {e}")
    ROOT.geometry("+3000+3000")                  # off the visible screen: the layout is still worked out


def tearDownModule():
    if ROOT is not None:
        ROOT.destroy()


def build_card(parent):
    """The dashboard's camera card, built with the real code on a stand-in for the dashboard."""
    D = dashboard.Dashboard
    app = types.SimpleNamespace(
        rotate_var=tk.StringVar(master=ROOT, value="none"), run_status_var=tk.StringVar(master=ROOT),
        detect_var=tk.BooleanVar(master=ROOT), lines_var=tk.BooleanVar(master=ROOT), detect_status_var=tk.StringVar(master=ROOT),
        detect_conf_var=tk.StringVar(master=ROOT, value="50%"), camera=types.SimpleNamespace(rotate=None),
        detector=types.SimpleNamespace(conf=0.5), last_frame_id=0)
    for name in ("_card", "_button", "_scale", "_detect_conf_changed"):
        setattr(app, name, functools.partial(getattr(D, name), app))
    app._save_frame = app._detect_toggled = app._test_photo_dialog = app.back_to_live = lambda: None
    return app, D._build_camera_card(app, parent)


class CameraCardLayoutTest(unittest.TestCase):
    def setUp(self):
        self.parent = tk.Frame(ROOT)
        self.parent.grid(row=0, column=0)
        self.parent.rowconfigure(0, weight=1)
        self.parent.columnconfigure(0, weight=1)
        self.addCleanup(self.parent.destroy)

    def test_a_longer_status_message_does_not_widen_the_column(self):
        # The camera column's width decides how much room the right-hand cards get: a status text that grew the card squeezed
        # them down to a sliver.
        app, body = build_card(self.parent)
        ROOT.update()
        before = self.parent.winfo_reqwidth()
        app.detect_status_var.set("yolo_20261108_152427.jpg: 2 paper balls, best 87%, 31 ms")
        ROOT.update()
        self.assertEqual(self.parent.winfo_reqwidth(), before)

    def test_the_card_has_a_distance_lines_checkbox_tied_to_lines_var(self):
        app, body = build_card(self.parent)
        boxes = []

        def walk(widget):
            for child in widget.winfo_children():
                if isinstance(child, tk.Checkbutton):
                    boxes.append(child)
                walk(child)
        walk(body)
        lines = [b for b in boxes if b.cget("text") == "Distance lines"]
        self.assertEqual(len(lines), 1)
        self.assertEqual(str(lines[0].cget("variable")), str(app.lines_var))
        self.assertFalse(app.lines_var.get())                      # off by default

    def test_the_controls_are_not_wider_than_the_rest_of_the_card(self):
        # the Save frame row is the card's width before the detector was added; the new rows must fit in it
        app, body = build_card(self.parent)
        ROOT.update()
        widths = sorted(child.winfo_reqwidth() for child in body.winfo_children() if isinstance(child, tk.Frame))
        self.assertLessEqual(widths[-1], 520, f"the widest row asks for {widths[-1]} px: {widths}")

    def test_a_short_window_cuts_the_picture_never_the_controls(self):
        app, body = build_card(self.parent)
        photo = tk.PhotoImage(master=ROOT, width=640, height=480)           # a camera picture makes the label ask for 640x480
        app.video.configure(image=photo, text="")
        self.parent.grid_propagate(False)
        self.parent.configure(width=600, height=300)                         # a window too short for the whole picture
        ROOT.update()
        rows = [child for child in body.winfo_children() if isinstance(child, tk.Frame)]
        self.assertTrue(rows)
        for row in rows:
            self.assertGreaterEqual(row.winfo_height(), row.winfo_reqheight(), "a control row is cut off")


if __name__ == "__main__":
    unittest.main()
