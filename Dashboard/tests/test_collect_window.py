import sys
import tempfile
import time
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard  # noqa: E402
from helpers.ball_detector import Detection, DetectionResult  # noqa: E402
from helpers.grab_spot import GrabSpot, load_spot  # noqa: E402
from helpers.moves import arm_step, new_move, save_move, wait_step  # noqa: E402
from test_moves import FakeCar  # noqa: E402

ROOT = None
BOX = (300, 300, 340, 340)               # centre x = 320 of 640 -> 0.5, bottom edge 340


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


class FakeDetector:
    def __init__(self):
        self.enabled, self.conf, self.error, self.result = False, 0.5, "", None

    def latest_frame(self):
        return self.result


def make_app():
    app = types.SimpleNamespace(root=ROOT, moves_window=None, collect_window=None, detector=FakeDetector(), distance_model=None,
                                grab_spot=None, dist_cm=None, dist_stamp=None, detect_var=tk.BooleanVar(master=ROOT))
    app._button = lambda parent, text, command, **kw: tk.Button(parent, text=text, command=command)
    return app


class CollectWindowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)
        self.app = make_app()
        self.car = FakeCar()
        self.log_path = self.dir / "collect_log.txt"
        self.window = dashboard.CollectWindow(self.app, self.car, moves_dir=self.dir, log_path=self.log_path)
        self.addCleanup(self.window.close)
        self.now = time.monotonic()
        self.frame = 0

    def feed(self, boxes=(BOX,), n=1):
        """n detector results at the current time, ticked into the window."""
        for _ in range(n):
            self.frame += 1
            dets = [Detection(0.9, *b) for b in boxes]
            self.app.detector.result = DetectionResult(self.frame, dets, 30.0, self.now, 640, 480)
            self.window.tick(self.now)

    def reading(self, cm, age=0.0):
        self.app.dist_cm, self.app.dist_stamp = cm, self.now - age

    def steady(self):
        self.feed(n=3)

    def write_move(self, name="pick", steps=None, bad=False):
        move = new_move(name)
        move["steps"] = [arm_step({1: (200, 30)})] if bad else (steps if steps is not None else [wait_step(0.1)])
        save_move(self.dir / f"{name}.json", move)

    def ready_to_collect(self):
        self.app.grab_spot = GrabSpot(0.5, 16.0)
        self.write_move()
        self.window.select_move("pick")

    # ---- calibration ----
    def test_record_needs_a_steady_ball(self):
        self.assertFalse(self.window.record_sample(40))
        self.assertIn("ball", self.window.status_var.get())
        self.steady()
        self.assertTrue(self.window.record_sample(40))
        self.assertEqual(self.window.samples, [(40.0, 340.0)])

    def test_clear_samples(self):
        self.steady()
        self.window.record_sample(40)
        self.window.clear_samples()
        self.assertEqual(self.window.samples, [])

    def test_fit_and_save(self):
        path = self.dir / "c.json"
        self.window.samples = [(20.0, 300.0), (40.0, 250.0), (80.0, 225.0)]
        self.assertTrue(self.window.fit_and_save(path))
        self.assertTrue(path.exists())
        self.assertAlmostEqual(self.app.distance_model.distance_cm(250), 40.0, places=3)
        self.window.samples = [(20.0, 300.0), (40.0, 250.0)]
        self.assertFalse(self.window.fit_and_save(self.dir / "d.json"))
        self.assertIn("3", self.window.status_var.get())
        self.assertFalse((self.dir / "d.json").exists())

    # ---- grab spot ----
    def test_set_grab_spot(self):
        path = self.dir / "spot.json"
        self.steady()
        self.reading(16.0)
        self.assertTrue(self.window.set_grab_spot(path, now=self.now))
        spot = load_spot(path)
        self.assertEqual((spot.x_ratio, spot.ultrasonic_cm), (0.5, 16.0))
        self.assertEqual(self.app.grab_spot, spot)

    def test_grab_spot_needs_a_ball_and_a_good_reading(self):
        path = self.dir / "spot.json"
        self.reading(16.0)
        self.assertFalse(self.window.set_grab_spot(path, now=self.now))            # no ball yet
        self.steady()
        self.reading(None)
        self.assertFalse(self.window.set_grab_spot(path, now=self.now))            # no echo
        self.reading(16.0, age=3.0)
        self.assertFalse(self.window.set_grab_spot(path, now=self.now))            # an old reading
        self.app.dist_stamp = None
        self.assertFalse(self.window.set_grab_spot(path, now=self.now))            # never a reading
        self.assertFalse(path.exists())
        self.assertIsNone(self.app.grab_spot)

    def test_grab_spot_keeps_the_camera_estimate_when_calibrated(self):
        self.app.distance_model = dashboard.DistanceModel.fit([(20, 300), (40, 250), (80, 225)])
        self.app.distance_model.points = []
        self.steady()
        self.reading(16.0)
        self.assertTrue(self.window.set_grab_spot(self.dir / "s.json", now=self.now))
        self.assertAlmostEqual(self.app.grab_spot.camera_cm, self.app.distance_model.distance_cm(340), places=6)

    # ---- collect ----
    def test_move_names_lists_the_json_files(self):
        self.write_move("b")
        self.write_move("a")
        (self.dir / "notes.txt").write_text("x")
        self.assertEqual(self.window.move_names(), ["a", "b"])

    def test_collect_refuses(self):
        self.steady()
        self.reading(40)

        def refused(message):
            self.assertFalse(self.window.collect(self.now), msg=message)
            self.assertEqual(self.car.calls, [], msg=message)

        refused("no grab spot")
        self.app.grab_spot = GrabSpot(0.5, 16.0)
        refused("no move selected")
        self.write_move("bad", bad=True)
        self.window.select_move("bad")
        refused("a move with errors")
        self.write_move()
        self.window.select_move("pick")
        self.car.connected_flag = False
        refused("not connected")
        self.car.connected_flag = True
        self.assertTrue(self.window.collect(self.now))
        self.car.calls.clear()
        self.assertFalse(self.window.collect(self.now))                            # already running
        self.assertEqual(self.car.calls, [])

    def test_collect_runs_to_the_move(self):
        self.ready_to_collect()
        self.assertTrue(self.window.collect(self.now))
        self.assertTrue(self.app.detector.enabled)
        self.assertTrue(self.app.detect_var.get())
        for _ in range(100):
            self.now += 0.1
            self.reading(16.0)
            judging = self.window.approach._state in ("picking", "verifying")
            self.feed(boxes=() if judging else (BOX,))                    # picked: the ball is not on the floor any more
            if not self.window.approach.active:
                break
        self.assertIs(self.window.approach.ok, True)
        self.assertTrue(self.window.status_var.get().endswith("Done: picked") or "Done" in self.window.status_var.get())
        self.assertEqual([c for c in self.car.calls if c[0] == "start_drive"], [])

    def test_collect_plays_the_move_as_it_is_in_the_file_now(self):
        # the move was edited and saved (in the Moves window) after it was chosen here: Collect must not play the old copy
        self.ready_to_collect()
        self.write_move("pick", steps=[wait_step(0.1), wait_step(0.2)])
        self.assertTrue(self.window.collect(self.now))
        self.assertEqual(len(self.window.approach.move["steps"]), 2)

    def test_collect_refuses_when_the_move_file_became_invalid(self):
        self.ready_to_collect()
        self.write_move("pick", bad=True)
        self.assertFalse(self.window.collect(self.now))
        self.assertIn("will not play", self.window.status_var.get())

    def test_every_run_is_written_to_a_log_file(self):
        self.ready_to_collect()
        self.window.collect(self.now)
        for _ in range(12):
            self.now += 0.1
            self.reading(40)
            self.feed()
        self.window.stop()
        text = self.log_path.read_text(encoding="utf-8")
        self.assertIn("Collect started", text)
        self.assertIn("pick", text)                                   # the move's name
        self.assertIn("sensor 40.0 cm", text)                         # a decision line
        self.assertIn("Stopped: stopped by you", text)                # why it ended

    def test_a_log_file_that_cannot_be_written_does_not_break_collect(self):
        self.window.log_path = self.dir / "no_such_folder" / "x" / "log.txt"
        self.ready_to_collect()
        self.assertTrue(self.window.collect(self.now))

    def test_stop_ends_collect_and_stops_the_car(self):
        self.ready_to_collect()
        self.window.collect(self.now)
        for _ in range(12):                                                        # far away: it hops
            self.now += 0.1
            self.reading(40)
            self.feed()
        self.assertIn("start_drive", self.car.kinds())
        self.window.stop()
        self.assertFalse(self.window.approach.active)
        self.assertIn(("stop",), self.car.calls)

    def test_close_stops_it(self):
        self.ready_to_collect()
        self.window.collect(self.now)
        approach = self.window.approach
        self.window.close()
        self.assertFalse(approach.active)
        self.assertIn(("stop",), self.car.calls)
        self.assertIsNone(self.app.collect_window)


if __name__ == "__main__":
    unittest.main()
