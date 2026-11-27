import sys
import tempfile
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard  # noqa: E402
from helpers.moves import arm_step, describe_step, drive_step, turn_step, wait_step  # noqa: E402
from test_moves import FakeCar  # noqa: E402

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


def make_app():
    app = types.SimpleNamespace(root=ROOT, moves_window=None)
    app._button = lambda parent, text, command, **kw: tk.Button(parent, text=text, command=command)
    app.servo_scales = [tk.Scale(ROOT, from_=0, to=180) for _ in range(4)]
    app.speed_scales = [tk.Scale(ROOT, from_=1, to=600) for _ in range(4)]
    return app


class MovesWindowTest(unittest.TestCase):
    def setUp(self):
        self.app = make_app()
        self.car = FakeCar()
        logs = tempfile.TemporaryDirectory()                       # test runs must not write into the real log
        self.addCleanup(logs.cleanup)
        self.log_path = Path(logs.name) / "moves_log.txt"
        self.window = dashboard.MovesWindow(self.app, self.car, log_path=self.log_path)
        self.addCleanup(self.window.close)

    def set_pose(self, angles, speed=30):
        for i, angle in enumerate(angles):
            self.app.servo_scales[i].set(angle)
            self.app.speed_scales[i].set(speed)

    def test_a_new_move_starts_smooth(self):
        self.assertIs(self.window.move["smooth"], True)
        self.assertTrue(self.window.smooth_var.get())

    def test_smooth_setting_is_saved_and_loaded(self):
        self.window.add_wait(1)
        self.window.set_smooth(False)
        self.assertIs(self.window.move["smooth"], False)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "s.json"
            self.window.save(path)
            other = dashboard.MovesWindow(self.app, FakeCar())
            self.addCleanup(other.close)
            other.load(path)
        self.assertIs(other.move["smooth"], False)
        self.assertFalse(other.smooth_var.get())

    def test_a_file_without_the_setting_loads_as_not_smooth(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "old.json"
            path.write_text('{"name": "old", "version": 1, "steps": [{"type": "wait", "seconds": 1}]}')
            self.window.load(path)
        self.assertFalse(self.window.smooth_var.get())
        self.assertNotIn("smooth", self.window.move)

    def test_add_arm_pose_captures_all_sliders(self):
        self.set_pose([90, 150, 120, 45])
        self.window.add_arm_pose(0.5)
        self.assertEqual(self.window.move["steps"], [arm_step({0: (90, 30), 1: (150, 30), 2: (120, 30), 3: (45, 30)}, wait=0.5)])

    def test_a_new_move_has_both_straight_assists_on(self):
        self.assertIs(self.window.move["assist_gyro"], True)
        self.assertIs(self.window.move["assist_encoders"], True)
        self.assertTrue(self.window.assist_gyro_var.get())
        self.assertTrue(self.window.assist_enc_var.get())

    def test_assist_settings_are_saved_and_loaded(self):
        self.window.add_drive(10, 50)
        self.window.set_assist(gyro=True, encoders=False)
        self.assertIs(self.window.move["assist_encoders"], False)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "a.json"
            self.window.save(path)
            other = dashboard.MovesWindow(self.app, FakeCar())
            self.addCleanup(other.close)
            other.load(path)
        self.assertTrue(other.assist_gyro_var.get())
        self.assertFalse(other.assist_enc_var.get())

    def test_a_file_without_assist_settings_loads_with_both_on(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "old.json"
            path.write_text('{"name": "old", "version": 1, "steps": [{"type": "drive", "cm": 20}]}')
            self.window.set_assist(gyro=False, encoders=False)
            self.window.load(path)
        self.assertTrue(self.window.assist_gyro_var.get())
        self.assertTrue(self.window.assist_enc_var.get())

    def test_add_drive_with_a_speed(self):
        self.window.add_drive(10, 50)
        self.window.add_drive(30)
        self.assertEqual(self.window.move["steps"], [drive_step(10, 50), drive_step(30, 100)])

    def test_add_delete_and_reorder(self):
        self.window.add_drive(20)
        self.window.add_turn(90)
        self.window.select(0)
        self.window.move_selected(1)
        self.assertEqual(self.window.move["steps"], [turn_step(90), drive_step(20)])
        self.window.delete_selected()                      # the moved step (the drive) stays selected
        self.assertEqual(self.window.move["steps"], [turn_step(90)])

    def test_edit_value_and_recapture_pose(self):
        self.window.add_drive(20)
        self.window.add_arm_pose(0.5)
        self.window.select(0)
        self.window.edit_selected_value(35)
        self.assertEqual(self.window.move["steps"][0], drive_step(35))
        self.set_pose([10, 100, 100, 10], speed=60)
        self.window.select(1)
        self.window.recapture_selected_pose()
        self.assertEqual(self.window.move["steps"][1], arm_step({0: (10, 60), 1: (100, 60), 2: (100, 60), 3: (10, 60)}, wait=0.5))

    def test_save_load_roundtrip(self):
        self.window.add_drive(20)
        self.window.add_wait(1)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "x.json"
            self.window.save(path)
            other = dashboard.MovesWindow(self.app, FakeCar())
            self.addCleanup(other.close)
            other.load(path)
        self.assertEqual(other.move, self.window.move)
        self.assertEqual(other.listbox.get(0, "end"), tuple(describe_step(s, dashboard.SERVO_NAMES) for s in other.move["steps"]))

    def test_play_refused_when_move_invalid(self):
        self.window.move["steps"].append(arm_step({1: (200, 30)}))
        self.assertTrue(self.window.errors())
        self.assertFalse(self.window.play())
        self.assertEqual(self.car.calls, [])
        self.assertIn("Step 1", self.window.status_var.get())

    def test_play_and_stop(self):
        self.window.move["steps"].append(wait_step(5))
        self.assertTrue(self.window.play())
        self.assertTrue(self.window.player.active)
        self.window.stop()
        self.assertFalse(self.window.player.active)
        self.assertIn("stop", self.car.kinds())

    def test_stop_without_playing_still_stops_the_car(self):
        self.window.stop()
        self.assertEqual(self.car.kinds(), ["stop"])

    def test_second_play_while_playing_is_refused(self):
        self.window.move["steps"].append(arm_step({1: (150, 30)}))
        self.assertTrue(self.window.play())
        first = self.car.kinds().count("servo")
        self.assertFalse(self.window.play())
        self.assertEqual(self.car.kinds().count("servo"), first)

    def log(self):
        return self.log_path.read_text(encoding="utf-8")

    def test_every_test_play_is_written_to_a_log_file(self):
        self.window.add_drive(5, 50)
        self.window.play()
        self.assertIn("step 1/1: Drive: forward 5 cm at 50%", self.log())
        self.car.motion_result, self.car.why = False, "the car gave up"
        self.car.finish_motion()
        self.window.tick()
        self.assertIn("the drive did not finish normally: the car gave up", self.log())
        self.assertEqual(self.log().count("step 1/1"), 1)
        self.assertRegex(self.log().splitlines()[0], r"^\d\d:\d\d:\d\d move ")

    def test_stop_and_close_are_logged(self):
        self.window.move["steps"].append(wait_step(5))
        self.window.play()
        self.window.stop()
        self.assertIn("stopped by you", self.log())
        self.window.play()
        self.window.close()
        self.assertIn("window closed", self.log())

    def test_a_log_file_that_cannot_be_written_does_not_stop_the_move(self):
        self.window.log_path = self.log_path / "no" / "such.txt"
        self.window.move["steps"].append(wait_step(5))
        self.assertTrue(self.window.play())

    def test_play_refused_when_move_is_empty(self):
        self.assertFalse(self.window.play())
        self.assertEqual(self.car.calls, [])

    def test_closing_the_window_stops_playback(self):
        self.window.move["steps"].append(wait_step(5))
        self.window.play()
        self.window.close()
        self.assertFalse(self.window.player.active)
        self.assertIn("stop", self.car.kinds())


if __name__ == "__main__":
    unittest.main()
