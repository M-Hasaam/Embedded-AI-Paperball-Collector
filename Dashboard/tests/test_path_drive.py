"""Path driving: the car stops its drives itself, fast while far and slow for the last part, and everything is logged."""
import math
import sys
import tempfile
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.path_settings import Odometer, PathSettings, load_settings  # noqa: E402


class SimCar:
    """A car that drives and turns the real PathTracker: drives end after a second, covering `gain` of what was asked."""

    def __init__(self, tracker):
        self.tracker = tracker
        self.drives, self.turns, self.sent = [], [], []
        self.gain, self.short, self.ok, self.why = 1.0, 1.0, True, ""
        self.starts = True
        self.now, self._until, self._left = 0.0, None, 0.0
        self.counted = [0.0, 0.0]
        self.distance = None                      # the front sensor ...
        self.wall_y = None                        # ... or an obstacle standing at this height of the map
        self.cancelled = 0
        self.pose = "other"
        self.over = 0.0                           # how far past the asked distance a drive of the car's own ends

    def sense(self):
        return self.distance if self.wall_y is None else max(0.0, self.wall_y - self.tracker.y)

    # what the runner needs from the car
    def cancel(self):
        self.cancelled += 1
        self._until = None

    def drive(self, cm, pct, assist):
        if not self.starts:
            return False
        self.drives.append((round(cm, 1), pct, assist))
        self._until, self._left = self.now + 1.0, cm * self.gain + math.copysign(self.over, cm)
        return True

    def drive_busy(self):
        if self._until is None:
            return False
        if self.now < self._until:
            return True
        self.tracker._advance(self._left, self.now)
        self.counted = [c + self._left for c in self.counted]
        self._until = None
        return False

    def drive_ok(self):
        return self.ok

    def drive_why(self):
        return self.why

    def wheels(self):
        return tuple(self.counted)

    def send(self, command):
        self.sent.append(command)
        if command.startswith("Y"):
            asked = float(command[1:])
            self.turns.append(asked)
            self.tracker.yaw += asked - math.copysign(min(self.short, abs(asked)), asked)
        if command == "S":
            self._until = None


class PathDriveTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.tracker = dashboard.PathTracker()
        self.car = SimCar(self.tracker)
        self.settings = PathSettings()
        self.finished = []
        self.runner = dashboard.PathRunner(self.tracker, self.car.send, lambda: self.car.sense(), self.car,
                                           lambda: self.settings, self.finished.append, lambda: self.car.pose)
        self.runner.log_file = Path(self.tmp.name) / "path_runner_log.txt"

    def run_path(self, points, seconds=120.0, path=True, label="Run path"):
        steps = dashboard.plan_steps(points)
        self.assertTrue(self.runner.start(steps, label, self.car.now, path=path))
        for _ in range(int(seconds * 20)):
            if not self.runner.active:
                return
            self.car.now = round(self.car.now + 0.05, 2)
            self.runner.tick(self.car.now, 0.0)

    def log(self):
        return self.runner.log_file.read_text(encoding="utf-8")

    def off(self, point):
        return math.hypot(self.tracker.x - point[0], self.tracker.y - point[1])

    def test_a_far_point_is_driven_fast_then_slow_for_the_last_part(self):
        self.run_path([(0.0, 80.0)])
        self.assertEqual(self.car.drives, [(60.0, 100, 1), (20.0, 50, 1)])
        self.assertLess(self.off((0.0, 80.0)), 3.0)
        self.assertEqual(self.finished, [True])

    def test_a_near_point_is_driven_slowly_in_one_go(self):
        self.run_path([(0.0, 18.0)])
        self.assertEqual(self.car.drives, [(18.0, 50, 1)])

    def test_no_short_burst_at_full_speed(self):
        # seen on the car: 3 cm at 100% before the slow part went 8 cm
        for cm, drives in ((23.4, [(23.4, 50, 1)]), (29.0, [(29.0, 50, 1)]), (31.0, [(11.0, 100, 1), (20.0, 50, 1)])):
            self.setUp()
            self.run_path([(0.0, cm)])
            self.assertEqual(self.car.drives, drives, cm)

    def test_the_settings_decide_the_distances_speeds_and_assist(self):
        self.settings = PathSettings().changed(slow_within_cm=10, fast_pct=50, slow_pct=100, assist="both")
        self.run_path([(0.0, 50.0)])
        self.assertEqual(self.car.drives, [(40.0, 50, 3), (10.0, 100, 3)])

    def test_every_point_of_a_square_is_reached(self):
        corners = [(0.0, 40.0), (40.0, 40.0), (40.0, 0.0)]
        for n in range(1, 4):
            self.setUp()
            self.run_path(corners[:n])
            self.assertLess(self.off(corners[n - 1]), 3.5, n)

    def test_a_drive_that_ends_short_is_finished_with_another(self):
        self.car.gain = 0.8
        self.run_path([(0.0, 80.0)])
        self.assertGreaterEqual(len(self.car.drives), 3)
        self.assertLess(self.off((0.0, 80.0)), 3.0)

    def test_going_past_the_point_is_corrected_once_by_reversing(self):
        self.car.gain = 1.3
        self.run_path([(0.0, 40.0)])
        self.assertTrue(any(cm < 0 for cm, _, _ in self.car.drives), self.car.drives)
        self.assertEqual(sum(1 for cm, _, _ in self.car.drives if cm < 0), 1)
        self.assertEqual(self.finished, [True])

    def test_a_car_that_never_gets_there_does_not_try_for_ever(self):
        self.car.gain = 0.1
        self.run_path([(0.0, 80.0)])
        self.assertFalse(self.runner.active)
        self.assertLessEqual(len(self.car.drives), 4)
        self.assertIn("still", self.log())

    def test_turns_ask_for_a_little_more_on_a_path_but_not_for_a_single_turn(self):
        self.run_path([(-30.0, 0.0)])                                 # 90 degrees to the left
        self.assertAlmostEqual(self.car.turns[0], 91.0, places=1)
        self.setUp()
        self.runner.start([["turn_to", 30.0]], "Turn +30", self.car.now)
        self.runner.tick(0.05, 0.0)
        self.assertAlmostEqual(self.car.turns[0], 30.0, places=1)

    def test_corners_are_recorded_so_the_way_back_can_be_driven(self):
        self.run_path([(0.0, 40.0), (40.0, 40.0)])
        self.assertEqual(len(self.tracker.corners), 3)
        self.assertAlmostEqual(self.tracker.corners[1][1], 40.0, delta=3.0)

    def test_an_obstacle_ahead_stops_a_forward_drive(self):
        self.car.distance = 12.0
        self.run_path([(0.0, 80.0)])
        self.assertEqual(self.finished, [False])
        self.assertIn("obstacle ahead", self.runner.status)
        self.assertIn("S", self.car.sent)

    def test_a_failed_or_refused_drive_stops_the_path_with_the_reason(self):
        self.car.ok, self.car.why = False, "the car gave up"
        self.run_path([(0.0, 80.0)])
        self.assertIn("the car gave up", self.runner.status)
        self.setUp()
        self.car.starts = False
        self.run_path([(0.0, 80.0)])
        self.assertIn("could not start the drive", self.runner.status)

    def test_the_log_tells_the_whole_story(self):
        self.run_path([(0.0, 80.0), (30.0, 80.0)])
        text = self.log()
        for part in ("START Run path: goto(0,80), goto(30,80)", "SETTINGS slow_within_cm 20, ", "trust right",
                     "AIM at corner (0.0,80.0): distance 80.0 cm", "DRIVE fast: 60.0 cm at 100%", "DRIVE slow: 20.0 cm at 50%",
                     "driving: wheels L", "DRIVE end: asked 60.0 cm, wheels L +60.0 R +60.0 cm",
                     "POINT 1 of 2 reached: 0.", "cm from it", "TURN start", "DONE Run path in", "cm from the last point"):
            self.assertIn(part, text)
        self.assertRegex(text.splitlines()[0], r"^\d\d:\d\d:\d\d +\d")          # a clock time, to match the other logs

    def test_wheels_that_disagree_are_pointed_out(self):
        original = self.car.drive_busy

        def one_counter_runs_away():
            busy = original()
            if not busy:
                self.car.counted[0] += 40.0
            return busy

        self.car.drive_busy = one_counter_runs_away
        self.run_path([(0.0, 18.0)])
        self.assertIn("THE WHEELS DISAGREE", self.log())


class StopDistanceTest(unittest.TestCase):
    """How close to an obstacle a path stops depends on how the arm stands; near one the car creeps up instead of reacting."""
    setUp, run_path, log = PathDriveTest.setUp, PathDriveTest.run_path, PathDriveTest.log

    def gap(self):
        return self.car.wall_y - self.tracker.y

    def test_each_arm_pose_has_its_own_stop_distance(self):
        for pose, stop in (("closed", 5.0), ("default", 10.0), ("other", 20.0)):
            self.setUp()
            self.car.pose, self.car.wall_y = pose, 50.0
            self.run_path([(0.0, 80.0)])
            self.assertEqual(self.finished, [False], pose)
            self.assertIn("obstacle ahead", self.runner.status)
            self.assertTrue(stop - 0.5 <= self.gap() <= stop + 2.5, (pose, self.gap()))
            self.assertEqual(len(self.tracker.obstacles), 1)

    def test_the_distances_can_be_changed(self):
        self.settings = PathSettings().changed(stop_closed_cm=12)
        self.car.pose, self.car.wall_y = "closed", 60.0
        self.run_path([(0.0, 80.0)])
        self.assertTrue(11.5 <= self.gap() <= 14.5, self.gap())

    def test_towards_an_obstacle_it_only_drives_slowly_and_never_further_than_the_room(self):
        self.car.pose, self.car.wall_y = "closed", 40.0
        self.run_path([(0.0, 80.0)])
        self.assertTrue(self.car.drives)
        for cm, pct, _ in self.car.drives:
            self.assertEqual(pct, 50, self.car.drives)
            self.assertLessEqual(cm, 35.0)

    def test_it_ends_at_the_stop_distance_although_the_cars_drives_run_a_little_past(self):
        # seen on the car: asked for 16 cm up to the obstacle it drove 17.6 and stood 2 cm from it instead of 5
        for pose, stop, wall in (("closed", 5.0, 21.0), ("default", 10.0, 70.0), ("closed", 5.0, 140.0)):
            self.setUp()
            self.car.pose, self.car.wall_y, self.car.over = pose, wall, 1.5
            self.run_path([(0.0, 150.0)])
            self.assertTrue(stop - 0.5 <= self.gap() <= stop + 2.0, (pose, wall, self.gap(), self.car.drives))

    def test_a_far_obstacle_changes_nothing_and_the_point_is_reached(self):
        self.car.wall_y = 200.0
        self.run_path([(0.0, 80.0)])
        self.assertEqual(self.car.drives, [(60.0, 100, 1), (20.0, 50, 1)])
        self.assertEqual(self.finished, [True])

    def test_an_obstacle_just_behind_the_point_means_a_slow_drive_to_the_point(self):
        self.car.pose, self.car.wall_y = "closed", 48.0
        self.run_path([(0.0, 40.0)])
        self.assertEqual(self.car.drives, [(40.0, 50, 1)])
        self.assertEqual(self.finished, [True])

    def start_and_wait_for_a_drive(self, point, pose="closed"):
        self.car.pose = pose
        self.runner.start(dashboard.plan_steps([point]), "Run path", self.car.now, path=True)
        for _ in range(100):
            self.car.now = round(self.car.now + 0.05, 2)
            self.runner.tick(self.car.now, 0.0)
            if self.runner.state == "driving":
                return
        self.fail("it never drove")

    def one_tick(self):
        self.car.now = round(self.car.now + 0.05, 2)
        self.runner.tick(self.car.now, 0.0)

    def test_something_appearing_during_a_fast_drive_slows_the_car_down_first(self):
        self.start_and_wait_for_a_drive((0.0, 80.0))
        self.assertEqual(self.car.drives[-1][1], 100)
        self.car.distance = 20.0                                      # closer than 5 + the braking room
        self.one_tick()
        self.assertEqual(self.car.cancelled, 1)
        self.assertIn("S", self.car.sent)
        self.assertTrue(self.runner.active)                           # not given up: it creeps up to it
        for _ in range(30):
            self.one_tick()
        self.assertEqual(self.car.drives[-1][1], 50)
        self.assertAlmostEqual(self.car.drives[-1][0], 13.5, delta=0.5)      # 15 cm of room, less what a drive runs past

    def test_something_appearing_during_a_slow_drive(self):
        self.start_and_wait_for_a_drive((0.0, 20.0))
        self.car.distance = 9.0                                       # new, and near the stop distance: look again
        self.one_tick()
        self.assertTrue(self.runner.active)
        self.assertEqual(self.car.cancelled, 1)
        self.setUp()
        self.start_and_wait_for_a_drive((0.0, 20.0))
        self.car.distance = 3.0                                       # already closer than allowed: stop at once
        self.one_tick()
        self.assertFalse(self.runner.active)
        self.assertIn("obstacle ahead", self.runner.status)

    def test_the_log_says_which_pose_and_distance_count(self):
        self.car.pose, self.car.wall_y = "closed", 30.0
        self.run_path([(0.0, 80.0)])
        self.assertIn("arm closed: stop 5 cm from an obstacle", self.log())


class ArmPoseTest(unittest.TestCase):
    def test_the_pose_is_told_from_the_servo_sliders(self):
        for angles, pose in (([100, 135, 40, 90], "closed"), ([99, 137, 42, 90], "closed"), ([90, 120, 90, 90], "default"),
                             ([100, 120, 90, 90], "other"), ([100, 135, 60, 90], "other")):
            self.assertEqual(dashboard.arm_pose(angles), pose, angles)


class DashboardPathTest(unittest.TestCase):
    def test_the_runner_may_drive_while_it_is_active(self):
        sent = []
        app = types.SimpleNamespace(connected=types.SimpleNamespace(is_set=lambda: True), runner=types.SimpleNamespace(active=True),
                                    encoder_seq=0, _send=sent.append)
        self.assertFalse(dashboard.Dashboard.drive_encoder(app, 20, 50, 1))
        self.assertTrue(dashboard.Dashboard.drive_encoder(app, 20, 50, 1, from_runner=True))
        self.assertEqual(sent, ["M20 50 1"])

    def test_the_runners_car_uses_the_cars_own_drive(self):
        calls = []
        app = types.SimpleNamespace(drive_encoder=lambda cm, pct, assist, from_runner=False: calls.append((cm, pct, assist, from_runner)) or True,
                                    drive_busy=lambda: False, drive_failed=True, drive_fail_reason="why", wheel_cm=(1.0, 2.0))
        car = dashboard.RunnerCar(app)
        self.assertTrue(car.drive(10, 50, 1))
        self.assertEqual(calls, [(10, 50, 1, True)])
        self.assertEqual((car.drive_busy(), car.drive_ok(), car.drive_why(), car.wheels()), (False, False, "why", (1.0, 2.0)))
        app.drive_wait_seq = 5
        car.cancel()
        self.assertIsNone(app.drive_wait_seq)

    def test_the_map_trusts_the_chosen_wheel_during_a_drive(self):
        fed = []
        text = types.SimpleNamespace(set=lambda value: None)
        app = types.SimpleNamespace(_enc_ignore_until=0.0, tracker=types.SimpleNamespace(feed_encoder=lambda d, s: fed.append(d), cmd="S"),
                                    enc_dist_var=text, enc_speed_var=text, enc_wheels_var=text, wheel_cm=None, wheel_speed=0.0,
                                    odometer=Odometer(), path_settings=PathSettings(), drive_state=0)
        dashboard.Dashboard._show_encoder(app, 100.0, 100.0, 0, 0, 1.0)
        app.drive_state = 1
        dashboard.Dashboard._show_encoder(app, 165.4, 109.8, 0, 0, 1.2)
        self.assertAlmostEqual(fed[-1], 9.8)

    def test_settings_are_saved_and_told_to_the_car(self):
        with tempfile.TemporaryDirectory() as tmp:
            sent = []
            app = types.SimpleNamespace(path_settings=PathSettings(), path_settings_file=Path(tmp) / "p.json", _send=sent.append)
            app._send_wheel_rule = lambda: dashboard.Dashboard._send_wheel_rule(app)
            dashboard.Dashboard.apply_path_settings(app, trust="left", slow_within_cm="35")
            self.assertEqual((app.path_settings.trust, app.path_settings.slow_within_cm), ("left", 35.0))
            self.assertEqual(load_settings(app.path_settings_file), app.path_settings)
            self.assertEqual(sent, ["W2 4 15"])


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


class PathSettingsWindowTest(unittest.TestCase):
    def setUp(self):
        if ROOT is None:
            self.skipTest("no display for Tk")
        self.applied = []
        tracker = dashboard.PathTracker()
        self.app = types.SimpleNamespace(root=ROOT, tracker=tracker, path_settings=PathSettings(), path_window=None,
                                         runner=types.SimpleNamespace(status="Idle", active=False))

        def apply(**values):
            self.applied.append(values)
            self.app.path_settings = self.app.path_settings.changed(**values)

        self.app.apply_path_settings = apply
        for name in ("path_return", "path_abort", "path_clear", "path_test_drive"):
            setattr(self.app, name, lambda: None)
        self.app.path_run_plan = lambda points: None
        self.app._button = dashboard.Dashboard._button.__get__(self.app)
        self.window = dashboard.PathWindow(self.app)
        self.addCleanup(self.window.close)

    def test_the_boxes_show_the_settings(self):
        shown = {name: var.get() for name, var in self.window.setting_vars.items()}
        self.assertEqual(shown, {"stop_closed_cm": "5", "stop_default_cm": "10", "stop_other_cm": "20",
                                 "fast_pct": "100", "slow_within_cm": "20", "slow_pct": "50", "arrive_cm": "3",
                                 "turn_within_deg": "2", "turn_extra_deg": "1", "differ_cm": "4", "differ_pct": "15",
                                 "trust": "right", "assist": "gyro", "manual_guard": "on", "hand_slow_cm": "50"})

    def test_a_change_is_applied_and_shown_as_kept(self):
        self.window.setting_vars["slow_within_cm"].set("500")
        self.window.setting_vars["trust"].set("left")
        self.window.apply_settings()
        self.assertEqual((self.app.path_settings.slow_within_cm, self.app.path_settings.trust), (200.0, "left"))
        self.assertEqual(self.window.setting_vars["slow_within_cm"].get(), "200")

    def test_everything_fits_in_the_smallest_window(self):
        self.window.win.geometry("{}x{}+-4000+-4000".format(*self.window.win.minsize()))
        self.window.win.update()
        rows = [w for w in self.window.win.winfo_children() if isinstance(w, tk.Frame)]
        for row in rows:
            self.assertLessEqual(row.winfo_y() + row.winfo_height(), self.window.win.winfo_height(), row)
            for widget in row.winfo_children():
                self.assertLessEqual(widget.winfo_x() + widget.winfo_width(), row.winfo_width(), (row, widget))
                self.assertGreaterEqual(widget.winfo_width(), min(widget.winfo_reqwidth(), row.winfo_width()), widget)
        self.assertGreater(self.window.canvas.winfo_height(), 150)


if __name__ == "__main__":
    unittest.main()
