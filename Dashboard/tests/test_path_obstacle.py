"""An obstacle that stops a path is drawn on the map as a red wall where the front sensor saw it."""
import sys
import tempfile
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import dashboard  # noqa: E402
from helpers.path_settings import PathSettings  # noqa: E402
import test_path_drive as driving  # noqa: E402  (the simulated car; its own tests are not run again)

AHEAD = dashboard.SENSOR_AHEAD_CM


class TrackerObstacleTest(unittest.TestCase):
    def test_it_is_placed_ahead_of_the_car_at_the_sensors_distance(self):
        tracker = dashboard.PathTracker()
        tracker.x, tracker.y = 10.0, 20.0
        tracker.add_obstacle(15.0)                                    # facing up the map
        (x, y, heading), = tracker.obstacles
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 20.0 + AHEAD + 15.0)
        tracker.yaw = tracker.yaw_ref + 90.0                          # facing left
        tracker.add_obstacle(5.0)
        x, y, heading = tracker.obstacles[-1]
        self.assertAlmostEqual(x, 10.0 - AHEAD - 5.0)
        self.assertAlmostEqual(y, 20.0)
        self.assertAlmostEqual(heading, 90.0)

    def test_seeing_the_same_obstacle_again_does_not_add_another(self):
        tracker = dashboard.PathTracker()
        tracker.add_obstacle(15.0)
        tracker.add_obstacle(12.0)
        self.assertEqual(len(tracker.obstacles), 1)
        self.assertAlmostEqual(tracker.obstacles[0][1], AHEAD + 12.0)  # the newest sighting counts
        tracker.add_obstacle(60.0)
        self.assertEqual(len(tracker.obstacles), 2)

    def test_clearing_the_path_removes_them(self):
        tracker = dashboard.PathTracker()
        tracker.add_obstacle(15.0)
        tracker.reset()
        self.assertEqual(tracker.obstacles, [])


class RunnerObstacleTest(unittest.TestCase):
    """Uses the simulated car of the path driving tests."""
    setUp, run_path, log = driving.PathDriveTest.setUp, driving.PathDriveTest.run_path, driving.PathDriveTest.log

    def test_a_stop_for_an_obstacle_records_where_it_is(self):
        self.car.distance = 12.0
        self.run_path([(0.0, 80.0)])
        (x, y, _), = self.tracker.obstacles
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, AHEAD + 12.0)
        self.assertIn("OBSTACLE 12 cm ahead", self.log())
        self.assertIn("obstacle ahead", self.runner.status)

    def test_its_place_is_taken_again_once_the_car_has_stopped(self):
        self.car.distance = 18.0
        self.run_path([(0.0, 80.0)])
        self.tracker.y, self.car.distance = 9.0, 6.0                  # it rolled on: 9 cm further, now 6 cm in front
        self.runner.settle_obstacle(self.car.now + 0.3)               # too soon
        self.assertAlmostEqual(self.tracker.obstacles[0][1], AHEAD + 18.0)
        self.runner.settle_obstacle(self.car.now + 1.5)
        self.assertEqual(len(self.tracker.obstacles), 1)
        self.assertAlmostEqual(self.tracker.obstacles[0][1], 9.0 + AHEAD + 6.0)
        self.assertIn("OBSTACLE after stopping: 6 cm ahead", self.log())

    def test_a_reading_that_is_gone_after_stopping_keeps_the_first_place(self):
        for later in (None, 250.0):
            self.setUp()
            self.car.distance = 18.0
            self.run_path([(0.0, 80.0)])
            self.car.distance = later
            self.runner.settle_obstacle(self.car.now + 1.5)
            self.assertAlmostEqual(self.tracker.obstacles[0][1], AHEAD + 18.0, msg=str(later))

    def test_other_stops_draw_no_wall(self):
        self.car.starts = False
        self.run_path([(0.0, 80.0)])
        self.assertEqual(self.tracker.obstacles, [])
        self.runner.settle_obstacle(self.car.now + 5.0)
        self.assertEqual(self.tracker.obstacles, [])


class HandDrivingTest(unittest.TestCase):
    """Driving forward by hand stops at an obstacle too (it used to drive straight into it)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sent = []
        tracker = dashboard.PathTracker()
        tracker.cmd = "U"
        runner = dashboard.PathRunner(tracker, self.sent.append, lambda: self.app.dist_cm)
        runner.log_file = Path(self.tmp.name) / "path_runner_log.txt"
        status = types.SimpleNamespace(value="")
        status.set = lambda text: setattr(status, "value", text)
        self.app = types.SimpleNamespace(tracker=tracker, runner=runner, path_settings=PathSettings(), dist_cm=12.0,
                                         dist_stamp=100.0, _send=self.sent.append, run_status_var=status,
                                         connected=types.SimpleNamespace(is_set=lambda: True),                                          car_speed_pct=lambda: 100, pose=lambda: "closed", hand_stop="",
                                         hand_slowed=False)

    def guard(self, now=100.1):
        dashboard.Dashboard._obstacle_guard(self.app, now)

    def test_it_slows_down_well_before_the_obstacle_then_brakes_and_takes_over(self):
        self.app.dist_cm = 80.0
        self.guard(100.0)
        self.assertEqual(self.sent, [])
        self.app.dist_cm = 50.0                                       # closer than 5 cm + the slow-down room of 50
        self.guard(100.1)
        self.assertEqual(self.sent, ["P50"])
        self.assertIn("Slowed to 50%: obstacle 50 cm ahead", self.app.hand_stop)
        self.app.dist_cm = 31.0
        self.guard(100.2)
        self.assertFalse(self.app.runner.active)
        self.app.dist_cm = 30.0
        self.guard(100.3)
        self.assertEqual(self.sent, ["P50", "S"])
        self.assertTrue(self.app.runner.active)                       # the car now creeps up by itself: no second press
        self.assertEqual(self.app.runner.label, "Approach")
        log = self.app.runner.log_file.read_text(encoding="utf-8")
        for part in ("HAND slow", "HAND brake", "START Approach"):
            self.assertIn(part, log)

    def test_one_that_appears_suddenly_is_braked_for_at_once(self):
        self.app.dist_cm = 28.0
        self.guard()
        self.assertEqual(self.sent, ["P50", "S"])
        self.assertTrue(self.app.runner.active)

    def test_the_speed_is_given_back_when_the_way_is_clear_or_the_car_stops(self):
        self.app.dist_cm = 50.0
        self.guard(100.0)
        self.app.dist_cm = 57.0                                       # only just out: still careful
        self.guard(100.1)
        self.assertEqual(self.sent, ["P50"])
        self.app.dist_cm = None                                       # no echo says nothing: stays careful
        self.guard(100.2)
        self.assertEqual(self.sent, ["P50"])
        self.app.dist_cm = 90.0
        self.guard(100.3)
        self.assertEqual(self.sent, ["P50", "P100"])
        self.sent.clear()
        self.app.dist_cm = 50.0
        self.guard(100.4)
        self.app.tracker.cmd = "S"
        self.guard(100.5)
        self.assertEqual(self.sent, ["P50", "P100"])

    def test_where_it_brakes_depends_on_the_arm_pose(self):
        self.app.car_speed_pct = lambda: 50
        self.app.dist_cm = 31.0
        self.guard()
        self.assertEqual(self.sent, [])                               # already slow: nothing to slow down
        self.app.pose = lambda: "other"                               # 20 cm there
        self.guard()
        self.assertEqual(self.sent, ["S"])

    def test_it_leaves_the_car_alone_otherwise(self):
        for change in (dict(dist_cm=None), dict(dist_stamp=90.0), dict(path_settings=PathSettings().changed(manual_guard="off")),
                       dict(connected=types.SimpleNamespace(is_set=lambda: False))):
            self.setUp()
            for name, value in change.items():
                setattr(self.app, name, value)
            self.guard()
            self.assertEqual(self.sent, [], change)
        for cmd in ("S", "D", "L", "R"):
            self.setUp()
            self.app.tracker.cmd = cmd
            self.guard()
            self.assertEqual(self.sent, [], cmd)
        self.setUp()
        self.app.runner.active = True                                 # a path looks after itself
        self.guard()
        self.assertEqual(self.sent, [])

    def test_moving_the_speed_slider_does_not_undo_the_slowing_down(self):
        slider = types.SimpleNamespace(value=100.0)
        slider.get = lambda: slider.value
        self.app.car_speed = slider
        for value, slowed, sent in ((100.0, True, "P50"), (40.0, True, "P40"), (100.0, False, "P100")):
            self.sent.clear()
            slider.value, self.app.hand_slowed = value, slowed
            dashboard.Dashboard._send_speed(self.app)
            self.assertEqual(self.sent, [sent], (value, slowed))


class RollingCar(driving.SimCar):
    """The car as the log shows it: a command arrives late, it coasts on after a stop, the front sensor lags behind, and a
    drive of its own ends a little past what it was asked."""
    COMMAND_LATE, SENSOR_LATE, COAST_S = 0.15, 0.2, 0.08

    def __init__(self, tracker, wall):
        super().__init__(tracker)
        self.wall_y, self.over, self.pose = wall, 1.5, "closed"
        self.pct, self.moving, self.queue = 100, False, []
        self.seen = [(0.0, wall)]

    def write(self, command):
        self.queue.append((self.now + self.COMMAND_LATE, command))

    def speed(self):
        return 0.5 * self.pct if self.pct > 50 else 0.62 * self.pct       # 50 cm/s at 100%, 31 cm/s at 50%

    def step(self, dt):
        while self.queue and self.queue[0][0] <= self.now:
            command = self.queue.pop(0)[1]
            if command.startswith("P"):
                self.pct = int(command[1:])
            elif command == "U":
                self.moving = True
            elif command == "S":
                if self.moving:
                    self.tracker._advance(self.speed() * self.COAST_S, self.now)
                self.moving, self._until = False, None
        if self.moving:
            self.tracker._advance(self.speed() * dt, self.now)
        self.drive_busy()
        self.seen.append((self.now, self.wall_y - self.tracker.y))

    def reading(self):
        old = [d for t, d in self.seen if t <= self.now - self.SENSOR_LATE]
        return float(round(old[-1] if old else self.seen[0][1]))


class HandApproachTest(unittest.TestCase):
    """Holding forward towards an obstacle: the car gets to the stop distance by itself, not short of it and not past it."""

    def make(self, wall, pose="closed"):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tracker = dashboard.PathTracker()
        car = self.car = RollingCar(tracker, wall)
        car.pose = pose
        slider = types.SimpleNamespace(get=lambda: 100.0)
        app = self.app = types.SimpleNamespace(
            tracker=tracker, path_settings=PathSettings(), pose=lambda: car.pose, car_speed=slider, car_speed_pct=lambda: 100,
            hand_stop="", hand_slowed=False, dist_cm=wall, dist_stamp=0.0, moves_window=None, collect_window=None,
            _runner_sending=False, connected=types.SimpleNamespace(is_set=lambda: True),
            loop=types.SimpleNamespace(is_running=lambda: True), _write=car.write)
        app._send = lambda message: dashboard.Dashboard._send(app, message)
        app.runner = dashboard.PathRunner(tracker, lambda m: dashboard.Dashboard._runner_send(app, m), lambda: app.dist_cm,
                                          car, lambda: app.path_settings, None, app.pose)
        app.runner.log_file = Path(tmp.name) / "path_runner_log.txt"
        clock = types.SimpleNamespace(monotonic=lambda: car.now, strftime=lambda *a: "00:00:00")
        for owner, name, value in ((dashboard, "time", clock),
                                   (dashboard.asyncio, "run_coroutine_threadsafe", lambda c, loop: None)):
            self.addCleanup(setattr, owner, name, getattr(owner, name))
            setattr(owner, name, value)

    def run_for(self, seconds):
        for _ in range(int(round(seconds / 0.05))):
            self.car.now = round(self.car.now + 0.05, 2)
            self.car.step(0.05)
            self.app.dist_cm, self.app.dist_stamp = self.car.reading(), self.car.now
            dashboard.Dashboard._obstacle_guard(self.app, self.car.now)
            if self.app.runner.active:
                self.app.runner.tick(self.car.now, 0.0)
            self.app.runner.settle_obstacle(self.car.now)

    def gap(self):
        return self.car.wall_y - self.app.tracker.y

    def test_one_long_press_ends_at_the_stop_distance(self):
        for pose, stop in (("closed", 5.0), ("default", 10.0), ("other", 20.0)):
            for wall in (150.0, 63.0, 34.0):
                self.make(wall, pose)
                self.app._send("U")
                self.run_for(12.0)
                self.assertTrue(stop - 1.0 <= self.gap() <= stop + 2.0, (pose, wall, self.gap(), self.car.drives))
                self.assertFalse(self.app.runner.active)
                self.assertIn(f"limit {stop:g} cm", self.app.runner.status)
                self.assertEqual(len(self.app.tracker.obstacles), 1)

    def test_a_press_close_to_it_creeps_the_rest_of_the_way(self):
        for wall in (16.0, 9.0):                      # seen on the car: from 16 cm every press ended differently
            self.make(wall)
            self.app._send("U")
            self.run_for(8.0)
            self.assertTrue(4.0 <= self.gap() <= 7.0, (wall, self.gap(), self.car.drives))
            self.assertTrue(self.car.drives)
            self.assertEqual({pct for _, pct, _ in self.car.drives}, {50})

    def test_at_the_stop_distance_it_does_not_move_forward(self):
        self.make(5.0)
        self.app._send("U")
        self.run_for(3.0)
        self.assertEqual(self.car.drives, [])
        self.assertEqual(self.gap(), 5.0)
        self.assertIn("limit 5 cm", self.app.runner.status)

    def test_letting_go_ends_it(self):
        self.make(150.0)
        self.app._send("U")
        for _ in range(200):
            self.run_for(0.05)
            if self.app.runner.active:
                break
        self.app._send("S")                           # the key is released while the car is taking over
        self.assertFalse(self.app.runner.active)
        before = self.gap()
        self.run_for(4.0)
        self.assertEqual(self.car.drives, [])
        self.assertGreater(self.gap(), before - 8.0)

    def test_a_false_alarm_does_not_send_the_car_off_on_its_own(self):
        self.make(200.0)
        self.app.dist_cm, self.app.dist_stamp = 20.0, 0.0             # one wrong low reading as forward is pressed
        self.app._send("U")
        self.run_for(10.0)
        self.assertLessEqual(200.0 - self.gap(), 47.0)                # slowly, and only as far as it reaches (40 + 5, a little past)
        self.assertTrue(all(pct == 50 for _, pct, _ in self.car.drives), self.car.drives)

    def test_without_a_reading_it_does_not_creep_blind(self):
        self.make(150.0)
        self.app.runner.approach(0.0, 0.2)
        self.car.reading = lambda: None
        self.run_for(2.0)
        self.assertEqual(self.car.drives, [])
        self.assertIn("no distance reading", self.app.runner.status)

    def test_with_the_guard_off_forward_is_just_forward(self):
        self.make(30.0)
        self.app.path_settings = PathSettings().changed(manual_guard="off")
        self.app._send("U")
        self.run_for(0.4)
        self.assertTrue(self.car.moving)
        self.assertFalse(self.app.runner.active)


class AutomationUndisturbedTest(unittest.TestCase):
    """While Collect or Moves pick up and release, the arm or the ball sits in front of the sensor. The obstacle guard is for
    driving by hand only: it must leave their arm moves, drives and turns exactly as they are."""

    def setUp(self):
        HandApproachTest.make(self, 3.0)                             # "3 cm ahead": the arm or the ball
        self.sent = []
        app = self.app
        app._write = self.sent.append
        app.drive_failed, app.drive_wait_seq, app.encoder_seq, app.runner_ok = False, None, 0, True
        app._path_ready = lambda: True
        app.drive_encoder = lambda *a, **k: dashboard.Dashboard.drive_encoder(app, *a, **k)
        app.turn_by = lambda deg: dashboard.Dashboard.turn_by(app, deg)
        self.car_for_moves = dashboard.DashboardCar(app)             # what both Collect and Moves drive the car with

    def tick(self, seconds):
        for _ in range(int(round(seconds / 0.05))):
            self.car.now = round(self.car.now + 0.05, 2)
            self.app.dist_cm, self.app.dist_stamp = 3.0, self.car.now
            dashboard.Dashboard._obstacle_guard(self.app, self.car.now)
            if self.app.runner.active:
                self.app.runner.tick(self.car.now, 0.0)
            self.app.runner.settle_obstacle(self.car.now)

    def test_a_drive_towards_the_ball_or_away_after_releasing_it_goes_through(self):
        for cm in (5, -5):
            self.sent.clear()
            self.assertTrue(self.car_for_moves.start_drive(cm, 100, 3))
            self.tick(2.0)
            self.assertEqual(self.sent, [f"M{cm} 100 3"])             # no stop, no speed change, nothing added
            self.assertFalse(self.app.runner.active)
        self.assertEqual(self.app.tracker.obstacles, [])

    def test_a_turn_goes_through(self):
        self.assertTrue(self.car_for_moves.start_turn(90))
        self.tick(1.0)
        self.assertTrue(self.app.runner.active)
        self.assertEqual([m[0] for m in self.sent], ["S", "Y"])       # the turn's own commands only
        self.assertNotIn("obstacle", self.app.runner.status)

    def test_arm_moves_are_never_held_back(self):
        for command in ("V1 135 60", "H2 40", "V3 90 30"):
            self.app._send(command)
        self.tick(0.5)
        self.assertEqual(self.sent, ["V1 135 60", "H2 40", "V3 90 30"])



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


class MapWallTest(unittest.TestCase):
    def setUp(self):
        if ROOT is None:
            self.skipTest("no display for Tk")
        self.tracker = dashboard.PathTracker()
        app = types.SimpleNamespace(root=ROOT, tracker=self.tracker, path_settings=PathSettings(), path_window=None,
                                    runner=types.SimpleNamespace(status="Idle", active=False))
        for name in ("path_return", "path_abort", "path_clear", "path_test_drive"):
            setattr(app, name, lambda: None)
        app.path_run_plan = app.apply_path_settings = lambda *a, **k: None
        app._button = dashboard.Dashboard._button.__get__(app)
        logs = tempfile.TemporaryDirectory()
        self.addCleanup(logs.cleanup)
        self.window = dashboard.PathWindow(app, touch_log=Path(logs.name) / "touchpad_log.txt")
        self.addCleanup(self.window.close)
        self.window.win.geometry("660x760+-4000+-4000")
        self.window.win.update()

    def walls(self):
        c = self.window.canvas
        return [item for item in c.find_withtag("obstacle") if c.type(item) == "polygon"]

    def test_a_red_wall_is_drawn_across_the_way(self):
        self.window.refresh(force=True)
        self.assertEqual(self.walls(), [])
        self.tracker.add_obstacle(20.0)
        self.window.refresh(force=True)
        wall, = self.walls()
        self.assertEqual(self.window.canvas.itemcget(wall, "fill"), dashboard.BAD)
        xs, ys = self.window.canvas.coords(wall)[::2], self.window.canvas.coords(wall)[1::2]
        scale = self.window.tf[2]
        self.assertAlmostEqual((max(xs) - min(xs)) / scale, dashboard.WALL_WIDE_CM, delta=0.2)    # across the way ...
        self.assertLess(max(ys) - min(ys), (max(xs) - min(xs)) / 2)                              # ... and thin along it
        cx, cy, scale, w, h = self.window.tf
        near_edge = max(ys)                                           # the edge toward the car is where the sensor saw it
        self.assertAlmostEqual(cy - (near_edge - h / 2) / scale, AHEAD + 20.0, delta=0.5)
        self.assertIn("Obstacles seen: 1", self.window.pos_var.get())

    def car(self, kind):
        c = self.window.canvas
        items = [item for item in c.find_withtag(kind)]
        self.assertTrue(items, kind)
        return c.bbox(items[0]) if kind != "car" else c.coords(items[0])

    def size(self):
        xs, ys = self.car("car")[::2], self.car("car")[1::2]
        scale = self.window.tf[2]
        return (max(xs) - min(xs)) / scale, (max(ys) - min(ys)) / scale

    def test_the_car_is_drawn_to_scale_34_by_16_cm(self):
        self.assertEqual((dashboard.CAR_LENGTH_CM, dashboard.CAR_WIDTH_CM), (34.0, 16.0))
        self.assertEqual(dashboard.ARM_BEYOND_SENSOR_CM, 3.0)
        self.assertEqual(AHEAD, 34.0 / 2 - 3.0)                       # the sensor, 3 cm behind the arm's tip
        self.window.refresh(force=True)
        wide, long = self.size()
        self.assertAlmostEqual(wide, 16.0, delta=0.3)
        self.assertAlmostEqual(long, 34.0, delta=0.3)
        self.tracker.yaw = self.tracker.yaw_ref + 90.0                # turned: the long side lies across the screen
        self.window.refresh(force=True)
        wide, long = self.size()
        self.assertAlmostEqual(wide, 34.0, delta=0.3)
        self.assertAlmostEqual(long, 16.0, delta=0.3)

    def test_the_nose_shows_which_way_it_faces(self):
        self.tracker.yaw = self.tracker.yaw_ref + 90.0                # facing left
        self.window.refresh(force=True)
        nose = self.window.canvas.coords("car_nose")
        body = self.car("car")
        self.assertAlmostEqual(min(nose[::2]), min(body[::2]), delta=1.5)          # its tip reaches the front edge
        self.assertGreater(sum(body[::2]) / 4, sum(nose[::2]) / (len(nose) // 2))

    def test_the_sensor_is_marked_3_cm_behind_the_front(self):
        self.window.refresh(force=True)
        line = self.window.canvas.coords("car_sensor")
        self.assertAlmostEqual((line[1] - min(self.car("car")[1::2])) / self.window.tf[2], 3.0, delta=0.3)
        self.assertEqual(self.window.canvas.itemcget("car_sensor", "fill"), dashboard.BAD)

    def test_the_gap_between_the_cars_sensor_and_the_wall_is_the_distance(self):
        self.tracker.add_obstacle(20.0)
        self.window.refresh(force=True)
        wall, = self.walls()
        sensor = self.window.canvas.coords("car_sensor")[1]
        near = max(self.window.canvas.coords(wall)[1::2])
        self.assertAlmostEqual((sensor - near) / self.window.tf[2], 20.0, delta=0.5)

    def test_a_far_wall_is_kept_in_view(self):
        self.tracker.add_obstacle(250.0)
        self.window.refresh(force=True)
        wall, = self.walls()
        h = self.window.tf[4]
        self.assertTrue(all(0 <= y <= h for y in self.window.canvas.coords(wall)[1::2]))


if __name__ == "__main__":
    unittest.main()
