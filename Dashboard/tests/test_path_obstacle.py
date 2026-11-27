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
