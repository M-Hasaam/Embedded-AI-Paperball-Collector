import sys
import tempfile
import tkinter as tk
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.floor import Floor, dot_offset, load_zero, save_zero, tilt_words  # noqa: E402

FLAT = (0.0, 0.0, 1.0)


class WordsTest(unittest.TestCase):
    def test_tilt_in_words(self):
        self.assertEqual(tilt_words(0.4, -0.6), "level")
        self.assertEqual(tilt_words(0.2, 8.3), "8° right")
        self.assertEqual(tilt_words(0.0, -8.0), "8° left")
        self.assertEqual(tilt_words(5.0, 0.0), "5° front up")
        self.assertEqual(tilt_words(-5.0, 8.0), "5° front down · 8° right")
        self.assertEqual(tilt_words(1.5, 1.5, flat=2.0), "level")

    def test_the_dot_shows_which_way_the_car_leans(self):
        self.assertEqual(dot_offset(0.0, 0.0, 20), (0.0, 0.0))
        x, y = dot_offset(0.0, 7.5, 20)                              # leaning right: to the right
        self.assertEqual((round(x, 3), y), (10.0, 0.0))
        x, y = dot_offset(7.5, 0.0, 20)                              # front up, so it leans back: downwards
        self.assertEqual((x, round(y, 3)), (0.0, 10.0))
        x, y = dot_offset(40.0, 40.0, 20)                            # far over: it stays inside the circle
        self.assertAlmostEqual((x * x + y * y) ** 0.5, 20.0)


class FloorTest(unittest.TestCase):
    def feed(self, floor, pitch=0.0, roll=0.0, accel=FLAT, turning=0.0, wheels=0.0, n=1):
        for _ in range(n):
            state = floor.update(pitch, roll, accel, turning, wheels)
        return state

    def test_standing_still_it_judges_the_floor(self):
        state = self.feed(Floor(), roll=8.0, accel=(0.0, 0.14, 0.99))
        self.assertTrue(state.still)
        self.assertEqual((state.pitch, state.roll), (0.0, 8.0))
        self.assertEqual(state.words, "8° right")
        self.assertAlmostEqual(state.total, (0.14 ** 2 + 0.99 ** 2) ** 0.5, places=3)
        self.assertFalse(state.suspect)

    def test_the_acceleration_is_smoothed(self):
        floor = Floor()
        self.feed(floor)
        state = self.feed(floor, accel=(0.5, 0.0, 1.0))
        self.assertTrue(0.0 < state.accel[0] < 0.2, state.accel)
        state = self.feed(floor, accel=(0.5, 0.0, 1.0), n=60)
        self.assertAlmostEqual(state.accel[0], 0.5, places=2)

    def test_the_dot_keeps_following_a_car_that_is_moving(self):
        # recorded on the car: tipping it by hand also turns it a little, and the panel stood still half the time, up to 3 s
        # long, while the car's tilt got 49 degrees away from what it showed
        for moving in (dict(turning=15.0), dict(wheels=2.8)):
            floor = Floor()
            self.feed(floor, roll=3.0)
            state = self.feed(floor, roll=20.0, **moving)
            self.assertFalse(state.still, moving)                     # not a judgement of the floor ...
            self.assertEqual((state.pitch, state.roll), (0.0, 20.0))  # ... but it shows where the car is now
            self.assertEqual(state.words, "20° right")
            self.assertEqual(self.feed(floor, roll=0.5, **moving).words, "level")

    def test_a_total_far_from_one_g_while_standing_points_at_the_sensor(self):
        floor = Floor()
        state = self.feed(floor, accel=(0.0, 0.0, 1.4), n=5)
        self.assertTrue(state.suspect)
        state = self.feed(Floor(), accel=(0.0, 0.0, 1.4), wheels=20.0)
        self.assertFalse(state.suspect)                              # while driving that is just a bump

    def test_set_as_flat_takes_the_sensors_own_slant_out(self):
        floor = Floor()
        self.assertFalse(floor.set_flat())                           # nothing read yet
        self.feed(floor, pitch=5.0, roll=-1.0)
        self.assertTrue(floor.set_flat())
        self.assertEqual(floor.zero, (5.0, -1.0))
        state = self.feed(floor, pitch=5.0, roll=7.0)
        self.assertEqual((state.pitch, state.roll, state.words), (0.0, 8.0, "8° right"))
        self.assertEqual(self.feed(floor, pitch=5.5, roll=-0.5).words, "flat")

    def test_set_as_flat_needs_a_still_car(self):
        floor = Floor()
        self.feed(floor, pitch=5.0, turning=20.0)
        self.assertFalse(floor.set_flat())
        self.assertEqual(floor.zero, (0.0, 0.0))

    def test_reset_forgets_the_readings_but_not_the_flat_setting(self):
        floor = Floor((5.0, -1.0))
        self.feed(floor, roll=3.0)
        floor.reset()
        self.assertFalse(floor.set_flat())
        self.assertEqual(floor.zero, (5.0, -1.0))

    def test_the_flat_setting_is_saved(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "floor_level.json"
            self.assertEqual(load_zero(path), (0.0, 0.0))            # never set
            save_zero(path, (5.0, -1.25))
            self.assertEqual(load_zero(path), (5.0, -1.25))
            for bad in ("", "not json", "[1]", '{"pitch": "x", "roll": 1}'):
                path.write_text(bad)
                self.assertEqual(load_zero(path), (0.0, 0.0), bad)


# ---- the dashboard's side of it ----
import dashboard  # noqa: E402

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


class LevelPanelTest(unittest.TestCase):
    def setUp(self):
        if ROOT is None:
            self.skipTest("no display for Tk")
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "floor_level.json"
        self.floor = Floor()
        self.app = types.SimpleNamespace(_button=lambda parent, text, command, **kw: tk.Button(parent, text=text, command=command))
        self.panel = dashboard.LevelPanel(ROOT, self.app, self.floor, self.path)
        self.addCleanup(self.panel.destroy)

    def dot_centre(self):
        x1, y1, x2, y2 = self.panel.canvas.coords(self.panel.dot)
        return (x1 + x2) / 2, (y1 + y2) / 2

    def test_it_shows_the_floor_and_moves_the_dot(self):
        middle = self.dot_centre()
        self.panel.show(self.floor.update(0.0, 8.0, (0.0, 0.14, 0.99), 0.0, 0.0))
        self.assertEqual(self.panel.words.get(), "8° right")
        self.assertEqual(self.panel.total.get(), "floor · 1.00 g")
        self.assertGreater(self.dot_centre()[0], middle[0])
        moved = self.dot_centre()
        self.panel.show(self.floor.update(0.0, -8.0, (0.0, -0.14, 0.99), 30.0, 0.0))
        self.assertLess(self.dot_centre()[0], middle[0], moved)         # follows while moving
        self.assertEqual(self.panel.words.get(), "8° left")
        self.assertRegex(self.panel.total.get(), r"^moving · [01]\.\d\d g")      # the total is smoothed
        self.panel.show(None)
        self.assertEqual((self.panel.words.get(), self.panel.total.get()), ("--", ""))
        self.assertEqual(self.dot_centre(), middle)

    def test_set_as_flat_saves_and_a_moving_car_is_refused(self):
        self.floor.update(5.0, 8.0, FLAT, 30.0, 0.0)
        self.panel.set_flat()
        self.assertIn("keep still", self.panel.words.get())
        self.assertFalse(self.path.exists())
        self.floor.update(5.0, 8.0, FLAT, 0.0, 0.0)
        self.panel.set_flat()
        self.assertEqual(load_zero(self.path), (5.0, 8.0))
        self.assertEqual(self.panel.words.get(), "flat")


class ShowMotionTest(unittest.TestCase):
    def test_back_up_is_shown_as_front_down(self):
        # seen on the car with its back raised: the car sends pitch +11 (accelerometer x -0.17) and the card said "front up"
        seen = []
        app = types.SimpleNamespace(posted=[])
        app._post = app.posted.append
        app._show_mpu = lambda yaw, pitch, roll, gz, stamp, **kw: seen.append((pitch, roll))
        dashboard.Dashboard._on_mpu(app, None, b"0.0,11.0,2.0,0.0,0.0,0.0,-0.17,0.02,0.92")
        app.posted[0]()
        self.assertEqual(tilt_words(*seen[0]), "11° front down · 2° right")

    def test_tipping_the_car_counts_as_moving_too(self):
        asked, shown = [], []
        text = types.SimpleNamespace(set=lambda value: None)
        floor = types.SimpleNamespace(update=lambda *a: asked.append(a) or Floor().update(*a))
        app = types.SimpleNamespace(tracker=types.SimpleNamespace(feed_yaw=lambda *a, **k: None), _yaw_resync=False,
                                    heading_var=text, spin_var=text, tilt_var=text, accel_var=text, floor=floor, wheel_speed=0.0,
                                    level=types.SimpleNamespace(show=shown.append), _draw_dial=lambda: None)
        dashboard.Dashboard._show_mpu(app, 0.0, 1.0, 2.0, 0.5, accel=FLAT, rates=(-40.0, 8.0, 0.5))
        self.assertEqual(asked[0][3], 40.0)                           # the fastest of the three gyro axes
        self.assertFalse(shown[0].still)

    def test_the_acceleration_reaches_the_card(self):
        seen = []
        app = types.SimpleNamespace(posted=[])
        app._post = app.posted.append
        app._show_mpu = lambda *args, **kw: seen.append((args, kw))
        dashboard.Dashboard._on_mpu(app, None, b"-1.0,2.0,3.0,0.1,0.2,0.3,-0.05,0.14,0.99")
        app.posted[0]()
        self.assertEqual(seen[0][1]["accel"], (-0.05, 0.14, 0.99))
        self.assertEqual(seen[0][1]["rates"], (0.1, 0.2, 0.3))


if __name__ == "__main__":
    unittest.main()
