import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.sensor_watch import SensorWatch  # noqa: E402

LINE = "0.0,0.1,-0.2,0.0,0.0,0.1,-0.04,0.01,0.94"


class SensorWatchTest(unittest.TestCase):
    def test_changing_readings_are_a_live_sensor(self):
        watch = SensorWatch()
        for n in range(200):
            watch.reading(f"{LINE[:-1]}{n % 3}", n * 0.04)
        self.assertEqual(watch.check(199 * 0.04), (None, 0.0))

    def test_a_still_car_that_repeats_a_reading_for_a_moment_is_live(self):
        watch = SensorWatch()
        for n in range(20):                                           # 0.8 s of the same line
            watch.reading(LINE, n * 0.04)
        self.assertIsNone(watch.check(0.8)[0])

    def test_the_same_reading_for_two_seconds_means_the_sensor_is_stuck(self):
        watch = SensorWatch()
        for n in range(61):
            watch.reading(LINE, n * 0.04)
        state, seconds = watch.check(2.4)
        self.assertEqual(state, "frozen")
        self.assertAlmostEqual(seconds, 2.4)
        watch.reading("1.0,0.1,-0.2,0.0,0.0,0.1,-0.04,0.01,0.94", 2.5)
        self.assertIsNone(watch.check(2.5)[0])                        # it is back

    def test_no_readings_at_all_for_a_while_is_silence(self):
        watch = SensorWatch()
        self.assertIsNone(watch.check(100.0)[0])                      # nothing ever read: nothing to say
        watch.reading(LINE, 10.0)
        self.assertIsNone(watch.check(11.0)[0])
        self.assertEqual(watch.check(12.0), ("silent", 2.0))
        watch.reset()
        self.assertIsNone(watch.check(20.0)[0])


class DashboardWatchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = Path(self.tmp.name) / "sensor_log.txt"
        text = types.SimpleNamespace(value="Turning speed: stopped")
        text.set = lambda value: setattr(text, "value", value)
        self.shown = []
        label = types.SimpleNamespace(configure=lambda **kw: self.shown.append(kw.get("fg")))
        level = types.SimpleNamespace(show=lambda state: self.shown.append(("level", state)))
        self.app = types.SimpleNamespace(mpu_watch=SensorWatch(), mpu_state=None, sensor_log=self.log, spin_var=text,
                                         spin_label=label, level=level, connected=types.SimpleNamespace(is_set=lambda: True), wheel_cm=None, wheel_speed=0.0,
                                         mpu_log=types.SimpleNamespace(write=lambda *a: None, note=lambda *a: None))

    def note(self, text, now):
        dashboard.Dashboard._note_mpu(self.app, text, now)

    def tick(self, now):
        dashboard.Dashboard._mpu_tick(self.app, now)

    def logged(self):
        return self.log.read_text(encoding="utf-8") if self.log.exists() else ""

    def test_a_stuck_sensor_is_said_on_the_card_and_written_down_once(self):
        for n in range(80):
            self.note(LINE, n * 0.04)
            self.tick(n * 0.04)
        self.assertIn("Motion sensor stuck", self.app.spin_var.value)
        self.assertIn(("level", None), self.shown)
        self.assertEqual(self.logged().count("motion sensor frozen"), 1)
        self.assertIn(LINE, self.logged())
        self.note("5.0,0.1,-0.2,0.0,0.0,0.1,-0.04,0.01,0.94", 4.0)
        self.tick(4.0)
        self.assertIn("motion sensor back after", self.logged())
        self.assertEqual(self.shown[-1], dashboard.MUTED)             # the warning colour is taken off again

    def test_silence_is_said_only_while_connected(self):
        self.note(LINE, 0.0)
        self.app.connected = types.SimpleNamespace(is_set=lambda: False)
        self.tick(5.0)
        self.assertEqual(self.logged(), "")
        self.app.connected = types.SimpleNamespace(is_set=lambda: True)
        self.tick(5.0)
        self.assertIn("Motion sensor: no readings", self.app.spin_var.value)
        self.assertIn("motion sensor silent", self.logged())

    def test_every_motion_line_from_the_car_is_noted(self):
        posted, noted = [], []
        app = types.SimpleNamespace(_post=posted.append, _show_mpu=lambda *a, **k: None,
                                    _note_mpu=lambda text, stamp: noted.append(text))
        dashboard.Dashboard._on_mpu(app, None, LINE.encode())
        for job in posted:
            job()
        self.assertEqual(noted, [LINE])


if __name__ == "__main__":
    unittest.main()
