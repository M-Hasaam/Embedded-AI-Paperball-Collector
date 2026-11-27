import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.path_settings import Odometer, PathSettings, load_settings, save_settings, wheels_disagree  # noqa: E402


class SettingsTest(unittest.TestCase):
    def test_the_defaults(self):
        s = PathSettings()
        self.assertEqual((s.slow_within_cm, s.fast_pct, s.slow_pct, s.trust, s.differ_cm, s.differ_pct, s.assist),
                         (20.0, 100, 50, "right", 4.0, 15.0, "gyro"))
        self.assertEqual((s.arrive_cm, s.turn_within_deg, s.turn_extra_deg), (3.0, 2.0, 1.0))
        self.assertEqual((s.stop_closed_cm, s.stop_default_cm, s.stop_other_cm), (5.0, 10.0, 20.0))
        self.assertEqual([s.stop_cm(p) for p in ("closed", "default", "other")], [5.0, 10.0, 20.0])
        self.assertEqual(s.changed(stop_closed_cm=0).stop_closed_cm, 3.0)       # the sensor cannot see closer

    def test_the_commands_for_the_car(self):
        self.assertEqual(PathSettings().wheel_command(), "W1 4 15")
        self.assertEqual(PathSettings().changed(trust="left", differ_cm=6, differ_pct=20).wheel_command(), "W2 6 20")
        self.assertEqual(PathSettings().changed(trust="lower").wheel_command(), "W0 4 15")
        self.assertEqual([PathSettings().changed(assist=a).assist_mode() for a in ("off", "gyro", "encoders", "both")], [0, 1, 2, 3])

    def test_values_are_kept_within_what_works(self):
        s = PathSettings().changed(slow_within_cm=-5, fast_pct=73, slow_pct="100", arrive_cm=0, turn_within_deg=99,
                                   turn_extra_deg=9, differ_cm=0, differ_pct=500, trust="middle", assist="x")
        self.assertEqual((s.slow_within_cm, s.arrive_cm, s.turn_within_deg, s.turn_extra_deg), (0.0, 1.0, 10.0, 3.0))
        self.assertIn(s.fast_pct, (50, 100))                          # only the speeds where the encoders are accurate
        self.assertEqual((s.slow_pct, s.differ_cm, s.differ_pct, s.trust, s.assist), (100, 1.0, 100.0, "right", "gyro"))

    def test_nonsense_leaves_the_old_value(self):
        s = PathSettings().changed(slow_within_cm="abc", differ_cm="", arrive_cm=float("nan"))
        self.assertEqual((s.slow_within_cm, s.differ_cm, s.arrive_cm), (20.0, 4.0, 3.0))
        self.assertEqual(PathSettings().changed(slow_within_cm="12,5").slow_within_cm, 12.5)

    def test_they_are_saved_and_loaded(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "path_settings.json"
            self.assertEqual(load_settings(path), PathSettings())
            chosen = PathSettings().changed(slow_within_cm=35, trust="left", fast_pct=50)
            save_settings(path, chosen)
            self.assertEqual(load_settings(path), chosen)
            for bad in ("", "[1]", "nope", '{"slow_within_cm": "x", "unknown": 1}'):
                path.write_text(bad)
                self.assertEqual(load_settings(path), PathSettings(), bad)


class WheelRuleTest(unittest.TestCase):
    def test_the_same_rule_as_the_cars(self):                         # the cases of tests/drive_wheels_test.cpp
        s = PathSettings()
        self.assertTrue(wheels_disagree(65.4, 9.8, s))
        self.assertFalse(wheels_disagree(20.0, 22.0, s))
        self.assertFalse(wheels_disagree(100.0, 91.0, s))
        self.assertTrue(wheels_disagree(100.0, 80.0, s))
        self.assertTrue(wheels_disagree(9.0, 4.0, s))
        self.assertFalse(wheels_disagree(7.0, 4.0, s))
        self.assertTrue(wheels_disagree(-20.0, 5.0, s))


class OdometerTest(unittest.TestCase):
    def drive(self, readings, straight=True, settings=PathSettings(), start=(100.0, 200.0)):
        odo = Odometer()
        odo.feed(*start, False, settings)
        total = 0.0
        for left, right in readings:
            total = odo.feed(start[0] + left, start[1] + right, straight, settings)
        return total

    def test_agreeing_wheels_give_their_mean(self):
        self.assertAlmostEqual(self.drive([(5, 5), (10, 11), (20, 21)]), 20.5)

    def test_a_left_counter_that_runs_away_is_ignored(self):
        self.assertAlmostEqual(self.drive([(5, 5), (30, 8), (65.4, 9.8)]), 9.8)
        self.assertAlmostEqual(self.drive([(65.4, 9.8)], settings=PathSettings().changed(trust="left")), 65.4)
        self.assertAlmostEqual(self.drive([(25, -1700)], settings=PathSettings().changed(trust="lower")), 25)   # the smaller count
        self.assertAlmostEqual(self.drive([(25, -1700)], settings=PathSettings().changed(trust="left")), 25)

    def test_backwards(self):
        self.assertAlmostEqual(self.drive([(-5, -5), (-40, -10.2)]), -10.2)

    def test_while_turning_the_mean_counts_so_a_turn_does_not_move_the_car(self):
        self.assertAlmostEqual(self.drive([(-8, 8), (-15, 15)], straight=False), 0.0)

    def test_one_straight_drive_after_another(self):
        odo, s = Odometer(), PathSettings()
        odo.feed(0, 0, False, s)
        odo.feed(10, 10, True, s)
        odo.feed(10, 10, False, s)                                    # stopped
        odo.feed(2, 18, False, s)                                     # turned in place
        self.assertAlmostEqual(odo.feed(70, 28, True, s), 20.0)       # second drive: 10 on the right, 68 on the left

    def test_a_reset_of_the_counters_is_not_a_drive(self):
        odo, s = Odometer(), PathSettings()
        odo.feed(5000, 5000, False, s)
        self.assertEqual(odo.feed(0, 0, True, s), 0.0)
        self.assertAlmostEqual(odo.feed(5, 5, True, s), 5.0)
        odo.forget_counters()
        self.assertAlmostEqual(odo.feed(900, 900, True, s), 5.0)      # a new starting point


if __name__ == "__main__":
    unittest.main()
