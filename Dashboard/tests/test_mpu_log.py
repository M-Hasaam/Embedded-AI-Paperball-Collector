import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.mpu_log import COLUMNS, MpuLog  # noqa: E402

LINE = "1.0,0.1,-0.2,0.0,0.0,0.1,-0.04,0.01,0.94"


class MpuLogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "mpu_log.csv"

    def made(self, **kw):
        log = MpuLog(self.path, **kw)
        self.addCleanup(log.close)
        return log

    def lines(self):
        return self.path.read_text(encoding="utf-8").splitlines()

    def test_each_reading_is_a_row_with_the_time_the_car_line_and_the_wheels(self):
        log = self.made()
        log.write(100.0, LINE, (10.0, 11.5), 3.25)
        log.write(100.04, LINE, None, 0.0)
        log.flush()
        rows = [line for line in self.lines() if not line.startswith("#")]
        self.assertEqual(rows[0], COLUMNS)
        first, second = rows[1].split(","), rows[2].split(",")
        self.assertEqual(len(first), len(COLUMNS.split(",")))
        self.assertRegex(first[0], r"^\d\d:\d\d:\d\d\.\d{3}$")
        self.assertEqual((first[1], second[1]), ("0.000", "0.040"))            # seconds since the log began
        self.assertEqual(",".join(first[2:11]), LINE)
        self.assertEqual(first[11:], ["10.0", "11.5", "3.2"])
        self.assertEqual(second[11:], ["", "", "0.0"])

    def test_a_line_that_is_not_nine_numbers_is_kept_as_a_note(self):
        log = self.made()
        log.write(1.0, "NA", None, 0.0)
        log.note(2.0, "sent C")
        log.flush()
        notes = [line for line in self.lines() if line.startswith("#")]
        self.assertTrue(any("car says: NA" in line for line in notes), notes)
        self.assertTrue(notes[-1].endswith("sent C"), notes)

    def test_a_new_session_is_added_after_the_old_one(self):
        first = self.made()
        first.write(1.0, LINE, None, 0.0)
        first.close()
        second = self.made()
        second.write(9.0, LINE, None, 0.0)
        second.flush()
        self.assertEqual(sum(1 for line in self.lines() if "session started" in line), 2)
        self.assertEqual(sum(1 for line in self.lines() if line == COLUMNS), 1)

    def test_it_reaches_the_disk_within_a_second_without_a_flush(self):
        log = self.made()
        log.write(1.0, LINE, None, 0.0)
        log.write(2.1, LINE, None, 0.0)
        self.assertIn(LINE, self.path.read_text(encoding="utf-8"))

    def test_a_full_file_is_put_aside_and_a_new_one_begun(self):
        log = self.made(max_bytes=600)
        for n in range(40):
            log.write(n * 0.04, LINE, None, 0.0)
        log.flush()
        self.assertTrue(self.path.with_suffix(".old.csv").exists())
        self.assertLess(self.path.stat().st_size, 900)
        self.assertIn(COLUMNS, self.lines())

    def test_a_file_that_cannot_be_written_does_no_harm(self):
        log = MpuLog(self.path / "no" / "such.csv")
        log.write(1.0, LINE, None, 0.0)
        log.note(2.0, "x")
        log.flush()
        log.close()


class DashboardMpuLogTest(unittest.TestCase):
    def test_readings_and_commands_reach_the_log(self):
        seen = []
        log = types.SimpleNamespace(write=lambda *a: seen.append(("row",) + a), note=lambda stamp, text: seen.append(("note", text)))
        app = types.SimpleNamespace(mpu_watch=types.SimpleNamespace(reading=lambda text, stamp: None), mpu_log=log,
                                    wheel_cm=(1.0, 2.0), wheel_speed=0.5)
        dashboard.Dashboard._note_mpu(app, LINE, 5.0)
        self.assertEqual(seen, [("row", 5.0, LINE, (1.0, 2.0), 0.5)])
        loop = types.SimpleNamespace(is_running=lambda: False)
        app = types.SimpleNamespace(mpu_log=log, connected=types.SimpleNamespace(is_set=lambda: True), loop=loop)
        seen.clear()
        dashboard.Dashboard._log_command(app, "C")
        dashboard.Dashboard._log_command(app, "V0 90 60")
        dashboard.Dashboard._log_command(app, "M10 50 3")
        self.assertEqual(seen, [("note", "sent C"), ("note", "sent M10 50 3")])      # arm commands do not matter here


if __name__ == "__main__":
    unittest.main()
