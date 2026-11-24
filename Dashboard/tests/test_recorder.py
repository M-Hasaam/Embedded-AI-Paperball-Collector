import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.recorder import RATES, FrameRecorder  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.clock = Clock()

    def make(self, per_second=5, rotate="none"):
        rec = FrameRecorder(self.root, per_second, rotate, clock=self.clock)
        rec.start()
        return rec

    def files(self, rec):
        return sorted(p.name for p in rec.folder.glob("*.jpg"))

    def test_the_camera_s_exact_bytes_are_saved(self):
        rec = self.make(per_second=None)
        self.assertTrue(rec.offer(b"\xff\xd8exact-bytes\xff\xd9"))
        self.assertEqual(next(rec.folder.glob("*.jpg")).read_bytes(), b"\xff\xd8exact-bytes\xff\xd9")

    def test_frames_are_numbered_in_order_with_their_time(self):
        rec = self.make(per_second=None)
        rec.offer(b"a")
        self.clock.t += 1.5
        rec.offer(b"b")
        self.assertEqual(self.files(rec), ["f00000_t0000.00.jpg", "f00001_t0001.50.jpg"])

    def test_the_rate_limit_keeps_about_that_many_per_second(self):
        rec = self.make(per_second=5)                  # at least 0.2 s between saved frames
        saved = []
        for _ in range(11):                            # frames 0.05 s apart for half a second
            saved.append(rec.offer(b"x"))
            self.clock.t += 0.05
        self.assertEqual(saved, [True, False, False, False, True, False, False, False, True, False, False])
        self.assertEqual(rec.frames, 3)

    def test_every_frame_option_saves_them_all(self):
        rec = self.make(per_second=None)
        for _ in range(10):
            self.assertTrue(rec.offer(b"x"))
            self.clock.t += 0.001
        self.assertEqual(rec.frames, 10)

    def test_stop_writes_the_notes_and_later_frames_are_ignored(self):
        rec = self.make(per_second=None, rotate="cw")
        rec.offer(b"a")
        self.clock.t += 2
        rec.stop()
        self.assertFalse(rec.offer(b"b"))
        notes = json.loads((rec.folder / "recording.json").read_text())
        self.assertEqual((notes["frames"], notes["rotate"], notes["seconds"]), (1, "cw", 2.0))
        self.assertFalse(rec.active)

    def test_notes_exist_while_recording_so_a_crash_still_leaves_them(self):
        rec = self.make(rotate="180")
        self.assertEqual(json.loads((rec.folder / "recording.json").read_text())["rotate"], "180")

    def test_two_recordings_in_the_same_second_get_their_own_folders(self):
        with mock.patch("helpers.recorder.time.strftime", return_value="20260101_120000"):
            a, b = self.make(), self.make()
        self.assertNotEqual(a.folder, b.folder)

    def test_a_write_error_stops_the_recording_and_says_why(self):
        rec = self.make(per_second=None)
        with mock.patch.object(Path, "write_bytes", side_effect=OSError("disk full")):
            self.assertFalse(rec.offer(b"a"))
        self.assertIn("disk full", rec.error)
        self.assertFalse(rec.active)

    def test_the_menu_offers_the_rates(self):
        self.assertEqual(RATES["every frame"], None)
        self.assertEqual(RATES["5 per s"], 5)


if __name__ == "__main__":
    unittest.main()
