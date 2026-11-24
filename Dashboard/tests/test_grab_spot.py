import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.grab_spot import GrabSpot, GrabSpotError, load_spot, save_spot  # noqa: E402


class GrabSpotTest(unittest.TestCase):
    def test_pick_range_is_two_cm_either_side(self):
        spot = GrabSpot(0.5, 16.0)
        self.assertEqual((spot.pick_min, spot.pick_max), (14.0, 18.0))

    def test_save_and_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            for spot in (GrabSpot(0.4, 16.0), GrabSpot(0.62, 20.5, 19.0)):
                path = Path(tmp) / "s.json"
                save_spot(path, spot)
                self.assertEqual(load_spot(path), spot)

    def test_missing_file_gives_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIsNone(load_spot(Path(tmp) / "none.json"))

    def test_bad_files_raise(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i, text in enumerate(("garbage", '{"x_ratio": 1.5, "ultrasonic_cm": 16}', '{"x_ratio": 0.5, "ultrasonic_cm": 2}',
                                      "[1]", '{"x_ratio": "a", "ultrasonic_cm": 16}')):
                path = Path(tmp) / f"{i}.json"
                path.write_text(text)
                with self.assertRaises(GrabSpotError, msg=text):
                    load_spot(path)


if __name__ == "__main__":
    unittest.main()
