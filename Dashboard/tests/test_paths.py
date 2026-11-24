import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers import paths  # noqa: E402

DASHBOARD = Path(__file__).resolve().parents[1]


class PathsTest(unittest.TestCase):
    def test_the_folders_are_where_the_layout_says(self):
        self.assertEqual(paths.DASHBOARD_DIR, DASHBOARD)
        self.assertEqual(paths.DATA_DIR, DASHBOARD / "data")             # everything the program writes
        self.assertEqual(paths.MOVES_DIR, DASHBOARD / "moves")           # the recorded moves
        self.assertEqual(paths.MODEL_PATH, DASHBOARD.parent / "Training" / "paper_ball_detector" / "roboflow-paperball.onnx")

    def test_data_file_makes_the_folder_and_returns_a_path_inside_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(paths, "DATA_DIR", Path(tmp) / "data"):
                path = paths.data_file("servo_last_angles.json")
                self.assertTrue((Path(tmp) / "data").is_dir())
                self.assertEqual(path, Path(tmp) / "data" / "servo_last_angles.json")
                self.assertEqual(paths.data_file("x.txt"), Path(tmp) / "data" / "x.txt")      # a second call is fine


class LayoutTest(unittest.TestCase):
    def test_the_helper_modules_are_not_loose_in_embedded(self):
        loose = sorted(p.name for p in DASHBOARD.glob("*.py"))
        self.assertEqual(loose, ["dashboard.py"])                       # dashboard.py is the only program file at this level

    def test_the_helpers_folder_is_a_package_with_the_modules(self):
        names = {p.name for p in (DASHBOARD / "helpers").glob("*.py")}
        self.assertTrue({"__init__.py", "paths.py", "moves.py", "ball_detector.py"} <= names)

    def test_generated_files_are_not_loose_in_embedded(self):
        for name in ("ai_car_address.txt", "path_runner_log.txt", "servo_last_angles.json"):
            self.assertFalse((DASHBOARD / name).exists(), f"{name} should be in data/")


if __name__ == "__main__":
    unittest.main()
