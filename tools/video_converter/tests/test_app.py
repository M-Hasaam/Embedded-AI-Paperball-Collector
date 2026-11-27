import subprocess
import sys
import tempfile
import time
import tkinter as tk
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import converter  # noqa: E402
from video_converter import ConverterApp  # noqa: E402


class AppTest(unittest.TestCase):
    """The real window (kept hidden) running the real ffmpeg on tiny generated clips."""

    @classmethod
    def setUpClass(cls):
        cls.ffmpeg = converter.find_ffmpeg()

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.folder = Path(tmp.name)
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = ConverterApp(self.root, self.ffmpeg)
        self.addCleanup(self.close)

    def close(self):
        self.app.stop_and_wait()
        self.root.destroy()

    def clip(self, name, seconds=1):
        path = self.folder / name
        subprocess.run([self.ffmpeg, "-v", "error", "-f", "lavfi",
                        "-i", f"testsrc2=size=1920x1080:rate=30:duration={seconds}",
                        "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p", str(path)], check=True)
        return path

    def rows(self):
        return [tuple(str(v) for v in self.app.tree.item(item, "values")) for item in self.app.tree.get_children()]

    def column(self, index):
        return [row[index] for row in self.rows()]

    def state(self, button):
        return str(button.cget("state"))

    def run_to_the_end(self):
        deadline = time.time() + 120
        while self.app.busy:
            self.assertLess(time.time(), deadline, "the conversion never finished")
            self.root.update()
            time.sleep(0.02)

    def test_added_videos_are_listed_once(self):
        self.assertEqual(self.state(self.app.convert_button), "disabled")
        path = self.clip("a.mov")
        self.app.add([str(path)])
        self.app.add([str(path)])
        self.assertEqual(self.rows(), [("a.mov", converter.size_text(path.stat().st_size), "", "waiting")])
        self.assertEqual(self.state(self.app.convert_button), "normal")

    def test_a_broken_file_is_marked_at_once_and_cannot_be_converted(self):
        broken = self.folder / "broken.MOV"
        broken.write_bytes(b"\x00" * 203)
        self.app.add([str(broken)])
        self.assertEqual(self.rows(), [("broken.MOV", "203 bytes", "", "not a playable video")])
        self.assertEqual(self.state(self.app.convert_button), "disabled")

    def test_long_texts_do_not_widen_the_window(self):
        self.root.update_idletasks()
        before = self.root.winfo_reqwidth()
        long = "C:/Users/someone/a very long folder name/" * 6
        self.app.set_folder(long)
        self.app.status_var.set(f"2 videos converted. Saved in {long}")
        self.root.update_idletasks()
        self.assertEqual(self.root.winfo_reqwidth(), before)

    def test_selected_videos_can_be_removed(self):
        self.app.add([str(self.clip("a.mov")), str(self.clip("b.mov"))])
        self.app.tree.selection_set(self.app.tree.get_children()[0])
        self.app.remove_button.invoke()
        self.assertEqual(self.column(0), ["b.mov"])

    def test_all_waiting_videos_are_converted_with_the_chosen_options(self):
        out = self.folder / "out"
        out.mkdir()
        self.app.add([str(self.clip("a.mov")), str(self.clip("b.mov"))])
        self.app.size_var.set("720p")
        self.app.set_folder(out)
        self.app.convert_button.invoke()
        self.assertEqual([self.state(b) for b in (self.app.add_button, self.app.remove_button,
                                                  self.app.convert_button, self.app.cancel_button)],
                         ["disabled", "disabled", "disabled", "normal"])
        self.run_to_the_end()
        self.assertEqual(sorted(p.name for p in out.iterdir()), ["a.mp4", "b.mp4"])
        self.assertEqual(converter.probe(self.ffmpeg, out / "a.mp4").height, 720)
        self.assertEqual(self.column(2), [converter.size_text((out / n).stat().st_size) for n in ("a.mp4", "b.mp4")])
        for status in self.column(3):
            self.assertRegex(status, r"^done, \d+% (smaller|bigger)$")
        self.assertEqual(self.app.status_var.get(), f"2 videos converted. Saved in {out}")
        self.assertEqual([self.state(b) for b in (self.app.add_button, self.app.convert_button,
                                                  self.app.cancel_button)], ["normal", "disabled", "disabled"])

    def test_cancel_stops_the_whole_list(self):
        self.app.add([str(self.clip("a.mov", seconds=5)), str(self.clip("b.mov"))])
        self.app.convert_button.invoke()
        self.app.cancel_button.invoke()
        self.run_to_the_end()
        self.assertEqual(self.column(3), ["cancelled", "waiting"])
        self.assertEqual(sorted(p.name for p in self.folder.iterdir()), ["a.mov", "b.mov"])
        self.assertEqual(self.app.status_var.get(), "Cancelled.")
        self.assertEqual(self.state(self.app.convert_button), "normal")

    def test_one_failure_does_not_stop_the_others(self):
        gone = self.clip("gone.mov")
        self.app.add([str(gone), str(self.clip("b.mov"))])
        gone.unlink()
        self.app.convert_button.invoke()
        self.run_to_the_end()
        self.assertTrue(self.column(3)[0].startswith("failed: "), self.column(3)[0])
        self.assertTrue(self.column(3)[1].startswith("done"), self.column(3)[1])
        self.assertIn("1 failed", self.app.status_var.get())


if __name__ == "__main__":
    unittest.main()
