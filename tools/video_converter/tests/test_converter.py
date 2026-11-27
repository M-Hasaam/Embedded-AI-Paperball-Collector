import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import converter  # noqa: E402
from converter import Cancelled, ConvertError, Options  # noqa: E402

IPHONE = """Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'IMG.MOV':
  Duration: 00:01:44.10, start: 0.000000, bitrate: 71251 kb/s
  Stream #0:0[0x1](und): Video: hevc (Main 10) (hvc1 / 0x31637668), yuv420p10le(tv, bt2020nc/bt2020/arib-std-b67), 3840x2160, 70972 kb/s, 59.97 fps, 60 tbr, 600 tbn (default)
  Stream #0:1[0x2](und): Audio: aac (LC) (mp4a / 0x6134706D), 48000 Hz, stereo, fltp, 177 kb/s (default)
At least one output file must be specified
"""

SILENT_SDR = """Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'screen.mov':
  Duration: 01:02:03.50, start: 0.000000, bitrate: 900 kb/s
  Stream #0:0(und): Video: h264 (High) (avc1 / 0x31637661), yuv420p(tv, bt709, progressive), 1280x720, 800 kb/s, 30 fps (default)
"""


def info(**changes):
    base = dict(duration=10.0, width=3840, height=2160, hdr=False, audio="aac")
    base.update(changes)
    return converter.VideoInfo(**base)


def filters(command):
    return command[command.index("-vf") + 1]


def value(command, flag):
    return command[command.index(flag) + 1]


class ParseTest(unittest.TestCase):
    def test_an_iphone_hdr_video(self):
        self.assertEqual(converter.parse_info(IPHONE), info(duration=104.1, hdr=True))

    def test_a_standard_video_without_sound(self):
        self.assertEqual(converter.parse_info(SILENT_SDR),
                         info(duration=3723.5, width=1280, height=720, audio=None))

    def test_the_other_kind_of_hdr(self):
        self.assertTrue(converter.parse_info(IPHONE.replace("arib-std-b67", "smpte2084")).hdr)

    def test_a_file_that_is_not_a_video(self):
        with self.assertRaises(ConvertError):
            converter.parse_info("broken.MOV: Invalid data found when processing input")


class CommandTest(unittest.TestCase):
    def command(self, video=None, **options):
        return converter.build_command("ffmpeg", Path("in.mov"), Path("out.mp4"), video or info(), Options(**options))

    def test_the_defaults_are_1080p_balanced_without_location(self):
        c = self.command()
        self.assertEqual((value(c, "-c:v"), value(c, "-crf"), value(c, "-map_metadata")), ("libx264", "21", "-1"))
        self.assertIn("min(1080,", filters(c))
        self.assertIn("+faststart", value(c, "-movflags"))
        self.assertIn("-n", c)                                   # ffmpeg itself must never overwrite
        self.assertEqual(c[-1], "out.mp4")

    def test_quality_and_size_choices(self):
        self.assertEqual([value(self.command(quality=q), "-crf") for q in ("Best", "Balanced", "Small")],
                         ["18", "21", "24"])
        self.assertIn("min(720,", filters(self.command(size="720p")))
        self.assertIn("min(2160,", filters(self.command(size="4K")))

    def test_only_hdr_is_colour_converted(self):
        self.assertIn("tonemap", filters(self.command(info(hdr=True))))
        self.assertNotIn("tonemap", filters(self.command(info(hdr=False))))

    def test_aac_sound_is_copied_and_other_sound_is_converted(self):
        self.assertEqual(value(self.command(info(audio="aac")), "-c:a"), "copy")
        self.assertEqual(value(self.command(info(audio="pcm_s16le")), "-c:a"), "aac")
        self.assertNotIn("-c:a", self.command(info(audio=None)))

    def test_location_can_be_kept(self):
        c = self.command(remove_location=False)
        self.assertEqual(value(c, "-map_metadata"), "0")
        self.assertIn("use_metadata_tags", value(c, "-movflags"))


class OutputPathTest(unittest.TestCase):
    def test_existing_files_are_never_chosen(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            source = folder / "IMG.MOV"
            self.assertEqual(converter.output_path(source), folder / "IMG.mp4")
            (folder / "IMG.mp4").write_bytes(b"x")
            self.assertEqual(converter.output_path(source), folder / "IMG (2).mp4")
            (folder / "IMG (2).mp4").write_bytes(b"x")
            self.assertEqual(converter.output_path(source), folder / "IMG (3).mp4")

    def test_another_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(converter.output_path(Path("a/b/IMG.MOV"), Path(tmp)), Path(tmp) / "IMG.mp4")


class TextTest(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual([converter.size_text(n) for n in (203, 78530425, 1137820913)],
                         ["203 bytes", "74.9 MB", "1085.1 MB"])

    def test_time_left(self):
        self.assertEqual(converter.time_left_text(0.0, 5.0), "")
        self.assertEqual(converter.time_left_text(0.5, 20.0), "about 20 s left")
        self.assertEqual(converter.time_left_text(0.25, 60.0), "about 3 min left")


class RealConversionTest(unittest.TestCase):
    """Runs the real ffmpeg on tiny generated clips."""

    @classmethod
    def setUpClass(cls):
        cls.ffmpeg = converter.find_ffmpeg()

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.folder = Path(tmp.name)

    def clip(self, name="clip.mov", size="1920x1440", seconds=1, hdr=False, audio="pcm_s16le"):
        path = self.folder / name
        command = [self.ffmpeg, "-v", "error", "-f", "lavfi", "-i", f"testsrc2=size={size}:rate=30:duration={seconds}",
                   "-f", "lavfi", "-i", f"sine=duration={seconds}", "-c:a", audio,
                   "-c:v", "libx264", "-preset", "ultrafast", "-metadata", "location=+33.6+073.1/"]
        if hdr:
            command += ["-vf", "format=yuv420p10le,setparams=color_primaries=bt2020:color_trc=arib-std-b67"
                               ":colorspace=bt2020nc"]
        else:
            command += ["-pix_fmt", "yuv420p"]
        subprocess.run(command + [str(path)], check=True)
        return path

    def details(self, path):
        return subprocess.run([self.ffmpeg, "-hide_banner", "-i", str(path)], capture_output=True, text=True,
                              encoding="utf-8", errors="replace").stderr

    def test_an_hdr_clip_becomes_a_smaller_standard_mp4(self):
        source = self.clip(hdr=True)
        before = source.read_bytes()
        self.assertTrue(converter.probe(self.ffmpeg, source).hdr)
        seen = []
        result = converter.convert(self.ffmpeg, source, Options(size="720p"), on_progress=seen.append)
        self.assertEqual(result, self.folder / "clip.mp4")
        made = converter.probe(self.ffmpeg, result)
        self.assertEqual((made.width, made.height, made.hdr, made.audio), (960, 720, False, "aac"))
        self.assertNotIn("073.1", self.details(result))
        self.assertEqual(seen[-1], 1.0)
        self.assertEqual(source.read_bytes(), before)
        self.assertEqual(sorted(p.name for p in self.folder.iterdir()), ["clip.mov", "clip.mp4"])

    def test_a_portrait_clip_is_sized_by_its_short_side_and_never_enlarged(self):
        tall = converter.convert(self.ffmpeg, self.clip("tall.mov", "1080x1920"), Options(size="720p"))
        small = converter.convert(self.ffmpeg, self.clip("small.mov", "320x240"), Options(size="1080p"))
        self.assertEqual([(i.width, i.height) for i in (converter.probe(self.ffmpeg, tall),
                                                        converter.probe(self.ffmpeg, small))],
                         [(720, 1280), (320, 240)])

    def test_cancelling_leaves_no_file(self):
        source = self.clip(seconds=5)
        stop = threading.Event()
        stop.set()
        with self.assertRaises(Cancelled):
            converter.convert(self.ffmpeg, source, Options(), cancel=stop)
        self.assertEqual([p.name for p in self.folder.iterdir()], ["clip.mov"])

    def test_a_broken_file_says_so_and_leaves_no_file(self):
        broken = self.folder / "broken.MOV"
        broken.write_bytes(b"\x00" * 203)
        with self.assertRaisesRegex(ConvertError, "not a playable video"):
            converter.convert(self.ffmpeg, broken, Options())
        self.assertEqual([p.name for p in self.folder.iterdir()], ["broken.MOV"])

    def test_a_failed_conversion_says_why_and_leaves_no_file(self):
        source = self.clip()
        missing = self.folder / "no such folder"
        with self.assertRaisesRegex(ConvertError, "(?i)no such file"):
            converter.convert(self.ffmpeg, source, Options(), folder=missing)
        self.assertEqual([p.name for p in self.folder.iterdir()], ["clip.mov"])


if __name__ == "__main__":
    unittest.main()
