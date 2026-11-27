"""MOV (or any video) -> smaller H.264 MP4, done by the ffmpeg that comes with the imageio-ffmpeg package."""

import re
import subprocess
import uuid
from dataclasses import dataclass
from pathlib import Path

SIZES = {"4K": 2160, "1080p": 1080, "720p": 720}      # the most pixels the short side may have
QUALITIES = {"Best": 18, "Balanced": 21, "Small": 24}  # x264 CRF: lower = better picture, bigger file
HDR_TRANSFERS = ("arib-std-b67", "smpte2084")          # HLG (iPhone) and PQ
# HDR to standard colour. Without this an iPhone HDR video looks washed out.
TONEMAP = ("zscale=t=linear:npl=203,format=gbrpf32le,zscale=p=bt709,tonemap=tonemap=hable:desat=0,"
           "zscale=t=bt709:m=bt709:r=tv,format=yuv420p,sidedata=mode=delete")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


class ConvertError(Exception):
    pass


class Cancelled(Exception):
    pass


@dataclass(frozen=True)
class VideoInfo:
    duration: float          # seconds
    width: int
    height: int
    hdr: bool
    audio: str | None        # codec name, None = no sound


@dataclass(frozen=True)
class Options:
    size: str = "1080p"
    quality: str = "Balanced"
    remove_location: bool = True


def find_ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except (ImportError, RuntimeError):
        raise ConvertError("ffmpeg is missing. Install it with:  py -3.12 -m pip install imageio-ffmpeg") from None


def parse_info(text):
    """Reads what `ffmpeg -i file` prints about a video."""
    duration = re.search(r"Duration: (\d+):(\d+):(\d+(?:\.\d+)?)", text)
    video = re.search(r"Stream #.*?: Video: .*", text)
    size = video and re.search(r", (\d+)x(\d+)", video.group())
    if not (duration and size):
        raise ConvertError("not a playable video")
    hours, minutes, seconds = (float(part) for part in duration.groups())
    audio = re.search(r"Stream #.*?: Audio: (\w+)", text)
    return VideoInfo(duration=round(hours * 3600 + minutes * 60 + seconds, 2),
                     width=int(size.group(1)), height=int(size.group(2)),
                     hdr=any(name in video.group() for name in HDR_TRANSFERS),
                     audio=audio.group(1) if audio else None)


def probe(ffmpeg, path):
    printed = subprocess.run([ffmpeg, "-hide_banner", "-i", str(path)], capture_output=True, text=True,
                             encoding="utf-8", errors="replace", creationflags=NO_WINDOW).stderr
    return parse_info(printed)


def build_command(ffmpeg, source, target, info, options):
    side = SIZES[options.size]
    # ffmpeg turns a portrait phone video upright first, so iw/ih are what the viewer sees. Never enlarges.
    scale = (f"scale=w='if(gt(iw,ih),-2,trunc(min({side},iw)/2)*2)'"
             f":h='if(gt(iw,ih),trunc(min({side},ih)/2)*2,-2)':flags=lanczos")
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostats", "-progress", "pipe:1", "-n",
               "-i", str(source), "-map", "0:v:0", "-map", "0:a:0?",
               "-vf", f"{scale},{TONEMAP if info.hdr else 'format=yuv420p'}",
               "-c:v", "libx264", "-preset", "medium", "-crf", str(QUALITIES[options.quality]), "-profile:v", "high"]
    if info.hdr:
        command += ["-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709"]
    if info.audio:
        command += ["-c:a", "copy"] if info.audio == "aac" else ["-c:a", "aac", "-b:a", "192k"]
    if options.remove_location:
        command += ["-map_metadata", "-1", "-movflags", "+faststart"]
    else:
        command += ["-map_metadata", "0", "-movflags", "+faststart+use_metadata_tags"]
    return command + [str(target)]


def output_path(source, folder=None):
    """<name>.mp4 beside the source (or in `folder`), numbered so that no existing file is ever replaced."""
    folder = Path(folder) if folder else Path(source).parent
    stem = Path(source).stem
    path = folder / f"{stem}.mp4"
    number = 2
    while path.exists():
        path = folder / f"{stem} ({number}).mp4"
        number += 1
    return path


def size_text(count):
    if count < 1024:
        return f"{count} bytes"
    if count < 1024 * 1024:
        return f"{count / 1024:.1f} KB"
    return f"{count / (1024 * 1024):.1f} MB"


def time_left_text(fraction, elapsed):
    if fraction <= 0:
        return ""
    left = elapsed * (1 - fraction) / fraction
    return f"about {left:.0f} s left" if left < 60 else f"about {left / 60:.0f} min left"


def convert(ffmpeg, source, options, folder=None, on_progress=None, cancel=None):
    """Makes the MP4 and returns its path. `cancel` is a threading.Event. The source is only ever read."""
    source = Path(source)
    info = probe(ffmpeg, source)
    if cancel and cancel.is_set():
        raise Cancelled
    target = output_path(source, folder)
    # Written under another name first, so a stopped or failed run never leaves a half video that looks finished.
    part = target.with_name(f"{target.stem}.{uuid.uuid4().hex[:8]}.part.mp4")
    errors = []
    process = subprocess.Popen(build_command(ffmpeg, source, part, info, options), stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               encoding="utf-8", errors="replace", creationflags=NO_WINDOW)
    try:
        for line in process.stdout:
            if cancel and cancel.is_set():
                raise Cancelled
            progress = re.match(r"(\w+)=(.*)", line.strip())
            if not progress:
                if line.strip():
                    errors.append(line.strip())
            elif progress.group(1) == "out_time_us" and progress.group(2).isdigit() and info.duration and on_progress:
                on_progress(min(int(progress.group(2)) / 1e6 / info.duration, 0.999))
        if process.wait() != 0:
            raise ConvertError("; ".join(errors[-3:]) or f"ffmpeg stopped with code {process.returncode}")
        if cancel and cancel.is_set():
            raise Cancelled
        final = output_path(source, folder)
        part.rename(final)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdout.close()
        part.unlink(missing_ok=True)
    if on_progress:
        on_progress(1.0)
    return final
