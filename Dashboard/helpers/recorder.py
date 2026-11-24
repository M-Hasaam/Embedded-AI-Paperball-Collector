"""Saves the camera's frames to a folder while you drive around, to make training pictures later.

The frames are the camera's exact JPEG bytes (no re-encoding, no rotation, no overlay), the same as the 'Save frame' button
writes, and the folder's recording.json says which rotation was set so they can be turned the way the model sees them.
"""
import json
import time
from pathlib import Path
from threading import Lock

# menu text -> frames saved per second (None = every frame the camera delivers)
RATES = {"1 per s": 1, "2 per s": 2, "5 per s": 5, "every frame": None}
DEFAULT_RATE = "5 per s"


class FrameRecorder:
    """One recording = one folder under `root`: f00000_t0000.00.jpg (frame number, seconds since the start) and recording.json.

    offer() is called from the camera thread for every new frame; start() and stop() from the dashboard."""

    def __init__(self, root, per_second=5, rotate="none", clock=time.monotonic):
        self.root = Path(root)
        self.min_gap = 0.0 if not per_second else 1.0 / per_second
        self.rotate = rotate
        self.clock = clock
        self.folder = None
        self.frames = 0
        self.error = ""
        self.active = False
        self._lock = Lock()
        self._started = self._last = None
        self._wall_start = ""

    def start(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        folder, n = self.root / f"rec_{stamp}", 1
        while folder.exists():
            n += 1
            folder = self.root / f"rec_{stamp}_{n}"
        folder.mkdir(parents=True)
        self.folder = folder
        self._wall_start = time.strftime("%Y-%m-%d %H:%M:%S")
        self._started = self.clock()
        self.active = True
        self._write_notes()

    def offer(self, jpeg):
        """Save this frame if recording and the rate limit allows it. Returns whether it was saved."""
        with self._lock:
            if not self.active:
                return False
            now = self.clock()
            if self._last is not None and now - self._last < self.min_gap - 1e-9:
                return False
            try:
                (self.folder / f"f{self.frames:05d}_t{now - self._started:07.2f}.jpg").write_bytes(jpeg)
            except OSError as e:
                self.error = f"recording stopped: {e}"
                self.active = False
                return False
            self.frames += 1
            self._last = now
            return True

    def stop(self):
        with self._lock:
            if self.active:
                self.active = False
        self._write_notes()

    def _write_notes(self):
        notes = {"started": self._wall_start, "rotate": self.rotate, "max_frames_per_second": None if not self.min_gap else 1.0 / self.min_gap,
                 "frames": self.frames, "seconds": round(self.clock() - self._started, 2) if self._started is not None else 0}
        try:
            (self.folder / "recording.json").write_text(json.dumps(notes, indent=2))
        except OSError:
            pass
