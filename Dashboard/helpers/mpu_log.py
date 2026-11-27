"""Every reading of the motion sensor, written to a file, so that a fault can be looked at afterwards.

One row per line the car sends (25 a second): the time, then the car's own numbers (before the dashboard flips any sign) and
the wheel encoders. Lines starting with # are events: a session starting, a command sent to the car, the sensor getting stuck.
"""
import time
from pathlib import Path

COLUMNS = "time,seconds,yaw,pitch,roll,gx,gy,gz,ax,ay,az,wheel_l,wheel_r,wheel_speed"
MAX_BYTES = 20_000_000       # about three hours; then the file is put aside as ....old.csv and a new one begun
FLUSH_S = 1.0                # what has been written reaches the disk at least this often


class MpuLog:
    """A log that never gets in the way: if the file cannot be written, nothing happens."""

    def __init__(self, path, max_bytes=MAX_BYTES):
        self.path, self.max_bytes = Path(path), max_bytes
        self._file, self._size = None, 0
        self._start = self._flushed = None
        self._started = False

    def write(self, stamp, text, wheels, speed):
        """A line from the car that arrived at `stamp` (time.monotonic()); wheels = (left cm, right cm) or None."""
        if text.count(",") != 8:
            return self.note(stamp, f"car says: {text}")
        left, right = (f"{v:.1f}" for v in wheels) if wheels else ("", "")
        self._add(stamp, f"{self._clock()},{self._seconds(stamp):.3f},{text},{left},{right},{speed:.1f}")

    def note(self, stamp, text):
        self._add(stamp, f"# {self._clock()} {self._seconds(stamp):.3f} {text}")

    def flush(self):
        try:
            if self._file is not None:
                self._file.flush()
        except OSError:
            pass

    def close(self):
        try:
            if self._file is not None:
                self._file.close()
        except OSError:
            pass
        self._file = None

    # -- inside --
    @staticmethod
    def _clock():
        now = time.time()
        return f"{time.strftime('%H:%M:%S', time.localtime(now))}.{int(now % 1 * 1000):03d}"

    def _seconds(self, stamp):
        if self._start is None:
            self._start = stamp
        return stamp - self._start

    def _open(self):
        self._size = self.path.stat().st_size if self.path.exists() else 0
        self._file = open(self.path, "a", encoding="utf-8")
        if self._size == 0:
            self._put(COLUMNS)

    def _put(self, line):
        self._file.write(line + "\n")
        self._size += len(line) + 2

    def _add(self, stamp, line):
        try:
            if self._file is None:
                self._open()
            if not self._started:
                self._started = True
                self._put(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} session started")
            if self._size > self.max_bytes:
                self.close()
                self.path.replace(self.path.with_suffix(".old" + self.path.suffix))
                self._open()
            self._put(line)
            if self._flushed is None:
                self._flushed = stamp
            elif stamp - self._flushed >= FLUSH_S:
                self._file.flush()
                self._flushed = stamp
        except OSError:
            self.close()
