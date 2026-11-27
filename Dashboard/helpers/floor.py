"""Is the floor flat? Worked out from the car's tilt and accelerometer (the MPU-6050), for the Motion card.

Only a standing car can judge a floor: while it speeds up or brakes, the push looks like a slope to the accelerometer. The
tilt is shown all the time, but marked as "moving" then, and "flat" is only said of a floor. And the
sensor may not sit level on the car, so "Set as flat" (on a floor known to be level) stores that slant and takes it out.
"""
import json
import math
from pathlib import Path
from typing import NamedTuple

FLAT_DEG = 2.0              # a floor within this of level (both ways) is called flat
FULL_DEG = 15.0             # the tilt that takes the dot of the level to the edge of its circle
TURNING_DPS = 3.0           # turning faster than this ...
WHEELS_CM_S = 0.5           # ... or wheels faster than this: the car is moving
TOTAL_TOL_G = 0.08          # a standing car's total acceleration is 1 g; further off than this points at the sensor
SMOOTHING = 0.15            # how much of each new accelerometer reading is taken (25 arrive every second)


def tilt_words(pitch, roll, flat=1.0):
    """pitch + = front up, roll + = leaning right, in degrees."""
    parts = []
    if abs(pitch) >= flat:
        parts.append(f"{abs(pitch):.0f}° front {'up' if pitch > 0 else 'down'}")
    if abs(roll) >= flat:
        parts.append(f"{abs(roll):.0f}° {'right' if roll > 0 else 'left'}")
    return " · ".join(parts) or "level"


def dot_offset(pitch, roll, radius):
    """Where the level's dot sits, in pixels from the centre (x right, y down; the top of the circle is the car's front).
    It shows which way the car leans: right side low = right, front up (so it leans back) = down."""
    x, y = roll / FULL_DEG * radius, pitch / FULL_DEG * radius
    away = math.hypot(x, y)
    if away > radius:
        x, y = x * radius / away, y * radius / away
    return x, y


class FloorState(NamedTuple):
    still: bool          # the car stands still, so pitch and roll are the floor's
    pitch: float         # the car's tilt now, with the flat setting taken out
    roll: float
    words: str           # "flat", "8° right", ... ("level" instead of "flat" while moving: that says nothing of the floor)
    accel: tuple         # smoothed (x, y, z) in g
    total: float         # its length in g
    suspect: bool        # standing still, yet the total is far from 1 g


class Floor:
    def __init__(self, zero=(0.0, 0.0)):
        self.zero = (float(zero[0]), float(zero[1]))     # (pitch, roll) the sensor reads on a level floor
        self.reset()

    def reset(self):
        self._accel = None
        self._tilt = None                # the sensor's (pitch, roll) at the last reading, while it was still

    def update(self, pitch, roll, accel, turning_dps, wheels_cm_s):
        """One reading of the car: its tilt (degrees), accelerometer (g), the fastest of its three turning speeds and its
        wheel speed."""
        if self._accel is None:
            self._accel = tuple(accel)
        else:
            self._accel = tuple(old + SMOOTHING * (new - old) for old, new in zip(self._accel, accel))
        total = math.sqrt(sum(a * a for a in self._accel))
        still = abs(turning_dps) < TURNING_DPS and abs(wheels_cm_s) < WHEELS_CM_S
        self._tilt = (pitch, roll) if still else None
        shown = (pitch - self.zero[0], roll - self.zero[1])
        words = tilt_words(*shown, flat=FLAT_DEG)
        if still and words == "level":
            words = "flat"
        return FloorState(still, shown[0], shown[1], words, self._accel, total, still and abs(total - 1.0) > TOTAL_TOL_G)

    def set_flat(self):
        """This floor is level: what the sensor reads now becomes zero. False unless the car stands still."""
        if self._tilt is None:
            return False
        self.zero = self._tilt
        return True


def load_zero(path):
    """The saved flat setting; (0, 0) if there is none or the file cannot be read."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return float(data["pitch"]), float(data["roll"])
    except (OSError, ValueError, KeyError, TypeError):
        return 0.0, 0.0


def save_zero(path, zero):
    Path(path).write_text(json.dumps({"pitch": zero[0], "roll": zero[1]}) + "\n", encoding="utf-8")
