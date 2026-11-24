"""How far away the paper ball is, from where its box sits in the camera picture, like the guide lines of a car's reversing camera.

Something that touches the floor appears lower in the picture the closer it is, and the curve is 1 / distance = a x row + b, where
`row` is the bottom edge of the ball's box. The owner puts a ball at known distances (tape measure, from the camera lens) and the
model is fitted to them (the maths of camera_distance.py). It measures things that touch the floor, and it is only used for the paper
ball: the detector does not find walls or other objects. The ultrasonic sensor is the more exact one at short range; this model reaches
farther and cross-checks the sensor.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

from helpers.paths import data_file

FOV_DEG = 36.0                         # the picture's horizontal field of view. Measured on the car, not the sensor's data sheet figure (55): in four gyro turns the ball moved 1.5-1.7x more than 55 predicted
GUIDE_DISTANCES_CM = (20, 30, 40, 60, 80)
CALIB_FILE = data_file("ball_calibration.json")
RED, YELLOW, GREEN = (255, 0, 0), (255, 255, 0), (0, 255, 0)      # RGB, like the dashboard's camera frames


class CalibrationError(ValueError):
    """The calibration cannot be made or read."""


class DistanceModel:
    """1 / distance (cm) = a x row + b, where row is the bottom edge of the ball's box in picture pixels."""

    def __init__(self, a, b, points=None):
        self.a, self.b = float(a), float(b)
        self.points = [(float(d), float(y)) for d, y in (points or [])]     # the calibration samples (distance cm, bottom row)

    @classmethod
    def fit(cls, samples):
        """samples: [(distance_cm, bottom_row)]; at least 3, and the curve must rise (closer = lower in the picture)."""
        if len(samples) < 3:
            raise CalibrationError("need at least 3 distances to calibrate")
        d = np.array([s[0] for s in samples], float)
        y = np.array([s[1] for s in samples], float)
        if (d <= 0).any():
            raise CalibrationError("every distance must be more than 0 cm")
        a, b = np.polyfit(y, 1.0 / d, 1)
        if a <= 0:
            raise CalibrationError("the ball's bottom edge did not move down as it came closer: check the distances")
        return cls(a, b, samples)

    def distance_cm(self, bottom_row):
        """Distance to a ball whose box ends at this row; None above the horizon (no sensible distance)."""
        v = self.a * bottom_row + self.b
        return 1.0 / v if v > 1e-6 else None

    def y_for(self, distance_cm):
        """The picture row where something on the floor at this distance has its bottom edge."""
        return (1.0 / distance_cm - self.b) / self.a

    def errors(self):
        """[(real cm, estimated cm, estimated - real)] for each calibration sample."""
        out = []
        for real, row in self.points:
            estimated = self.distance_cm(row)
            if estimated is not None:
                out.append((real, estimated, estimated - real))
        return out

    def to_dict(self):
        return {"a": self.a, "b": self.b, "points": [list(p) for p in self.points]}

    @classmethod
    def from_dict(cls, data):
        try:
            return cls(float(data["a"]), float(data["b"]), [(float(p[0]), float(p[1])) for p in data.get("points", [])])
        except (KeyError, TypeError, ValueError, IndexError) as e:
            raise CalibrationError(f"the calibration file is not a distance model ({e!r})") from e


def save_model(path, model):
    Path(path).write_text(json.dumps(model.to_dict(), indent=2) + "\n", encoding="utf-8")


def load_model(path):
    """The saved model, None if there is no file; CalibrationError if the file cannot be read."""
    path = Path(path)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise CalibrationError(f"cannot read {path.name}: {e}") from e
    if not isinstance(data, dict):
        raise CalibrationError(f"{path.name} is not a distance model")
    return DistanceModel.from_dict(data)


def ball_angle_deg(x_ratio, fov_deg=FOV_DEG):
    """Direction of something at this place across the picture (0 = left edge, 1 = right edge), in degrees from the centre line;
    + = to the left, the same sign as the car's turns."""
    focal = 0.5 / math.tan(math.radians(fov_deg) / 2.0)                    # in picture widths
    return math.degrees(math.atan((0.5 - x_ratio) / focal))


def draw_guides(rgb, model, distances=GUIDE_DISTANCES_CM):
    """A copy of the picture with a horizontal line and a label at the row of each distance (red near, yellow in the middle, green
    far), like a reversing camera. Lines that fall outside the picture are left out. The same picture if there is no model."""
    if model is None:
        return rgb
    out = rgb.copy()
    h, w = out.shape[:2]
    font = max(0.45, w / 1300.0)
    for d in distances:
        row = int(round(model.y_for(d)))
        if not 0 <= row < h:
            continue
        color = RED if d < 30 else YELLOW if d < 50 else GREEN
        cv2.line(out, (0, row), (w - 1, row), color, 1)
        cv2.putText(out, f"{d} cm", (8, max(14, row - 4)), cv2.FONT_HERSHEY_SIMPLEX, font, color, 1)
    return out


def draw_spot_marker(rgb, x_ratio):
    """A copy of the picture with a dashed vertical line where the taught grab direction is."""
    out = rgb.copy()
    h, w = out.shape[:2]
    x = int(round(x_ratio * w))
    for y in range(0, h, 16):
        cv2.line(out, (x, y), (x, min(h - 1, y + 8)), YELLOW, 2)
    cv2.putText(out, "grab direction", (min(x + 6, max(0, w - 120)), 16), cv2.FONT_HERSHEY_SIMPLEX, max(0.45, w / 1300.0), YELLOW, 1)
    return out
