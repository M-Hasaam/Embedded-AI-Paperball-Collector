"""The taught grab spot: where a paper ball sits (left-right in the camera picture) and how far the ultrasonic sensor reads when the
car is in the right place for the recorded pick-up move. Collect drives until the ball is at this spot."""
import json
from dataclasses import dataclass
from pathlib import Path

from helpers.paths import data_file

SPOT_FILE = data_file("grab_spot.json")
RANGE_MARGIN_CM = 2.0                      # the pick range is the taught reading +/- this
MIN_CM, MAX_CM = 5.0, 60.0                 # a reading outside this is not a sensible grab distance


class GrabSpotError(ValueError):
    """The grab spot cannot be read or is not valid."""


@dataclass(frozen=True)
class GrabSpot:
    x_ratio: float                         # the ball's centre across the picture: 0 = left edge, 1 = right edge
    ultrasonic_cm: float                   # the sensor reading with the ball in the grab place
    camera_cm: float | None = None         # what the camera model said then (None without a calibration)

    @property
    def pick_min(self):
        return self.ultrasonic_cm - RANGE_MARGIN_CM

    @property
    def pick_max(self):
        return self.ultrasonic_cm + RANGE_MARGIN_CM


def _check(x_ratio, ultrasonic_cm):
    if not 0 < x_ratio < 1:
        raise GrabSpotError("the ball's place across the picture must be between 0 and 1")
    if not MIN_CM <= ultrasonic_cm <= MAX_CM:
        raise GrabSpotError(f"the sensor reading must be between {MIN_CM:g} and {MAX_CM:g} cm")


def save_spot(path, spot):
    _check(spot.x_ratio, spot.ultrasonic_cm)
    data = {"x_ratio": spot.x_ratio, "ultrasonic_cm": spot.ultrasonic_cm, "camera_cm": spot.camera_cm}
    Path(path).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def load_spot(path):
    """The saved spot, None if there is no file; GrabSpotError if the file is unreadable or not valid."""
    path = Path(path)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        camera = data.get("camera_cm")
        spot = GrabSpot(float(data["x_ratio"]), float(data["ultrasonic_cm"]), None if camera is None else float(camera))
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as e:
        raise GrabSpotError(f"cannot read {path.name}: {e!r}") from e
    _check(spot.x_ratio, spot.ultrasonic_cm)
    return spot
