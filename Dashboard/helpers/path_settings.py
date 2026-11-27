"""The adjustable settings of path driving, and the two-wheel distance rule.

The car stops its drives by itself on the wheel encoders; these settings say how fast to drive, when to slow down, how exact to
be, and which wheel to believe when the two differ a lot. The car has the same wheel rule (AI_Car.ino, drive_wheels.h) and is
told the setting with the command W.
"""
import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path

SPEEDS = (50, 100)                   # the speeds where the wheel encoders are accurate
TRUSTS = ("lower", "right", "left")  # in the order of the car's numbers 0, 1, 2
ASSISTS = ("off", "gyro", "encoders", "both")        # in the order of the car's numbers 0..3
JUMP_CM = 3000.0                     # the counters changed by more than this between two readings: reset or restarted

RANGES = {"stop_closed_cm": (3.0, 100.0), "stop_default_cm": (3.0, 100.0), "stop_other_cm": (3.0, 100.0),
          "slow_within_cm": (0.0, 200.0), "hand_slow_cm": (0.0, 200.0), "arrive_cm": (1.0, 15.0), "turn_within_deg": (1.0, 10.0),
          "turn_extra_deg": (0.0, 3.0), "differ_cm": (1.0, 100.0), "differ_pct": (0.0, 100.0)}
SWITCH = ("on", "off")
CHOICES = {"fast_pct": SPEEDS, "slow_pct": SPEEDS, "trust": TRUSTS, "assist": ASSISTS, "manual_guard": SWITCH}


@dataclass(frozen=True)
class PathSettings:
    slow_within_cm: float = 20.0     # this close to a point the car changes to the slow speed
    fast_pct: int = 100
    slow_pct: int = 50
    arrive_cm: float = 3.0           # a point counts as reached within this
    turn_within_deg: float = 2.0     # no turn when the car points this close to where it should
    turn_extra_deg: float = 1.0      # asked for on top of a turn, because the car stops about this much short
    trust: str = "right"             # the wheel that decides when the two differ a lot
    differ_cm: float = 4.0           # "a lot": more than this many cm apart ...
    differ_pct: float = 15.0         # ... and more than this share of the larger count
    assist: str = "gyro"             # straight-drive assist; the encoder assist would follow a faulty counter
    stop_closed_cm: float = 5.0      # stop this far from an obstacle with the arm in the "All closed" pose ...
    stop_default_cm: float = 10.0    # ... in the "All default" pose ...
    stop_other_cm: float = 20.0      # ... and in any other (the arm may stick out further)
    manual_guard: str = "on"         # driving forward by hand ends at these distances too (the car creeps up to them)
    hand_slow_cm: float = 50.0       # ... and is slowed to 50% this far before them (0 = not slowed)

    def stop_cm(self, pose):
        """The stop distance for an arm pose: "closed", "default" or "other"."""
        return {"closed": self.stop_closed_cm, "default": self.stop_default_cm}.get(pose, self.stop_other_cm)

    def changed(self, **values):
        """A copy with these values, each kept within what works; one that is not understood keeps the old value."""
        good = {}
        for name, value in values.items():
            if name in RANGES:
                try:
                    number = float(str(value).strip().replace(",", "."))
                except ValueError:
                    continue
                if math.isfinite(number):
                    low, high = RANGES[name]
                    good[name] = min(max(number, low), high)
            elif name in CHOICES:
                options = CHOICES[name]
                for option in options:
                    if str(value).strip() == str(option):
                        good[name] = option
        return replace(self, **good)

    def wheel_command(self):
        return f"W{TRUSTS.index(self.trust)} {self.differ_cm:.0f} {self.differ_pct:.0f}"

    def assist_mode(self):
        return ASSISTS.index(self.assist)

    def describe(self):
        return ", ".join(f"{name} {value:g}" if isinstance(value, float) else f"{name} {value}"
                         for name, value in asdict(self).items())


def load_settings(path):
    """The saved settings; the defaults for anything missing or unreadable."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return PathSettings().changed(**{k: v for k, v in data.items() if k in RANGES or k in CHOICES})
    except (OSError, ValueError, AttributeError, TypeError):
        return PathSettings()


def save_settings(path, settings):
    Path(path).write_text(json.dumps(asdict(settings), indent=2) + "\n", encoding="utf-8")


def wheels_disagree(left, right, settings):
    return abs(left - right) > max(settings.differ_cm, settings.differ_pct / 100.0 * max(abs(left), abs(right)))


def straight_distance(left, right, settings):
    """How far a straight drive went, from what each wheel counted since it began."""
    if wheels_disagree(left, right, settings):
        if settings.trust == "right":
            return right
        if settings.trust == "left":
            return left
        return left if abs(left) < abs(right) else right
    return (left + right) / 2.0


class Odometer:
    """The distance along the way, from the car's two wheel counters. During a straight drive the wheel rule applies to the
    whole drive so far; otherwise (turning, standing) the mean counts, so that a turn in place does not move the car."""

    def __init__(self):
        self.total = 0.0
        self.forget_counters()

    def forget_counters(self):
        """The car's counters were zeroed or the car restarted: the next reading is a new starting point."""
        self._last = self._from = None
        self._done = 0.0

    def feed(self, left, right, straight, settings):
        last, self._last = self._last, (left, right)
        if last is None or max(abs(left - last[0]), abs(right - last[1])) > JUMP_CM:
            self._from = None
            return self.total
        if straight:
            if self._from is None:
                self._from, self._done = last, 0.0
            gone = straight_distance(left - self._from[0], right - self._from[1], settings)
            self.total += gone - self._done
            self._done = gone
        else:
            self._from = None
            self.total += ((left - last[0]) + (right - last[1])) / 2.0
        return self.total
