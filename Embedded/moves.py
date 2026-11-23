"""Recorded moves for the arm and the car: step data, checks, descriptions and JSON files.

A move is a dict {"name", "version", "steps": [...]}; each step is one of
    {"type": "arm",   "servos": [{"i": 1, "angle": 150, "speed": 30}, ...], "wait": 0.5}   servo angles (deg, deg/s), then a pause (s)
    {"type": "drive", "cm": 20}        straight, + forward / - back, always at 100% speed
    {"type": "turn",  "deg": 90}       on the spot, + left / - right
    {"type": "wait",  "seconds": 1}
Moves are plain JSON files (see tests/example_grab.json; your own live in moves/) and can be edited by hand; validate_move() checks them
before anything plays. A move can also say "smooth", "assist_gyro" and "assist_encoders" (true/false).
"""
import copy
import json
import math
from pathlib import Path
from typing import Protocol

MOVES_DIR = Path(__file__).with_name("moves")
ARM_MARGIN_S = 0.3          # the car does not report where the arm is, so a step waits its travel time plus this
DRIVE_SPEED_PCT = 100       # the default drive speed
DRIVE_SPEEDS = (50, 100)    # the speeds where the wheel encoders are accurate (measured against a tape)
MAX_DRIVE_CM = 300
MAX_TURN_DEG = 360
MAX_WAIT_S = 60
SPEED_RANGE = (1, 600)      # deg/s, like the dashboard's servo speed slider


class MoveFileError(Exception):
    """A move file that cannot be read as a move."""


# ---------------------------------------------------------------- making steps
def arm_step(servos, wait=0.5):
    """servos: {servo index: (angle, speed deg/s)}."""
    return {"type": "arm",
            "servos": [{"i": i, "angle": angle, "speed": speed} for i, (angle, speed) in sorted(servos.items())],
            "wait": wait}


def drive_step(cm, speed=DRIVE_SPEED_PCT):
    """Drive `cm` straight (+ forward, - back) at `speed` percent; the car counts its wheel encoders and stops at the distance."""
    return {"type": "drive", "cm": cm, "speed": speed}


def turn_step(deg):
    return {"type": "turn", "deg": deg}


def wait_step(seconds):
    return {"type": "wait", "seconds": seconds}


def new_move(name):
    return {"name": name, "version": 1, "steps": []}


# ---------------------------------------------------------------- checking
def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _validate_arm(n, step, limits, errors):
    servos = step.get("servos")
    if not isinstance(servos, list) or not servos:
        errors.append(f"Step {n}: arm needs a non-empty list of servos")
    else:
        for entry in servos:
            if not isinstance(entry, dict) or not _is_int(entry.get("i")) or not 0 <= entry["i"] < len(limits):
                errors.append(f"Step {n}: servo {entry!r} needs an index 0..{len(limits) - 1}")
                continue
            i = entry["i"]
            lo, hi = limits[i]
            angle, speed = entry.get("angle"), entry.get("speed")
            if not _is_int(angle) or not lo <= angle <= hi:
                errors.append(f"Step {n}: servo {i} angle {angle!r} must be a whole number from {lo} to {hi}")
            if not _is_int(speed) or not SPEED_RANGE[0] <= speed <= SPEED_RANGE[1]:
                errors.append(f"Step {n}: servo {i} speed {speed!r} must be a whole number from {SPEED_RANGE[0]} to {SPEED_RANGE[1]} deg/s")
    if "wait" in step and not (_is_number(step["wait"]) and 0 <= step["wait"] <= MAX_WAIT_S):
        errors.append(f"Step {n}: wait {step['wait']!r} must be a number from 0 to {MAX_WAIT_S} s")


def _validate_number(n, step, field, label, limit, errors):
    value = step.get(field)
    if not _is_number(value) or value == 0 or abs(value) > limit:
        errors.append(f"Step {n}: {label} {field} {value!r} must be a number, not 0, and at most {limit} in size")


def validate_move(move, limits):
    """Every problem found, one string each ('Step 3: ...'); an empty list means the move is fine to play.
    limits[i] is servo i's (min, max) angle."""
    if not isinstance(move, dict) or not isinstance(move.get("steps"), list):
        return ["The move needs a list of steps"]
    errors = []
    for key in ("smooth", "assist_gyro", "assist_encoders"):
        if key in move and not isinstance(move[key], bool):
            errors.append(f"The move's '{key}' setting must be true or false")
    for n, step in enumerate(move["steps"], 1):
        if not isinstance(step, dict):
            errors.append(f"Step {n}: not a step ({step!r})")
            continue
        kind = step.get("type")
        if kind == "arm":
            _validate_arm(n, step, limits, errors)
        elif kind == "drive":
            _validate_number(n, step, "cm", "drive", MAX_DRIVE_CM, errors)
            if "speed" in step and not (_is_number(step["speed"]) and step["speed"] in DRIVE_SPEEDS):
                errors.append(f"Step {n}: drive speed {step['speed']!r} must be {DRIVE_SPEEDS[0]} or {DRIVE_SPEEDS[1]} "
                              f"(the speeds where the wheel encoders are accurate)")
        elif kind == "turn":
            _validate_number(n, step, "deg", "turn", MAX_TURN_DEG, errors)
        elif kind == "wait":
            seconds = step.get("seconds")
            if not _is_number(seconds) or not 0 < seconds <= MAX_WAIT_S:
                errors.append(f"Step {n}: wait seconds {seconds!r} must be a number above 0 and at most {MAX_WAIT_S}")
        else:
            errors.append(f"Step {n}: unknown type {kind!r}")
    return errors


# ---------------------------------------------------------------- describing
def _fmt(value):
    return f"{value:g}"


def describe_step(step, names):
    """One line for the step list. Never raises: a broken step is described as such."""
    try:
        kind = step["type"]
        if kind == "arm":
            parts = [f"{names[s['i']] if 0 <= s['i'] < len(names) else 'Servo ' + str(s['i'])} {s['angle']}°@{s['speed']}/s"
                     for s in step["servos"]]
            text = "Arm: " + ", ".join(parts)
            if step.get("wait", 0):
                text += f", then wait {_fmt(step['wait'])} s"
            return text
        if kind == "drive":
            return (f"Drive: {'forward' if step['cm'] > 0 else 'back'} {_fmt(abs(step['cm']))} cm "
                    f"at {_fmt(step.get('speed', DRIVE_SPEED_PCT))}%")
        if kind == "turn":
            return f"Turn: {'left' if step['deg'] > 0 else 'right'} {_fmt(abs(step['deg']))}°"
        if kind == "wait":
            return f"Wait: {_fmt(step['seconds'])} s"
    except (KeyError, TypeError, IndexError):
        pass
    return f"Invalid step: {step!r}"


def arm_duration(step, current, margin=ARM_MARGIN_S):
    """How long to give an arm step: the slowest servo's travel time from `current` angles, a margin, then the step's own pause."""
    travel = max((abs(s["angle"] - current.get(s["i"], s["angle"])) / s["speed"] for s in step["servos"]), default=0.0)
    return travel + margin + step.get("wait", 0)


def sync_speeds(step, current):
    """A copy of the arm step whose speeds make every servo that moves arrive at the same moment (a straight line between the
    poses instead of the short mover finishing first). The recorded speeds are limits: a speed is only ever lowered, never above
    what was recorded. A servo that does not move keeps its speed."""
    result = copy.deepcopy(step)
    travel = [abs(s["angle"] - current.get(s["i"], s["angle"])) for s in step["servos"]]
    times = [d / s["speed"] for d, s in zip(travel, step["servos"]) if d > 0]
    if not times:
        return result
    slowest = max(times)
    for d, s in zip(travel, result["servos"]):
        if d > 0:
            s["speed"] = max(1, math.ceil(round(d / slowest, 6)))      # round(): 10 / (20 / 60) must be 30, not 30.000000000000004
    return result


# ---------------------------------------------------------------- files
def load_move(path):
    try:
        move = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise MoveFileError(f"Cannot read {Path(path).name}: {e}") from e
    if not isinstance(move, dict) or not isinstance(move.get("steps"), list):
        raise MoveFileError(f"{Path(path).name} is not a move (it needs an object with a list of steps)")
    return move


def save_move(path, move):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(move, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


# ---------------------------------------------------------------- playing
class PlayerCar(Protocol):
    """What the player needs from the car (the dashboard adapts its own commands to this)."""

    def servo(self, index: int, angle: int, speed: int) -> None: ...
    def servo_angles(self) -> dict: ...                  # last commanded angle of each servo
    def start_drive(self, cm: float, speed: int, assist: int) -> bool: ...   # the car counts its encoders and stops at the distance;
    #                                                      assist: straight-drive assist, 1 = gyro heading + 2 = encoders; False = could not start
    def start_turn(self, deg: float) -> bool: ...
    def motion_active(self) -> bool: ...                 # a drive or turn is still running
    def motion_ok(self) -> bool: ...                     # the last drive or turn ended normally
    def stop(self) -> None: ...
    def connected(self) -> bool: ...


class MovePlayer:
    """Plays a move's steps one after another. The caller calls tick(now) every ~50 ms; time is passed in, so tests drive it by hand."""

    def __init__(self, car, names):
        self.car, self.names = car, names
        self.active = False
        self.ok = None               # None until a move has ended, then True (finished) or False (stopped)
        self.status = "Idle"
        self.index = -1
        self._move = None
        self._state = "idle"         # "timed" (arm or wait step), "motion" (drive or turn) or "idle"
        self._until = 0.0
        self._current = {}           # servo index -> angle the arm should be at (from the car, then from the steps)

    def start(self, move, now):
        """False (and nothing sent) when already playing, nothing to play, or the car is not connected."""
        if self.active:
            return False
        if not move.get("steps"):
            self.status = "Nothing to play"
            return False
        if not self.car.connected():
            self.status = "Car not connected"
            return False
        self._move, self.index = copy.deepcopy(move), -1      # edits made in the window while it plays must not shift the steps
        self._current = dict(self.car.servo_angles())
        self.active, self.ok = True, None
        self._next(now)
        return self.active

    def tick(self, now):
        if not self.active:
            return
        if not self.car.connected():
            self.abort("car disconnected")
        elif self._state == "timed":
            if now >= self._until:
                self._next(now)
        elif self._state == "motion" and not self.car.motion_active():
            if self.car.motion_ok():
                self._next(now)
            else:
                self.abort("the drive or turn did not finish normally")

    def abort(self, reason):
        if not self.active:
            return
        self.active, self.ok, self._state = False, False, "idle"      # inactive first: a stop hook may call abort again
        self.status = f"Stopped: {reason}"
        self.car.stop()

    def _finish(self):
        self.active, self.ok, self._state = False, True, "idle"
        self.status = "Done"

    def _next(self, now):
        self.index += 1
        steps = self._move["steps"]
        if self.index >= len(steps):
            self._finish()
            return
        step = steps[self.index]
        self.status = f"Step {self.index + 1}/{len(steps)}: {describe_step(step, self.names)}"
        kind = step["type"]
        if kind == "arm":
            # "smooth" moves: the servos arrive together, and a following arm step starts as soon as this one is done (no
            # safety margin: a servo that is still moving just takes the new target). The margin stays before a drive or turn.
            smooth = bool(self._move.get("smooth"))
            play = sync_speeds(step, self._current) if smooth else step
            for s in play["servos"]:
                self.car.servo(s["i"], s["angle"], s["speed"])
            following = steps[self.index + 1] if self.index + 1 < len(steps) else None
            margin = 0.0 if smooth and following is not None and following.get("type") == "arm" else ARM_MARGIN_S
            self._until = now + arm_duration(play, self._current, margin)
            for s in play["servos"]:
                self._current[s["i"]] = s["angle"]
            self._state = "timed"
        elif kind == "wait":
            self._until = now + step["seconds"]
            self._state = "timed"
        elif kind == "drive":
            # straight assist for the drive: gyro heading (1) and wheel encoders (2), both on unless the move turns one off
            assist = (1 if self._move.get("assist_gyro", True) else 0) | (2 if self._move.get("assist_encoders", True) else 0)
            if self.car.start_drive(step["cm"], step.get("speed", DRIVE_SPEED_PCT), assist):
                self._state = "motion"
            else:
                self.abort("could not start the drive")
        elif kind == "turn":
            if self.car.start_turn(step["deg"]):
                self._state = "motion"
            else:
                self.abort("could not start the turn")
