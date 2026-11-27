"""Collect: driving the car to a paper ball and picking it up. This file holds the pieces that decide things, without any screen or
Bluetooth: the tracker that finds a steady ball in the detector's results (here), and the Approach state machine (below)."""
import math
from collections import deque
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Ball:
    x_ratio: float          # the box centre across the picture: 0 = left edge, 1 = right edge
    bottom_y: float         # the bottom edge of the box in picture pixels (how far down the picture the ball touches the floor)
    stamp: float            # time.monotonic() when the detector finished this picture
    # the rest is only for the log
    width: float = field(default=0.0, compare=False)              # the box, in picture pixels
    height: float = field(default=0.0, compare=False)
    confidence: float = field(default=0.0, compare=False)         # how sure the model is, 0 to 1
    in_view: int = field(default=1, compare=False)                # balls on the floor in this picture
    bottom_ratio: float = field(default=None, compare=False)      # the bottom edge down the picture: 0 = top, 1 = bottom


class BallTracker:
    """Picks the nearest ball (the lowest in the picture) from each detector result and calls it steady after `need_frames`
    results in a row whose ball moved no more than `max_jump_px` pixels. One stray box does not make the car turn."""

    HELD_ABOVE_FRAC = 0.23       # a box whose bottom edge is in the top 23% of the picture is the ball in the gripper, not on the floor
    #                              (the floor's far edge is at ~25%, so a ball 2 m away, bottom edge ~34%, is still a floor ball)

    def __init__(self, need_frames=3, max_jump_px=15.0, max_gap=2, hold_s=1.0):
        # max_gap results in a row without a ball are put up with (a hard ball flickers around the confidence threshold), and so is
        # any gap shorter than hold_s seconds (a moving car blurs the ball for a few results)
        self.need_frames, self.max_jump_px, self.max_gap, self.hold_s = need_frames, max_jump_px, max_gap, hold_s
        self.reset()

    def reset(self):
        self._seen = deque(maxlen=100)    # (time, place across) of every result that showed a ball on the floor
        self._last_id = None
        self._ball = None
        self._centre = None          # the previous result's centre (x, y) in pixels
        self._count = 0
        self._gap = 0                # results in a row without a ball
        self._size = 0.0             # the ball's box, longest side, in pixels
        self._last_seen = None       # time of the last result that showed a ball

    def update(self, result):
        if result.frame_id == self._last_id:
            return
        self._last_id = result.frame_id
        floor = [det for det in result.detections if det.y2 >= self.HELD_ABOVE_FRAC * result.height]
        if not floor:
            self._gap += 1
            if self._gap > self.max_gap and (self._last_seen is None or result.stamp - self._last_seen > self.hold_s):
                self._ball, self._centre, self._count = None, None, 0
            return                   # a short gap: the ball is still there, and keeps the time of the picture it was last seen in
        self._gap = 0
        self._last_seen = result.stamp
        d = max(floor, key=lambda det: det.y2)
        centre = ((d.x1 + d.x2) / 2.0, (d.y1 + d.y2) / 2.0)
        size = float(max(d.x2 - d.x1, d.y2 - d.y1))
        # a close ball is boxed whole in one result and by its top half in the next: its centre moves about as far as its own size
        allowed = max(self.max_jump_px, 0.75 * max(size, self._size))
        if self._centre is not None and max(abs(centre[0] - self._centre[0]), abs(centre[1] - self._centre[1])) <= allowed:
            self._count += 1
        else:
            self._count = 1
        self._centre, self._size = centre, size
        self._seen.append((result.stamp, centre[0] / result.width))
        self._ball = Ball(centre[0] / result.width, float(d.y2), result.stamp, width=float(d.x2 - d.x1), height=float(d.y2 - d.y1),
                          confidence=float(d.score), in_view=len(floor), bottom_ratio=d.y2 / result.height)

    def ball(self):
        """The newest steady ball, else None."""
        return self._ball if self._count >= self.need_frames else None

    def seen_since(self, stamp):
        """How many results since `stamp` showed a ball on the floor (steady or not, so a ball that is still rolling counts)."""
        return sum(1 for t, _ in self._seen if t >= stamp)

    def spread_since(self, stamp):
        """(leftmost, rightmost, results) of the ball's place across the picture since `stamp`; None if none showed it."""
        places = [x for t, x in self._seen if t >= stamp]
        return (min(places), max(places), len(places)) if places else None


# ---- the approach -------------------------------------------------------------------------------------------------------------

from typing import NamedTuple, Optional, Protocol          # noqa: E402

from helpers.ball_distance import ball_angle_deg, turn_to_ball   # noqa: E402


class Reading(NamedTuple):
    cm: Optional[float]      # the ultrasonic distance; None = no echo (nothing in front, or too far)
    stamp: float             # time.monotonic() when it arrived


class Telemetry(NamedTuple):
    heading: Optional[float] = None      # the gyro's heading in degrees, + = left, not wrapped at 360
    left_cm: Optional[float] = None      # how far each wheel's encoder has counted
    right_cm: Optional[float] = None
    gripper: str = "no readings"         # the sensor in the gripper, in words: "38 mm", "out of range", "no readings"


class World(Protocol):
    def ball(self) -> Optional[Ball]: ...                    # the steady ball, if there is one
    def frame_stamp(self) -> Optional[float]: ...            # time of the newest camera result
    def reading(self) -> Optional[Reading]: ...              # the newest ultrasonic reading
    def camera_cm(self, ball: Ball) -> Optional[float]: ...  # distance by the camera model; None when not calibrated
    def balls_seen_since(self, stamp: float) -> int: ...     # camera results since then that showed a ball on the floor
    def ball_spread(self, since: float) -> Optional[tuple]: ...   # (leftmost, rightmost, results), for the log
    def telemetry(self) -> Telemetry: ...                    # gyro, wheels and gripper sensor, for the log


class Approach:
    """Drives the car to the paper ball and plays the pick-up move. The camera gives the left-right direction; the ultrasonic sensor
    (read while the car stands still, on fresh data only) says when the ball is in the pick range; the camera distance cross-checks it.
    Anything unexpected stops the car (abort) instead of guessing."""

    TURN_TOL_DEG = 1.5              # car degrees: no turn when the ball needs less than this
    TURN_SHORT_DEG = 1.0            # the car stops this much short of a turn (median of 106 logged turns): ask for this much more
    SENSOR_SEES_DEG = 10.0          # the sensor's reading is the ball's distance only with the ball this close to straight ahead
    ASSUMED_CM = 40.0               # distance used for a turn when neither the sensor nor the camera can say
    SETTLE_S = 0.6
    FRAME_AFTER_S = 0.3
    READING_AFTER_S = 0.5
    NO_BALL_S = 3.0                 # standing still this long without seeing the ball = lost
    FRAME_STALE_S = 1.5
    FRAME_RESUME_S = 3.0            # the camera link often drops while the motors run: after a motion it gets this long to come back
    READING_STALE_S = 2.0
    MAX_DECISIONS = 25
    MAX_TIME_S = 90.0
    FAR_CM = 60.0
    HOP_MIN_CM = 3.0
    HOP_MAX_CM = 25.0
    BLIND_HOP_CM = 10.0
    SLOW_BELOW_CM = 15.0
    APPROACH_FRACTION = 0.7         # near the ball a hop covers this much of what is left (the car overshoots), until ...
    CREEP_CM = 8.0                  # ... this little is left, which is driven in one small step
    BLIND_WAIT_S = 1.5              # something closer than the pick range and no ball in view for this long: too close to see it
    RECOVER_BACK_CM = 12.0          # lost after driving forward: back up this far and look again
    TOO_CLOSE_EXTRA_CM = 6.0        # too close: back up to the pick distance and this much more
    MAX_RECOVERS = 3                # back-ups per attempt before giving up
    AGREE_CM = 8.0
    AGREE_FRACTION = 0.25
    CAMERA_MARGIN_CM = 5.0
    VERIFY_S = 1.5                  # after the move the car stands still this long, and the camera looks for a ball left on the floor
    MISS_FRAMES = 2                 # this many results showing a floor ball (after the move) means the pick missed
    MAX_ATTEMPTS = 5
    RETRY_BACK_CM = 10.0            # after a miss: back up this far, then turn to the ball's new place and approach again

    def __init__(self, world, car, player, move, spot):
        self.world, self.car, self.player, self.move, self.spot = world, car, player, move, spot
        self.active, self.ok, self.status, self.log = False, None, "Not started", []
        self._state = "deciding"                    # deciding / moving / picking / verifying
        self._started = 0.0                         # when this attempt began (the time limit is per attempt)
        self._verify_since = 0.0
        self.attempts = 0
        self._ready_since = float("-inf")           # when the last motion ended and the car stood still
        self._last_ball_seen = 0.0
        self._decisions = 0
        self._recoveries = 0                        # back-ups to find the ball again, in this attempt
        self._drove_forward = False                 # a forward hop was made in this attempt: the ball may now be too close to see
        self._now, self._pressed = 0.0, 0.0         # the latest tick, and when Collect was pressed (for the log)
        self._motion = None                         # (kind, value, telemetry before) of a motion whose result is not logged yet
        self._last_ball = None                      # the last steady ball seen, for the log

    # -- control --
    def start(self, now):
        if self.active or not self.car.connected():
            return False
        self.active, self.ok, self.status, self.log = True, None, "Looking for the ball", []
        self._state, self._started, self._ready_since = "deciding", now, float("-inf")
        self._last_ball_seen, self._decisions, self.attempts = now, 0, 1
        self._recoveries, self._drove_forward = 0, False
        self._now, self._pressed, self._motion, self._last_ball = now, now, None, None
        return True

    def abort(self, reason, show=True):
        """Stop everything. `show`: first write down where things stood (not needed right after a decision, which just did)."""
        if not self.active:
            return
        if show:
            self._log_result()
            self._log_scene(self._now, self.world.ball(), self.world.reading())
        self.active, self.ok, self.status = False, False, f"Stopped: {reason}"
        self.car.stop()
        if self.player.active:
            self.player.abort(reason)

    def _finish_ok(self):
        self.active, self.ok, self.status = False, True, "Done: picked"

    # -- the loop --
    def tick(self, now):
        if not self.active:
            return
        self._now = now
        self._last_ball = self.world.ball() or self._last_ball
        if not self.car.connected():
            return self.abort("the car disconnected")
        if now - self._started > self.MAX_TIME_S:
            return self.abort("took too long")
        if self._state == "picking":
            return self._tick_picking(now)
        if self._state == "verifying":
            return self._tick_verifying(now)
        if self._state == "moving":
            if self.car.motion_active():
                return
            if not self.car.motion_ok():
                return self.abort("the car could not finish the move")
            self._ready_since, self._state = now, "deciding"
        if now < self._ready_since + self.SETTLE_S:
            return
        self._tick_deciding(now)

    def _tick_picking(self, now):
        self.player.tick(now)
        if self.player.active:
            return
        if self.player.ok:
            self._state, self._verify_since, self.status = "verifying", now, "Checking the pick"
        else:
            self.abort(self.player.status)

    def _tick_verifying(self, now):
        """The move is done. A ball still lying on the floor means it missed (or rolled away): back up and try again."""
        if now < self._verify_since + self.VERIFY_S:
            return
        if self._camera_dead(now, self._verify_since):
            return self.abort("no new camera frame")
        frame = self.world.frame_stamp()
        if frame is None or frame < self._verify_since + self.FRAME_AFTER_S:
            return                                   # no picture taken since the move: "no ball seen" would prove nothing
        seen = self.world.balls_seen_since(self._verify_since + self.FRAME_AFTER_S)
        found = "a ball is still on the floor" if seen >= self.MISS_FRAMES else "no ball left on the floor"
        self.log.append(f"  check  : {found} (seen in {seen} pictures after the move)")
        self._log_scene(now, self.world.ball(), self.world.reading())
        if seen < self.MISS_FRAMES:
            return self._finish_ok()
        if self.attempts >= self.MAX_ATTEMPTS:
            return self.abort(f"missed {self.attempts} times", show=False)      # the check just wrote where things stood
        self.attempts += 1
        self._decisions, self._started, self._last_ball_seen = 0, now, now
        self._recoveries, self._drove_forward = 0, False
        if not self._start_drive(-self.RETRY_BACK_CM, 50):
            return self.abort("could not start the drive")
        self._state = "moving"
        self.status = f"Missed: backing up, attempt {self.attempts} of {self.MAX_ATTEMPTS}"

    def _camera_dead(self, now, since):
        """No camera result for too long. `since` is when the car last stood still: a pause before that is not held against the
        camera (it drops out while driving), but it must be back within FRAME_RESUME_S of the car stopping."""
        frame = self.world.frame_stamp()
        last = self._started if frame is None else frame
        if last < since:
            return now - since > self.FRAME_RESUME_S
        return now - last > self.FRAME_STALE_S

    def _tick_deciding(self, now):
        if self._camera_dead(now, self._ready_since):
            return self.abort("no new camera frame")
        reading = self.world.reading()
        if now - (self._started if reading is None else reading.stamp) > self.READING_STALE_S:
            return self.abort("no distance readings")
        ball = self.world.ball()
        if ball is not None:
            self._last_ball_seen = now
        elif self._too_close(now, reading):
            return self._back_up_to_look(now, reading.cm)
        elif now - max(self._last_ball_seen, self._ready_since) > self.NO_BALL_S:      # counted from when the car stood still
            if self._drove_forward and self._recoveries < self.MAX_RECOVERS:
                return self._back_up_to_look(now, None)       # we drove at it and now cannot see it: probably right under the nose
            return self.abort("lost the ball")
        if ball is None or reading is None or ball.stamp < self._ready_since + self.FRAME_AFTER_S \
                or reading.stamp < self._ready_since + self.READING_AFTER_S:
            return                                  # wait for a picture and a reading taken while the car stood still
        if self._decisions >= self.MAX_DECISIONS:
            return self.abort("too many steps")
        self._decisions += 1
        self._log_result()
        self.log.append(f"step {self._decisions}, attempt {self.attempts}, {now - self._pressed:.0f} s after Collect was pressed")
        self._log_scene(now, ball, reading)
        self._act(now, *self._decide(ball, reading.cm))

    # -- too close to see --
    def _too_close(self, now, reading):
        """The camera cannot see a ball a few cm away. No ball in view, and the sensor (read while standing still) says something is
        closer than the pick range, for long enough that it is not just a flicker."""
        return (reading is not None and reading.cm is not None and reading.cm < self.spot.pick_min
                and reading.stamp >= self._ready_since + self.READING_AFTER_S
                and now - max(self._ready_since, self._started) >= self.BLIND_WAIT_S and self._recoveries < self.MAX_RECOVERS)

    def _back_up_to_look(self, now, cm):
        """Reverse so the ball comes back into the camera's view. `cm` = the close sensor reading, None when the ball is just lost."""
        if cm is not None and cm < self.spot.pick_min:
            back, why = min(self.HOP_MAX_CM, self.spot.ultrasonic_cm - cm + self.TOO_CLOSE_EXTRA_CM), f"sensor {cm:.1f} cm"
        else:
            back, why = self.RECOVER_BACK_CM, "not in view after driving at it"
        back = round(back, 1)
        self._recoveries += 1
        self._log_result()
        self._log_scene(now, None, self.world.reading())
        self.log.append(f"no ball ({why}): backing up {back:g} cm to look again, try {self._recoveries} of {self.MAX_RECOVERS}")
        if not self._start_drive(-back, 50):
            return self.abort("could not start the drive")
        self._last_ball_seen = now
        self._state, self.status = "moving", f"Cannot see the ball: backing up {back:g} cm"

    # -- the decision table: returns (action, value, speed, why) --
    def _decide(self, ball, r):
        spot = self.spot
        c = self.world.camera_cm(ball)
        pick = spot.ultrasonic_cm
        tol = None if c is None else max(self.AGREE_CM, self.AGREE_FRACTION * c)
        # closer than the pick range (and not something in front of a farther ball): this close the picture jumps about, so
        # back off first and aim from there
        if r is not None and r < spot.pick_min and not (c is not None and r < c - tol):
            return "drive", -max(self.HOP_MIN_CM, pick - r), 50, "too close to aim or pick"
        if r is not None and abs(ball_angle_deg(ball.x_ratio)) <= self.SENSOR_SEES_DEG:
            distance, source = r, "sensor"
        elif c is not None:
            distance, source = c, "camera"
        else:
            distance, source = self.ASSUMED_CM, "assumed"
        need = turn_to_ball(ball.x_ratio, distance, spot.x_ratio, pick)
        if abs(need) > self.TURN_TOL_DEG:
            why = (f"the ball needs {abs(need):.1f} deg, plus {self.TURN_SHORT_DEG:.1f} because the car stops short; "
                   f"distance {distance:.0f} cm ({source})")
            return "turn", need + math.copysign(self.TURN_SHORT_DEG, need), 0, why
        agree = c is None or r is None or abs(r - c) <= tol
        hop = lambda d: min(self.HOP_MAX_CM, max(self.HOP_MIN_CM, d))
        # fast only when nothing is near: at 100% a hop toward a ball within FAR_CM ran past it, too close to be seen
        speed = lambda cm, away: 50 if cm < self.SLOW_BELOW_CM or away <= self.FAR_CM else 100
        step = lambda left: left if left <= self.CREEP_CM else round(left * self.APPROACH_FRACTION, 1)
        if r is not None and spot.pick_min <= r <= spot.pick_max:
            if not agree:
                return "stop", f"the sensor says {r:.0f} cm but the camera says {c:.0f} cm", 0, ""
            return "pick", 0, 0, f"front {r:.1f} cm is in the pick range {spot.pick_min:.0f}-{spot.pick_max:.0f} cm"
        if r is not None and r < spot.pick_min:
            return "stop", "something is closer than the ball", 0, ""
        if r is not None and r <= self.FAR_CM:
            if agree:
                cm = hop(step(r - pick))
            elif r < c:
                return "stop", "something is closer than the ball", 0, ""
            else:
                cm = hop(c - pick - self.CAMERA_MARGIN_CM)
            return "drive", cm, 50, f"{r - pick:.0f} cm left to the pick distance"
        if c is not None:
            cm = hop(c - pick - self.CAMERA_MARGIN_CM)
            return "drive", cm, speed(cm, c), "beyond the sensor: by the camera estimate"
        return "drive", self.BLIND_HOP_CM, 50, "beyond the sensor: a short blind hop"

    def _act(self, now, action, value, speed, why):
        if action == "stop":
            self.log.append(f"  decided: stop - {value}")
            return self.abort(value, show=False)
        if action == "turn":
            self.log.append(f"  decided: turn {self._side(value)} {abs(value):.1f} deg ({why})")
            before = self.world.telemetry()
            if not self.car.start_turn(value):
                return self.abort("could not start the turn", show=False)
            self._motion = ("turn", value, before)
            self._state, self.status = "moving", f"Turning {value:+.0f} deg"
        elif action == "drive":
            self.log.append(f"  decided: drive {'forward' if value > 0 else 'back'} {abs(value):.1f} cm at {speed}% ({why})")
            if not self._start_drive(value, speed):
                return self.abort("could not start the drive", show=False)
            if value > 0:
                self._drove_forward = True
            self._state, self.status = "moving", f"Driving {value:+.0f} cm"
        else:
            self.log.append(f"  decided: pick up ({why})")
            self.player.start(self.move, now)
            self._state, self.status = "picking", "Picking up"

    def _start_drive(self, cm, speed):
        before = self.world.telemetry()
        if not self.car.start_drive(cm, speed, 3):
            return False
        self._motion = ("drive", cm, before)
        return True

    # -- the log: what the camera and the sensors saw, and what the car then did --
    @staticmethod
    def _side(deg):
        return "left" if deg > 0 else "right"

    @staticmethod
    def _front(reading):
        if reading is None:
            return "front: no reading"
        return "front: nothing in range" if reading.cm is None else f"front {reading.cm:.1f} cm"

    def sensors_text(self):
        """The front and gripper sensors in a few words (the window adds this to every step of the pick-up move)."""
        return f"{self._front(self.world.reading())}, gripper {self.world.telemetry().gripper}"

    def _camera_text(self, now, ball):
        if ball is None:
            last = self._last_ball
            if last is None:
                return "no ball in view; none seen yet"
            return f"no ball in view; last seen {last.x_ratio * 100:.0f}% across, {max(0.0, now - last.stamp):.1f} s ago"
        offset = (ball.x_ratio - self.spot.x_ratio) * 100
        line = "on the grab line" if abs(offset) < 0.5 else f"{abs(offset):.0f}% {'right' if offset > 0 else 'left'} of the grab line"
        parts = [f"ball {ball.x_ratio * 100:.0f}% across ({line})"]
        if ball.width:
            parts.append(f"box {ball.width:.0f}x{ball.height:.0f} px")
        if ball.bottom_ratio is not None:
            parts.append(f"bottom edge {ball.bottom_ratio * 100:.0f}% down")
        if ball.confidence:
            parts.append(f"sure {ball.confidence * 100:.0f}%")
        parts.append(f"{ball.in_view} ball{'s' if ball.in_view != 1 else ''} in view")
        text = ", ".join(parts)
        spread = self.world.ball_spread(self._ready_since + self.FRAME_AFTER_S)
        if spread:
            text += f"; across in the last {spread[2]} pictures: {spread[0] * 100:.0f}-{spread[1] * 100:.0f}%"
        return text

    def _log_scene(self, now, ball, reading):
        self.log.append(f"  camera : {self._camera_text(now, ball)}")
        t = self.world.telemetry()
        parts = [self._front(reading), f"gripper {t.gripper}"]
        estimate = None if ball is None else self.world.camera_cm(ball)
        if estimate is not None:
            parts.insert(1, f"camera estimate {estimate:.1f} cm")
        if t.heading is not None:
            parts.append(f"heading {t.heading:+.1f}")
        if t.left_cm is not None:
            parts.append(f"wheels L {t.left_cm:.1f} R {t.right_cm:.1f} cm")
        self.log.append(f"  sensors: {', '.join(parts)}")

    def _log_result(self):
        """What the last turn or drive really did, by the gyro and the wheel encoders."""
        if self._motion is None:
            return
        (kind, value, before), self._motion = self._motion, None
        after = self.world.telemetry()
        turned = None if before.heading is None or after.heading is None else after.heading - before.heading
        wheels = None
        if before.left_cm is not None and after.left_cm is not None:
            wheels = f"wheels L {after.left_cm - before.left_cm:+.1f} R {after.right_cm - before.right_cm:+.1f} cm"
        if kind == "turn":
            asked = f"(asked {self._side(value)} {abs(value):.1f})"
            text = (f"the turn ended {asked}, no gyro reading" if turned is None else
                    f"turned {self._side(turned)} {abs(turned):.1f} deg {asked}")
            parts = [text, wheels]
        else:
            heading = None if turned is None else f"heading moved {self._side(turned)} {abs(turned):.1f} deg"
            parts = [f"drove (asked {value:+.1f} cm)", wheels, heading]
        self.log.append(f"  result : {'; '.join(part for part in parts if part)}")
