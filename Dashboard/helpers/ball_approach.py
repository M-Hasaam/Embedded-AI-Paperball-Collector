"""Collect: driving the car to a paper ball and picking it up. This file holds the pieces that decide things, without any screen or
Bluetooth: the tracker that finds a steady ball in the detector's results (here), and the Approach state machine (below)."""
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class Ball:
    x_ratio: float          # the box centre across the picture: 0 = left edge, 1 = right edge
    bottom_y: float         # the bottom edge of the box in picture pixels (how far down the picture the ball touches the floor)
    stamp: float            # time.monotonic() when the detector finished this picture


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
        self._seen = deque(maxlen=100)    # the time of every result that showed a ball on the floor
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
        self._seen.append(result.stamp)
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
        self._ball = Ball(centre[0] / result.width, float(d.y2), result.stamp)

    def ball(self):
        """The newest steady ball, else None."""
        return self._ball if self._count >= self.need_frames else None

    def seen_since(self, stamp):
        """How many results since `stamp` showed a ball on the floor (steady or not, so a ball that is still rolling counts)."""
        return sum(1 for t in self._seen if t >= stamp)


# ---- the approach -------------------------------------------------------------------------------------------------------------

from typing import NamedTuple, Optional, Protocol          # noqa: E402

from helpers.ball_distance import ball_angle_deg            # noqa: E402


class Reading(NamedTuple):
    cm: Optional[float]      # the ultrasonic distance; None = no echo (nothing in front, or too far)
    stamp: float             # time.monotonic() when it arrived


class World(Protocol):
    def ball(self) -> Optional[Ball]: ...                    # the steady ball, if there is one
    def frame_stamp(self) -> Optional[float]: ...            # time of the newest camera result
    def reading(self) -> Optional[Reading]: ...              # the newest ultrasonic reading
    def camera_cm(self, ball: Ball) -> Optional[float]: ...  # distance by the camera model; None when not calibrated
    def balls_seen_since(self, stamp: float) -> int: ...     # camera results since then that showed a ball on the floor


class Approach:
    """Drives the car to the paper ball and plays the pick-up move. The camera gives the left-right direction; the ultrasonic sensor
    (read while the car stands still, on fresh data only) says when the ball is in the pick range; the camera distance cross-checks it.
    Anything unexpected stops the car (abort) instead of guessing."""

    TURN_TOL_DEG = 2.0
    SETTLE_S = 0.6
    FRAME_AFTER_S = 0.3
    READING_AFTER_S = 0.5
    NO_BALL_S = 3.0
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

    # -- control --
    def start(self, now):
        if self.active or not self.car.connected():
            return False
        self.active, self.ok, self.status, self.log = True, None, "Looking for the ball", []
        self._state, self._started, self._ready_since = "deciding", now, float("-inf")
        self._last_ball_seen, self._decisions, self.attempts = now, 0, 1
        return True

    def abort(self, reason):
        if not self.active:
            return
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
        if self.world.balls_seen_since(self._verify_since + self.FRAME_AFTER_S) < self.MISS_FRAMES:
            return self._finish_ok()
        self.log.append(f"attempt {self.attempts}: the ball is still on the floor")
        if self.attempts >= self.MAX_ATTEMPTS:
            return self.abort(f"missed {self.attempts} times")
        self.attempts += 1
        self._decisions, self._started, self._last_ball_seen = 0, now, now
        if not self.car.start_drive(-self.RETRY_BACK_CM, 50, 3):
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
        elif now - self._last_ball_seen > self.NO_BALL_S:
            return self.abort("lost the ball")
        if ball is None or reading is None or ball.stamp < self._ready_since + self.FRAME_AFTER_S \
                or reading.stamp < self._ready_since + self.READING_AFTER_S:
            return                                  # wait for a picture and a reading taken while the car stood still
        if self._decisions >= self.MAX_DECISIONS:
            return self.abort("too many steps")
        self._decisions += 1
        self._act(now, *self._decide(ball, reading.cm))

    # -- the decision table: returns (action, value, speed, note) --
    def _decide(self, ball, r):
        spot = self.spot
        error = ball_angle_deg(ball.x_ratio) - ball_angle_deg(spot.x_ratio)
        c = self.world.camera_cm(ball)
        self._note = f"bearing {error:+.1f} deg, sensor {'none' if r is None else f'{r:.1f}'} cm, camera {'none' if c is None else f'{c:.1f}'} cm"
        if abs(error) > self.TURN_TOL_DEG:
            return "turn", error, 0, "turn"
        tol = None if c is None else max(self.AGREE_CM, self.AGREE_FRACTION * c)
        agree = c is None or r is None or abs(r - c) <= tol
        hop = lambda d: min(self.HOP_MAX_CM, max(self.HOP_MIN_CM, d))
        pick = spot.ultrasonic_cm
        speed = lambda cm: 50 if cm < self.SLOW_BELOW_CM else 100
        if r is not None and spot.pick_min <= r <= spot.pick_max:
            if not agree:
                return "stop", f"the sensor says {r:.0f} cm but the camera says {c:.0f} cm", 0, "disagree"
            return "pick", 0, 0, "pick"
        if r is not None and r < spot.pick_min:
            if c is not None and r < c - tol:
                return "stop", "something is closer than the ball", 0, "closer"
            return "drive", -max(self.HOP_MIN_CM, pick - r), 50, "back up"
        if r is not None and r <= self.FAR_CM:
            if agree:
                cm = hop(r - pick)
            elif r < c:
                return "stop", "something is closer than the ball", 0, "closer"
            else:
                cm = hop(c - pick - self.CAMERA_MARGIN_CM)
            return "drive", cm, speed(cm), "hop"
        if c is not None:
            cm = hop(c - pick - self.CAMERA_MARGIN_CM)
        else:
            cm = self.BLIND_HOP_CM
        return "drive", cm, speed(cm), "hop (no sensor reading)"

    def _act(self, now, action, value, speed, note):
        self.log.append(f"{self._note}: {note if action != 'stop' else value}")
        if action == "stop":
            return self.abort(value)
        if action == "turn":
            if not self.car.start_turn(value):
                return self.abort("could not start the turn")
            self._state, self.status = "moving", f"Turning {value:+.0f} deg"
        elif action == "drive":
            if not self.car.start_drive(value, speed, 3):
                return self.abort("could not start the drive")
            self._state, self.status = "moving", f"Driving {value:+.0f} cm"
        else:
            self.player.start(self.move, now)
            self._state, self.status = "picking", "Picking up"
