"""The simulated car and its sensors, with the errors the real one has. `SimCar` holds the truth (position, true heading, the clock);
`CarAPI` is the only thing the brain gets, and it offers just what the real car has: the gyro heading, the ultrasonic distance, the
camera, the gripper sensor, and commands to turn, drive, pick and drop. There are no encoders and no position.

Distances are centimetres, angles degrees counter-clockwise from +x (so a bearing + = to the left, like the real car's turns).
"""
import math
import random
from collections import defaultdict, namedtuple
from dataclasses import dataclass

from sim.world import Ball

SENSOR_OFFSET = 8.0          # the ultrasonic, the camera lens and the gripper are this far ahead of the car's centre
TICK = 0.1                   # seconds per step while driving
CAM_HALF_FOV = 18.0          # 36 degrees across, measured on the real car
CAM_RANGE = 220.0            # a ball is found out to about this far
TAG_RANGE = 500.0
US_MAX = 200.0
PICK_MIN, PICK_MAX = 10.0, 22.0       # a ball this far from the sensor, straight ahead, is in the jaws' reach
PICK_S = 6.0                          # an arm move to pick up a ball takes this long ...
DROP_S = 4.0                          # ... and one to let it go into the bin this long
GRIP_CLOSE = 0.55                     # the jaws close this far into a pick (0 = start, 1 = end)
GRIP_RELEASE = 0.55                   # the jaws open this far into a release
PICK_AIM_CM = 16.0                    # with no ball in reach the arm still reaches this far ahead of the sensor
DROP_MAX = 50.0                       # the bin's centre is within this of the sensor: the ball lands in the bin

Grip = namedtuple("Grip", "kind start end ok target")      # an arm move: 'pick' or 'drop', when, did it work, where it reached
Look = namedtuple("Look", "balls tag")     # balls: [(bearing deg, distance cm)]; tag: (bearing, distance) or None


def wrap(a):
    """Angle in (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


@dataclass
class Noise:
    """How wrong the car is. The defaults are meant to be a bit worse than the real car, not better."""
    gyro_bias_dps: float = 0.05        # leftover gyro drift after calibration (magnitude varies a bit, sign is random)
    gyro_jitter: float = 0.02          # random walk, degrees per sqrt(second)
    turn_resid_deg: float = 1.0        # a gyro turn ends this far (sigma) from its target
    speed_cm_s: float = 30.0           # what the brain believes the car's speed is
    speed_err: float = 0.08            # the real speed differs by this (sigma, fixed per run)
    hop_jitter: float = 0.03           # step-to-step variation
    veer_deg_per_cm: float = 0.03      # the path bends this much to the left per cm even with the gyro assist
    veer_sigma: float = 0.02
    us_sigma_cm: float = 1.0
    us_dropout: float = 0.03           # no echo at all
    thin_hit: float = 0.5              # a chair leg is seen this often
    ball_us_hit: float = 0.8           # a paper ball is seen by the ultrasonic this often
    cam_detect: float = 0.92
    cam_far_penalty: float = 0.3       # at the far end of the range the detection chance is this much lower
    cam_bearing_sigma: float = 0.7
    cam_dist_sigma: float = 0.10
    cam_false_pos: float = 0.02        # a ball that is not there, per camera picture
    tag_detect: float = 0.9
    tag_bearing_sigma: float = 0.8
    tag_dist_sigma: float = 0.06
    pick_success: float = 0.9
    jaw_flip: float = 0.02             # the gripper sensor is wrong this often
    turn_start_s: float = 0.15         # every turn takes this long to get going and settle ...
    turn_rate_dps: float = 120.0       # ... plus this many degrees a second while swinging
    look_s: float = 0.3                # a camera picture, from asking for it to having its result

    @classmethod
    def perfect(cls):
        return cls(gyro_bias_dps=0.0, gyro_jitter=0.0, turn_resid_deg=0.0, speed_err=0.0, hop_jitter=0.0, veer_deg_per_cm=0.0,
                   veer_sigma=0.0, us_sigma_cm=0.0, us_dropout=0.0, thin_hit=1.0, ball_us_hit=1.0, cam_detect=1.0,
                   cam_far_penalty=0.0, cam_bearing_sigma=0.0, cam_dist_sigma=0.0, cam_false_pos=0.0, tag_detect=1.0,
                   tag_bearing_sigma=0.0, tag_dist_sigma=0.0, pick_success=1.0, jaw_flip=0.0)


class SimCar:
    RADIUS = 10.0
    GUARD_CM = 12.0              # the car's own safety: a forward drive stops when the ultrasonic says something is this close

    def __init__(self, world, start_xy, start_heading=0.0, noise=None, seed=0):
        self.world, self.noise = world, noise or Noise()
        n = self.noise
        self.rng = random.Random(seed)
        self.x, self.y, self.h = float(start_xy[0]), float(start_xy[1]), float(start_heading)
        self.t = 0.0
        self.spent = defaultdict(float)                                   # seconds spent turning / driving / looking / ...
        self.gyro_err = 0.0                                              # what the gyro says minus the truth
        self.bias = n.gyro_bias_dps * self.rng.uniform(0.5, 1.5) * self.rng.choice((-1, 1))
        self.speed = n.speed_cm_s * (1 + self.rng.gauss(0, n.speed_err))
        self.veer = max(0.0, self.rng.gauss(n.veer_deg_per_cm, n.veer_sigma)) if n.veer_deg_per_cm else 0.0
        self.holding = False
        self.delivered = 0
        self.bumps = 0
        self.driven_cm = 0.0
        self.driven_since_pick = 0.0
        self.pick_xy = None
        self.events = []                                                  # ("pick"|"deliver"|"drop", t, x, y, extra)
        self.held = None                                                  # the Ball in the gripper
        self.grips = []                                                   # every arm move (Grip), for the playback
        self.path = [(self.x, self.y, self.t, self.h)]            # (x, y, time, true heading): for the playback

    # -- time and gyro --
    def _tick(self, dt, what="other"):
        self.t += dt
        self.spent[what] += dt
        self.gyro_err += self.bias * dt + self.rng.gauss(0, self.noise.gyro_jitter * math.sqrt(dt))

    def time(self):
        return self.t

    def heading(self):
        """The gyro's heading (what the car believes), wrapped to (-180, 180]."""
        return wrap(self.h + self.gyro_err)

    # -- motion --
    def turn(self, delta_deg):
        """Turn by delta on the gyro: the car stops when the gyro reads the target, so the true heading carries the gyro's drift."""
        target = self.heading() + delta_deg
        self._tick(abs(delta_deg) / self.noise.turn_rate_dps + (self.noise.turn_start_s if abs(delta_deg) > 0.5 else 0.0), "turn")
        self.h = target + self.rng.gauss(0, self.noise.turn_resid_deg) - self.gyro_err
        self.path.append((self.x, self.y, self.t, self.h))

    def drive(self, cm):
        """Drive forward (cm > 0) or back for the time that distance takes at the speed the car believes it has. Forward drives stop
        early when the ultrasonic sees something close (the guard). Returns the signed driving time in seconds: the only thing the
        car knows about how far it went."""
        n = self.noise
        sign = 1 if cm >= 0 else -1
        ticks = int(round(abs(cm) / n.speed_cm_s / TICK))
        elapsed = 0.0
        for _ in range(ticks):
            if sign > 0:
                us = self._ultrasonic()
                if us is not None and us < self.GUARD_CM:
                    break
            step = self.speed * TICK * (1 + self.rng.gauss(0, n.hop_jitter)) * sign
            a = math.radians(self.h)
            nx, ny = self.x + step * math.cos(a), self.y + step * math.sin(a)
            if self.world.blocked(nx, ny, self.RADIUS, self.t) and not self.world.blocked(self.x, self.y, self.RADIUS, self.t):
                self.bumps += 1                                              # (a car an obstacle was put on top of may drive out)
                self.events.append(("bump", self.t, self.x, self.y, None))
                break
            self.x, self.y = nx, ny
            self.h += self.veer * abs(step)
            self.driven_cm += abs(step)
            if self.holding:
                self.driven_since_pick += abs(step)
            self._tick(TICK, "drive")
            elapsed += TICK
            self.path.append((self.x, self.y, self.t, self.h))
        return sign * elapsed

    # -- sensors --
    def _sensor_xy(self):
        a = math.radians(self.h)
        return self.x + SENSOR_OFFSET * math.cos(a), self.y + SENSOR_OFFSET * math.sin(a)

    def _ultrasonic(self):
        n, rng = self.noise, self.rng
        sx, sy = self._sensor_xy()
        thin_seen = rng.random() < n.thin_hit
        ball_seen = rng.random() < n.ball_us_hit
        best = None
        for off in (-12, -6, 0, 6, 12):
            skip = []
            if not (abs(off) <= 6 and thin_seen):
                skip.append("thin")                                      # a thin thing only shows to the middle of the beam
            if not (abs(off) <= 6 and ball_seen):
                skip.append("ball")
            d, _ = self.world.cast(sx, sy, self.h + off, self.t, US_MAX, skip=tuple(skip))
            if d is not None and (best is None or d < best):
                best = d
        if best is None or rng.random() < n.us_dropout:
            return None
        return max(0.0, best + rng.gauss(0, n.us_sigma_cm))

    def ultrasonic(self):
        self._tick(0.05, "ultrasonic")
        return self._ultrasonic()

    def _rel(self, tx, ty):
        """(bearing relative to the car's nose, distance from the sensor) of a point."""
        sx, sy = self._sensor_xy()
        return wrap(math.degrees(math.atan2(ty - sy, tx - sx)) - self.h), math.hypot(tx - sx, ty - sy)

    def look(self):
        """One camera picture: the balls it detects and the bin's tag if visible."""
        n, rng, w = self.noise, self.rng, self.world
        self._tick(self.noise.look_s, "look")
        sx, sy = self._sensor_xy()
        balls = []
        for b in w.balls:
            if b.picked:
                continue
            rel, d = self._rel(b.x, b.y)
            if abs(rel) > CAM_HALF_FOV or d > CAM_RANGE:
                continue
            hit, _ = w.cast(sx, sy, self.h + rel, self.t, skip=("ball",))
            if hit is not None and hit < d - 4:
                continue                                                  # something is in the way
            if rng.random() > n.cam_detect * (1 - n.cam_far_penalty * d / CAM_RANGE):
                continue
            balls.append((rel + rng.gauss(0, n.cam_bearing_sigma), d * (1 + rng.gauss(0, n.cam_dist_sigma))))
        if rng.random() < n.cam_false_pos:
            balls.append((rng.uniform(-CAM_HALF_FOV, CAM_HALF_FOV), rng.uniform(60, 200)))
        tag = None
        rel, d = self._rel(*w.bin_xy)
        if abs(rel) <= CAM_HALF_FOV and d <= TAG_RANGE:
            hit, kind = w.cast(sx, sy, self.h + rel, self.t)
            if kind == "bin" and rng.random() < n.tag_detect:
                tag = (rel + rng.gauss(0, n.tag_bearing_sigma), d * (1 + rng.gauss(0, n.tag_dist_sigma)))
        return Look(balls, tag)

    def in_jaws(self):
        flip = self.rng.random() < self.noise.jaw_flip
        return self.holding != flip

    # -- gripper --
    def pick(self):
        """The arm reaches out, the jaws close on the ball (if one is in reach), the arm lifts back: PICK_S seconds, recorded as a Grip."""
        t0 = self.t
        self._tick(PICK_S, "grip")
        if self.holding:
            return
        best = None
        for b in self.world.balls:
            if b.picked:
                continue
            rel, d = self._rel(b.x, b.y)
            if PICK_MIN <= d <= PICK_MAX and abs(rel) < 10.0 and (best is None or d < best[0]):
                best = (d, b)
        ok = best is not None and self.rng.random() < self.noise.pick_success
        if best is not None:
            target = (best[1].x, best[1].y)
        else:                                                              # nothing in reach: it reaches for the spot ahead of the jaws
            sx, sy = self._sensor_xy()
            a = math.radians(self.h)
            target = (sx + PICK_AIM_CM * math.cos(a), sy + PICK_AIM_CM * math.sin(a))
        self.grips.append(Grip("pick", t0, self.t, ok, target))
        if ok:
            best[1].picked = True
            best[1].picked_at = t0 + GRIP_CLOSE * PICK_S                   # it leaves the floor when the jaws close, in the middle of the move
            self.held = best[1]
            self.holding = True
            self.driven_since_pick = 0.0
            self.pick_xy = (self.x, self.y)
            self.events.append(("pick", self.t, self.x, self.y, None))

    def drop(self):
        """The arm reaches out over the bin, the jaws open, the arm folds back: DROP_S seconds, recorded as a Grip."""
        if not self.holding:
            self._tick(DROP_S, "grip")
            return
        t0 = self.t
        self._tick(DROP_S, "grip")
        release = t0 + GRIP_RELEASE * DROP_S
        rel, d = self._rel(*self.world.bin_xy)
        self.holding = False
        if d <= DROP_MAX and abs(rel) < 30.0:
            self.grips.append(Grip("drop", t0, self.t, True, tuple(self.world.bin_xy)))
            self.delivered += 1
            self.held.delivered_at = release
            straight = math.hypot(self.pick_xy[0] - self.x, self.pick_xy[1] - self.y) if self.pick_xy else 0.0     # pick spot to drop spot
            self.events.append(("deliver", self.t, self.x, self.y, (self.driven_since_pick, straight)))
        else:                                                              # it fell on the floor in front of the car
            sx, sy = self._sensor_xy()
            a = math.radians(self.h)
            spot = (sx + 12 * math.cos(a), sy + 12 * math.sin(a))
            self.grips.append(Grip("drop", t0, self.t, False, spot))
            self.world.balls.append(Ball(spot[0], spot[1], created_at=release))
            self.events.append(("drop", self.t, self.x, self.y, None))


class CarAPI:
    """What the brain may use. It cannot see the position, the true heading or anything else about the truth."""
    __slots__ = ("_car",)

    def __init__(self, car):
        object.__setattr__(self, "_car", car)

    def time(self):
        return self._car.time()

    def heading(self):
        return self._car.heading()

    def turn(self, delta_deg):
        return self._car.turn(delta_deg)

    def drive(self, cm):
        return self._car.drive(cm)

    def ultrasonic(self):
        return self._car.ultrasonic()

    def look(self):
        return self._car.look()

    def in_jaws(self):
        return self._car.in_jaws()

    def pick(self):
        return self._car.pick()

    def drop(self):
        return self._car.drop()
