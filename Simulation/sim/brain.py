"""The roaming algorithm. It knows only what the real car knows (CarAPI: gyro heading, ultrasonic, camera, gripper sensor) and keeps its
own rough guess of where it is: gyro heading plus driving time x believed speed, corrected every time the bin's tag is seen.

It also keeps a map (sim/gridmap.py) of what it has measured: the floor it has seen to be free, the cells where something is, and the
floor the camera has looked at. With it:

  search   stop where the most never-looked-at floor can be seen from, spin round once (range readings and pictures) to look, then
           plan a path to the next such place and drive it, round whatever the map says is in the way
  collect  a ball is seen: go to it (round obstacles if need be), turn to it, hop closer, stop at the pick range, pick, check the
           gripper sensor
  home     carrying a ball: plan the shortest path to the bin; when its tag comes into view the tag takes over for the last metre
Nothing about the arena is given: a box that arrives is blocked on the map when it is seen or bumped, and cleared again when a reading
passes through where it was.
"""
import math
import random
from dataclasses import dataclass

from sim.car import SENSOR_OFFSET, wrap
from sim.gridmap import GridMap
from sim.lanes import lane_route

SPEED = 30.0                  # cm/s the brain believes the car drives at (the car's own figure)
SCAN_STEPS = 12               # a full scan looks every 30 degrees
HOP_GO = 80.0                 # the longest hop while following a planned path (an ultrasonic check before each)
HOP_HOME = 100.0              # the longest hop on the tag's final approach
US_RANGE = 200.0              # the ultrasonic's range
CLEAR_MARGIN = 20.0           # keep this much free in front of a hop
PICK_LO, PICK_HI, PICK_AIM = 12.0, 20.0, 16.0      # ultrasonic range at which the pick is tried, and where to stop
DROP_AT = 32.0                # drop when the tag is this close (sensor to bin centre)
BIN_RADIUS = 15.0             # the bin's size is known: the ultrasonic sees its near side this much closer than its centre
HOME_STANDOFF = 70.0          # the planned path to the bin ends this far from its centre; the tag does the rest
LANE_GAP = 70.0               # lawnmower search: lanes this far apart (the camera sees about this wide at a metre and a half)
LANE_MARGIN = 25.0            # ... keeping this far from the walls
LANE_HOP = 60.0               # ... with a picture and a distance check every step this long
LANE_ARRIVE = 20.0            # a lane end counts as reached this close
STRATEGIES = ("map", "lanes")
INSIDE_SLACK = 120.0          # a ball this far outside the arena (by the position guess) is a phantom
EXPLORE_DONE = 0.85            # 'nothing reachable to look at' only means the floor is covered when at least this much of it has been looked at
LANE_COVER_DONE = 0.9           # a lane pass that has looked at this much of the floor counts as a whole round
MAX_LANE_PASSES = 4           # (the lane search gives up on reaching that after this many passes)
FETCH_TRIES = 2               # a ball that is remembered but not there when the car comes back is tried this many times
SAME_BALL_CM = 45.0           # two sightings closer than this are the same ball
UNREACHABLE_S = 120.0         # a ball we could not get to is left alone this long


@dataclass
class Pending:
    """A ball the car has seen and not picked up: the place to come back to first."""
    id: int
    x: float
    y: float
    hits: int = 1                  # how many times it has been seen (a phantom is seen once; a ball again and again)
    tries: int = 0                 # how many times the car has gone back and not found it
    shown: bool = False            # it has been put on the picture


class Brain:
    def __init__(self, api, start_est=(40.0, 0.0), seed=1, strategy="map", bounds=None, rechecks=1):
        """strategy: 'map' (stop where the most unlooked-at floor can be seen, spin, look) or 'lanes' (lawnmower: drive straight lanes across
        the arena, looking ahead at every step). `bounds` = (x0, y0, x1, y1) of the arena in the bin's frame; the lanes need it."""
        if strategy not in STRATEGIES:
            raise ValueError(f"unknown search strategy {strategy!r}: use one of {STRATEGIES}")
        if strategy == "lanes" and bounds is None:
            raise ValueError("the lane search needs to know the arena: give bounds=(x0, y0, x1, y1)")
        self.strategy, self.bounds = strategy, bounds
        self.rechecks, self.rounds_done = rechecks, 0               # extra rounds after the whole floor has been looked at; rounds finished so far
        self.pending, self.ball_events, self._next_ball = [], [], 1   # balls seen but not picked up, and every change to that list (for the picture)
        self.route, self.route_i, self.pass_no = None, 0, 0         # the lane route being driven, how far along it, how many passes started
        self.picks, self._picks_at_pass_start, self._empty_passes, self._done = 0, 0, 0, False
        self.route_log = []                                         # (time, route) of every pass laid out, for the picture
        self.car = api
        self.rng = random.Random(seed)
        self.x, self.y = start_est                  # where the car thinks it is, in the bin's frame (the bin is the origin)
        self.map = GridMap()
        if bounds is not None:
            self.map.set_arena(*bounds)                              # the whole rectangle is the to-do list
        self.coverage_log = []                                      # (time, how much floor has been looked at, 0..1)
        self.map.unknown_cost = 1.5 if strategy == "map" else 3.0           # the map search likes straight lines across unexplored floor; the
                                                                    # lane search, which follows the walls, does better preferring floor it has seen
        self.carrying = False
        self.est_trail = [(self.x, self.y)]
        self.est_log = [(0.0, self.x, self.y)]          # the same with the time: for the 'where the car thinks it is' ghost
        self.collected = 0
        self.log = []
        self.deadline = float("inf")
        self.tag_heading = None                     # where the last scan saw the bin's tag: its absolute heading
        self.avoid = []                             # look-points we could not get to
        self.unreachable = []                       # (x, y, time) of balls we could not get to
        self.spotted = None                         # (absolute heading, distance) of a ball that came into view while driving
        self._last_range = None                     # (heading, range) of the newest ultrasonic reading

    # -- small helpers --
    def _note(self, text):
        self.log.append((round(self.car.time(), 1), text))

    def _over(self):
        return self.car.time() >= self.deadline

    def _face(self, heading, tol=1.0):
        delta = wrap(heading - self.car.heading())
        if abs(delta) > tol:
            self.car.turn(delta)

    def _range(self):
        """Three ultrasonic readings, the closest echo (a chair leg shows in only some of them); the map learns from it."""
        vals = [self.car.ultrasonic() for _ in range(3)]
        seen = [v for v in vals if v is not None]
        r = min(seen) if seen else None
        h = self.car.heading()
        self.map.ray(self.x, self.y, h, None if r is None else r + SENSOR_OFFSET, now=self.car.time())
        self.map.record_blocked(self.car.time())
        self.map.clear_blocked_near(0.0, 0.0, BIN_RADIUS + 25.0)    # the bin is where we are going, not something in the way
        self._last_range = (h, r)
        return r

    def _look(self):
        """One camera picture: the map notes the floor it covered, and the bin's tag (if in it) corrects the position guess."""
        look = self.car.look()
        h = self.car.heading()
        if look.tag is not None:
            self._fix(look.tag)
        limit = US_RANGE
        if self._last_range is not None and abs(wrap(h - self._last_range[0])) < 6.0 and self._last_range[1] is not None:
            limit = min(US_RANGE, self._last_range[1] + 15.0)          # the camera cannot see through what the ultrasonic hit
        self.map.wedge(self.x, self.y, h, limit, now=self.car.time())
        self.coverage_log.append((self.car.time(), self.map.coverage()))
        return look

    def _fix(self, tag):
        """The bin's tag gives the bearing and distance of the bin: that places the car, which cancels the drift."""
        bearing, dist = tag
        psi = math.radians(self.car.heading() + bearing)
        d = dist + SENSOR_OFFSET
        nx, ny = -d * math.cos(psi), -d * math.sin(psi)
        self.x, self.y = 0.7 * nx + 0.3 * self.x, 0.7 * ny + 0.3 * self.y
        self.est_trail.append((self.x, self.y))
        self.est_log.append((self.car.time(), self.x, self.y))

    def _move(self, cm):
        """Drive and update the position guess from the driving time (not the commanded distance: a hop can stop early)."""
        h0 = self.car.heading()
        elapsed = self.car.drive(cm)
        h1 = self.car.heading()
        mid = math.radians(h0 + wrap(h1 - h0) / 2.0)
        d = elapsed * SPEED
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)
        self.est_trail.append((self.x, self.y))
        self.est_log.append((self.car.time(), self.x, self.y))
        return d

    def _hop(self, cm, recover=True):
        """A hop that ends far short of what was asked means something is there (the guard stopped it, or it touched): the map keeps
        the place as blocked for good (a chair leg the sensor misses) and the car backs up a little."""
        d = self._move(cm)
        if recover and cm > 0 and d < 0.4 * cm:
            r = self._range()                                       # the guard stopped us: the reading marks what is there
            if r is None or r > 30.0:                               # nothing seen, yet we stopped: we touched something the sensor misses
                a = math.radians(self.car.heading())
                bx, by = self.x + 12.0 * math.cos(a), self.y + 12.0 * math.sin(a)
                if math.hypot(bx, by) > BIN_RADIUS + 25.0:
                    self.map.block(bx, by, now=self.car.time())
            self._note("hop stopped short: backing up")
            self._move(-12.0)
        return d

    @staticmethod
    def _bin_range(bin_centre):
        """What the distance sensor should read when it is looking at the bin: `bin_centre` is the distance from the car's CENTRE to the
        bin's centre, the sensor is SENSOR_OFFSET further forward, and the bin's near edge is BIN_RADIUS nearer than its centre."""
        return bin_centre - SENSOR_OFFSET - BIN_RADIUS

    def _clear_ahead(self, cm, bin_centre=None):
        """Is the way for a hop of cm free? An object where the bin should be (its tag is `bin_centre` away) is the bin, not an obstacle."""
        r = self._range()
        if r is None:
            return True
        if bin_centre is not None and abs(r - self._bin_range(bin_centre)) <= 10.0:
            return True
        return r > cm + CLEAR_MARGIN

    # -- scanning --
    def _scan(self, around=None, span=360.0):
        """Range readings and camera pictures while turning in 30 degree steps (a full turn, or just a sector): the map learns from
        every one. Returns the balls seen as (absolute heading, distance); `tag_heading` says where the bin's tag was seen, if it was."""
        balls = []
        self.tag_heading = None
        n = SCAN_STEPS if around is None else int(span // 30.0) + 1
        if around is not None:
            self._face(wrap(around - span / 2.0))
        for i in range(n):
            h = self.car.heading()
            self._range()
            look = self._look()
            if look.tag is not None:
                self.tag_heading = wrap(h + look.tag[0])
            for bearing, dist in look.balls:
                balls.append((wrap(h + bearing), dist))
            if around is not None and i == n - 1:
                break
            self.car.turn(360.0 / SCAN_STEPS if around is None else 30.0)
        self.map.scanned_at(self.x, self.y)                       # whether all round or a sector: we have looked from here
        return balls

    # -- driving along a planned path --
    def _goto(self, goal, stop_within=30.0, until=None, max_steps=80, hop_max=HOP_GO):
        """Drive toward `goal` (x, y in the bin's frame) along the shortest path the map allows, planning again after every hop because
        every ultrasonic reading and every bump changes the map. Returns 'arrived', 'until' (the `until` test said stop), 'blocked'
        (no path, or the way kept being blocked) or 'timeout'."""
        stuck = 0
        for _ in range(max_steps):
            if self._over():
                return "timeout"
            if math.hypot(goal[0] - self.x, goal[1] - self.y) <= stop_within:
                return "arrived"
            if until is not None and until():
                return "until"
            self.map.forget_old(self.car.time())
            path = self.map.plan((self.x, self.y), goal)
            if path is None:
                return "blocked"
            wp = self.map.shortcut(path)
            tx, ty = wp[1] if len(wp) > 1 else wp[0]
            seg = math.hypot(tx - self.x, ty - self.y)
            self._face(math.degrees(math.atan2(ty - self.y, tx - self.x)), tol=3.5)
            r = self._range()
            free = r if r is not None else US_RANGE
            hop = min(seg, hop_max, free - 20.0)
            if hop < 8.0:
                # the path leads straight into something (a wall shows on the map as a few dots, so the planner thinks there is a gap):
                # slide along it to the free side, then plan again from there
                stuck += 1
                if stuck >= 4:
                    return "blocked"
                self._sidestep((tx - self.x, ty - self.y))
                continue
            if self._hop(hop) < 0.4 * hop:
                stuck += 1
                if stuck >= 3:
                    return "blocked"
            else:
                stuck = 0
        return "timeout"

    def _sidestep(self, toward):
        """Something is right ahead: turn 90 degrees to the side that is nearer the way we want to go (the other if that is blocked)
        and slide along it for a bit. `toward` is the (dx, dy) we were heading in."""
        h = self.car.heading()
        want = math.degrees(math.atan2(toward[1], toward[0]))
        first = 90.0 if wrap(want - h) >= 0 else -90.0              # the side the goal lies on
        for off in (first, -first):
            self._face(wrap(h + off))
            r = self._range()
            free = r if r is not None else US_RANGE
            if free >= 45.0:
                self._hop(min(50.0, free - 25.0))
                return
        self.car.turn(90.0)                                          # boxed in on both sides: turn away and let the next plan try

    def _retrace(self, distance=60.0):
        """Back out the way we came: head for the place we were about `distance` cm ago along our own track. Used when the map says every
        way is blocked: the way we got in was not."""
        for px, py in reversed(self.est_trail[:-1]):
            if math.hypot(px - self.x, py - self.y) >= distance:
                self._face(math.degrees(math.atan2(py - self.y, px - self.x)))
                self._hop(min(distance + 20.0, math.hypot(px - self.x, py - self.y)))
                return True
        return False

    # -- the memory of balls waiting --
    def _remember(self, heading, dist, trusted=True):
        """Note a ball that has been seen but not picked up. `trusted` = seen in a scan or a picture of its own, so worth going back for at
        once; otherwise it has to be seen again first (the camera sometimes sees a ball that is not there)."""
        xy = self._ball_xy(heading, dist)
        if not self._inside(xy):
            return
        for p in self.pending:
            if math.hypot(p.x - xy[0], p.y - xy[1]) < SAME_BALL_CM:
                p.x, p.y, p.hits = 0.5 * (p.x + xy[0]), 0.5 * (p.y + xy[1]), p.hits + 1
                self._show(p)
                return
        p = Pending(self._next_ball, xy[0], xy[1], hits=2 if trusted else 1)
        self._next_ball += 1
        self.pending.append(p)
        self._show(p)

    def _show(self, p):
        """Put a ball on the picture once it is worth going back for (and move it if it was seen again a little differently)."""
        if p.hits >= 2:
            self.ball_events.append((self.car.time(), "move" if p.shown else "add", p.id, p.x, p.y))
            p.shown = True

    def _forget(self, p):
        if p in self.pending:
            self.pending.remove(p)
            if p.shown:
                self.ball_events.append((self.car.time(), "remove", p.id, p.x, p.y))

    def _forget_near(self, x, y, radius=SAME_BALL_CM):
        for p in [p for p in self.pending if math.hypot(p.x - x, p.y - y) < radius]:
            self._forget(p)

    def _fetchable(self):
        return [p for p in self.pending if p.hits >= 2 and p.tries < FETCH_TRIES]

    def _go_for_remembered(self):
        """Go back for the nearest ball that was seen and left. True if it was picked up; a ball that is not there after two tries is dropped."""
        if self.carrying:
            return False
        waiting = self._fetchable()
        if not waiting:
            return False
        p = min(waiting, key=lambda q: math.hypot(q.x - self.x, q.y - self.y))
        heading = math.degrees(math.atan2(p.y - self.y, p.x - self.x))
        dist = max(5.0, math.hypot(p.x - self.x, p.y - self.y) - SENSOR_OFFSET)
        if self._collect(heading, dist):
            self._forget(p)
            return True
        p.tries += 1
        if p.tries >= FETCH_TRIES:
            self._note("memory: the ball that was seen here is gone")
            self._forget(p)
        return False

    def _finished(self):
        """The whole floor has been looked at (as many rounds as asked for) and no ball is waiting."""
        return (not self.carrying) and (not self._fetchable()) and self.rounds_done > self.rechecks

    # -- collecting --
    def _ball_xy(self, heading, dist):
        a = math.radians(heading)
        return (self.x + (dist + SENSOR_OFFSET) * math.cos(a), self.y + (dist + SENSOR_OFFSET) * math.sin(a))

    def _is_unreachable(self, xy):
        now = self.car.time()
        return any(now - t < UNREACHABLE_S and math.hypot(xy[0] - ux, xy[1] - uy) < 50.0 for ux, uy, t in self.unreachable)

    def _collect(self, heading, dist):
        """Go to the ball (round obstacles if it is far), then the careful last bit: aim, hop, check against the ultrasonic, pick."""
        bx, by = self._ball_xy(heading, dist)
        self._face(heading)
        if dist > 80.0:
            near = (self.x + (dist - 50.0 + SENSOR_OFFSET) * math.cos(math.radians(heading)),
                    self.y + (dist - 50.0 + SENSOR_OFFSET) * math.sin(math.radians(heading)))
            status = self._goto(near, stop_within=25.0, max_steps=40)         # a trip to a ball is capped: it may be a phantom
            if status != "arrived":
                self.unreachable.append((bx, by, self.car.time()))
                self._note(f"collect: could not get to the ball ({status})")
                return False
            self._face(math.degrees(math.atan2(by - self.y, bx - self.x)))
        lost = tries = 0
        for _ in range(40):
            if self._over():
                return False
            balls = self._look().balls
            if not balls:
                lost += 1
                if lost >= 3:
                    self._note("collect: lost the ball")
                    return False
                continue
            lost = 0
            bearing, dist = min(balls, key=lambda b: b[1] + 2 * abs(b[0]))
            if abs(bearing) > 2.5:
                self.car.turn(bearing)
                continue
            us = self._range()
            tol = max(10.0, 0.25 * dist)
            if us is not None and us < dist - tol:                # something is closer than the ball: it is in the way, not the ball
                self._note("collect: something is closer than the ball")
                self._move(-12.0)
                self.unreachable.append((bx, by, self.car.time()))
                return False
            ref = us if (us is not None and abs(us - dist) <= tol) else dist
            if PICK_LO <= ref <= PICK_HI:
                self.car.pick()
                if self.car.in_jaws():
                    self.carrying = True
                    self.picks += 1
                    self._forget_near(*self._ball_xy(self.car.heading(), PICK_AIM))      # the one in the gripper is no longer waiting
                    self._note("collect: picked")
                    return True
                tries += 1
                if tries >= 3:
                    self._note("collect: pick failed 3 times")
                    return False
                self._hop(-6)
            elif ref < PICK_LO:
                self._hop(-(PICK_AIM - ref))
            else:
                cm = min(30.0, max(5.0, ref - PICK_AIM))
                if self._hop(cm) < 0.4 * cm:                      # stalled: something between us and the ball
                    self._note("collect: blocked on the way to the ball")
                    self.unreachable.append((bx, by, self.car.time()))
                    return False
        return False

    # -- going home --
    def _home_goal(self):
        """A spot HOME_STANDOFF from the bin, on the side we are coming from."""
        d = math.hypot(self.x, self.y)
        if d <= HOME_STANDOFF:
            return (self.x, self.y)
        return (self.x / d * HOME_STANDOFF, self.y / d * HOME_STANDOFF)

    def _inside(self, xy):
        """Is this place anywhere near the arena? The slack is big because the position guess drifts by up to a metre: a tight check threw
        away real balls near the walls (86% of the balls instead of 93%). It only rules out the absurd. Always True without the arena size."""
        if self.bounds is None:
            return True
        x0, y0, x1, y1 = self.bounds
        return x0 - INSIDE_SLACK <= xy[0] <= x1 + INSIDE_SLACK and y0 - INSIDE_SLACK <= xy[1] <= y1 + INSIDE_SLACK

    def _spot_ball(self):
        """Take a picture of what is ahead; True (and `spotted` set) if a ball is in it that lies inside the arena and that we have not given
        up on. (A ball seen outside the arena is a phantom: the camera sometimes sees one that is not there.)"""
        h = self.car.heading()
        seen = []
        for b, d in self._look().balls:
            if self._inside(self._ball_xy(wrap(h + b), d)) and not self._is_unreachable(self._ball_xy(wrap(h + b), d)):
                seen.append((wrap(h + b), d))
        if not seen:
            return False
        seen.sort(key=lambda bd: bd[1])
        self.spotted = seen[0]
        for h2, d2 in seen[1:]:                                     # more than one in view: the nearest now, the rest later
            self._remember(h2, d2)
        return True

    def _tag_in_view(self):
        look = self._look()
        return look.tag is not None

    def _go_home(self):
        lost = failures = 0
        for _ in range(200):
            if self._over():
                return False
            look = self._look()
            for b, d in look.balls:                                 # a ball seen on the way home: come back for it (once it has been seen twice)
                self._remember(wrap(self.car.heading() + b), d, trusted=False)
            if look.tag is not None:
                bearing, dist = look.tag
                if dist <= DROP_AT and abs(bearing) <= 10:
                    self.car.drop()
                    if not self.car.in_jaws():
                        self.carrying = False
                        self.collected += 1
                        self._note("home: delivered")
                        self._hop(-25)
                        return True
                    continue
                if abs(bearing) > 3:
                    self.car.turn(bearing)
                    continue
                hop = min(HOP_HOME, max(6.0, dist - 28.0))
                if dist < 120.0:
                    self.map.clear_blocked_near(0.0, 0.0, 110.0)    # the tag places us exactly: obstacles remembered here are ghosts
                a = math.radians(self.car.heading())
                ahead = (self.x + hop * math.cos(a), self.y + hop * math.sin(a))
                if self.map.line_free((self.x, self.y), ahead) and self._clear_ahead(hop, bin_centre=dist + SENSOR_OFFSET):
                    self._hop(hop)
                elif dist < 100.0:
                    self._sidestep((math.cos(a), math.sin(a)))      # close to the bin and something real is in the way: slide round it
                else:
                    self._goto(self._home_goal(), stop_within=25.0, max_steps=4)     # something is in the way: plan round it
            else:
                # no tag in view: drive the planned route toward where the bin should be (the tag will come into view on the way)
                status = self._goto(self._home_goal(), stop_within=30.0, until=self._tag_in_view)
                if status == "until":
                    continue
                failures = failures + 1 if status == "blocked" else 0
                if failures >= 2:
                    self._note("home: boxed in, backing out")
                    self.map.reset_obstacles()                      # the map has sealed every way out: the way we came in was open
                    self._retrace()
                    failures = 0
                    continue
                if status == "arrived":
                    self._scan()                                    # where we think the bin is, yet no tag: look all round
                    lost += 1
                else:                                               # the way was blocked: look the way home, no further
                    self._scan(around=math.degrees(math.atan2(-self.y, -self.x)), span=120.0)
                    lost += 1
                if self.tag_heading is not None:
                    self._face(self.tag_heading)
                elif lost >= 3:
                    self.car.turn(90)                               # no luck: change the view and try again
                    lost = 0
        self._note("home: gave up")
        return False

    # -- searching in lanes (the lawnmower) --
    def _new_pass(self):
        """Lay out the next set of lanes: along the longer side first, then across, then shifted half a gap, then across shifted."""
        n = self.pass_no
        self.pass_no += 1
        x0, y0, x1, y1 = self.bounds
        longer = "x" if (x1 - x0) >= (y1 - y0) else "y"
        along = longer if n % 2 == 0 else ("y" if longer == "x" else "x")
        self.route = lane_route(self.bounds, LANE_GAP, LANE_MARGIN, along, 0.0 if n < 2 else LANE_GAP / 2.0,
                                keepout=(0.0, 0.0, BIN_RADIUS + 35.0), start=(self.x, self.y))
        self.route_i = 0
        self.route_log.append((self.car.time(), list(self.route)))
        self.unreachable.clear()                                    # a fresh pass: give the balls we could not reach another chance
        self.avoid.clear()
        self._picks_at_pass_start = self.picks
        self._note(f"lanes: pass {n + 1}, {len(self.route) // 2} lanes along {along}")

    def _lane_step(self):
        """Drive to the next end of the next lane, looking ahead at every step (a ball in view stops the drive). When the route is done,
        start another pass; two passes in a row that found nothing end the search."""
        if self.route is None or self.route_i >= len(self.route):
            if self.route is not None:                              # a pass has just been finished
                if self.map.coverage() >= LANE_COVER_DONE:
                    self.rounds_done += 1
                    if self.rounds_done > self.rechecks:
                        self._note("lanes: the whole floor has been looked at")
                        self._done = True
                        return
                    self._note("lanes: looking again")
                    self.map.forget_looks(now=self.car.time())      # start the looking afresh for the extra round
                elif self.pass_no >= MAX_LANE_PASSES:
                    self._note("lanes: that is as much as the lanes can cover")
                    self._done = True
                    return
            self._new_pass()
        status = self._goto(self.route[self.route_i], stop_within=LANE_ARRIVE, until=self._spot_ball, hop_max=LANE_HOP)
        if status != "until":                                       # reached it, or could not: on to the next end either way
            self.route_i += 1

    # -- searching --
    def _explore(self):
        """Drive to the best place to look from next. False when there is nothing left worth looking at."""
        start = (self.x, self.y)
        self.map.forget_old(self.car.time())
        goal = self.map.best_look_point(start, avoid=self.avoid)
        if goal is None:
            return False
        status = self._goto(goal, stop_within=25.0, until=self._spot_ball)      # a picture every hop: stop for a ball seen on the way
        if status in ("blocked", "timeout"):
            self.avoid.append(goal)                                 # not that one again
        return True

    def run(self, max_time=900.0, give_up_after=300):
        """Search, collect, bring home. A ball seen and left is gone back for first; the search ends when the whole floor has been looked at
        (plus `rechecks` extra rounds) and no ball is waiting, or when the time is up."""
        self.deadline = max_time
        idle = 0                                                    # scans in a row that found nothing to collect
        first = True
        while self.car.time() < max_time:
            if self.carrying:
                self._go_home()
                idle = 0
                continue
            if self.spotted is not None:                            # seen from the road: go for it now
                heading, dist = self.spotted
                self.spotted = None
                if self._collect(heading, dist):
                    idle = 0
                continue
            if self._fetchable():                                   # a ball that was seen and left: before searching anywhere new
                if self._go_for_remembered():
                    idle = 0
                continue
            if self.strategy == "lanes":
                if self._done:
                    break
                self._lane_step()
                continue
            direction = None if first else self.map.look_direction(self.x, self.y)
            first = False
            if direction is None:
                balls = self._scan()                                # the first time (or nothing to aim at): look all round
            else:
                balls = self._scan(around=direction, span=180.0)    # look toward the floor that was never looked at
            candidates = [(h, d) for h, d in balls if self._inside(self._ball_xy(h, d)) and not self._is_unreachable(self._ball_xy(h, d))]
            if candidates:
                candidates.sort(key=lambda b: b[1])
                heading, dist = candidates[0]
                for h2, d2 in candidates[1:]:                       # two or more in view: pick up the nearest, remember the others
                    self._remember(h2, d2)
                if self._collect(heading, dist):
                    idle = 0
                continue
            idle += 1
            if idle >= give_up_after:
                self._note("search: nothing found for a long time, stopping")
                break
            if not self._explore():                                 # nothing reachable is worth looking at ...
                if self.map.coverage() < EXPLORE_DONE:              # ... yet much of the floor is not covered: the map has boxed the car in
                    self._note("search: boxed in by the map, forgetting the obstacles")
                    self.map.reset_obstacles()
                    self.map.forget_scans()
                    self.avoid.clear()
                    self.unreachable.clear()
                    if not self._retrace():
                        self.car.turn(90.0)
                    continue
                self.rounds_done += 1                               # ... and the floor is covered: a round is done
                if self.rounds_done > self.rechecks:
                    self._note("search: the whole floor has been looked at")
                    break
                self._note("search: looking again")                 # an extra round: a ball can be missed (or fall)
                self.map.forget_looks(now=self.car.time())
                self.map.forget_scans()
                self.map.reset_obstacles()                          # and the obstacles afresh: the old ones may be ghosts
                self.unreachable.clear()
                self.avoid.clear()
        return self.collected
