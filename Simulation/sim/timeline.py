"""A finished run as a story: what is true at any moment (where the car is, which balls are on the floor, which obstacles are there) for the
playback, a list of what happened and when, and the result in plain words."""
import bisect
import math
from collections import namedtuple

State = namedtuple("State", "t x y h carrying floor_balls delivered bumps temp_active guess arm route seen coverage pending obstacles")
Arm = namedtuple("Arm", "kind progress ok target")      # an arm move under way: pick or drop, 0..1 through it, did it work, where it reaches

MAX_BUMPS_LISTED = 8

# how a search is named in the results
STRATEGY_NAMES = {"map": "map-guided scanning (stop where the most unlooked-at floor can be seen, look all round, plan the next stop)",
                  "lanes": "lanes (lawnmower: straight lanes across the arena, looking ahead at every step)"}


def clock(seconds):
    s = int(max(0.0, seconds))
    return f"{s // 60}:{s % 60:02d}"


def _lerp_angle(a, b, k):
    d = (b - a + 180.0) % 360.0 - 180.0
    return a + d * k


def _index(result):
    """Sorted time lists for fast lookups (built once per result)."""
    idx = getattr(result, "_timeline_index", None)
    if idx is None:
        car, brain = result.car, result.brain
        idx = {
            "path_t": [p[2] for p in car.path],
            "guess_t": [g[0] for g in brain.est_log],
            "bump_t": sorted(e[1] for e in car.events if e[0] == "bump"),
            # the car holds a ball from each pick until the next delivery or drop
            "carry": _carry_intervals(car.events),
            "grip_t": [g.start for g in car.grips],
            "route_t": [t for t, _ in getattr(result.brain, "route_log", [])],
            "cover_t": [t for t, _ in getattr(result.brain, "coverage_log", [])],
            "ball_t": [e[0] for e in getattr(result.brain, "ball_events", [])],
        }
        result._timeline_index = idx
    return idx


def _carry_intervals(events):
    out, start = [], None
    for kind, t, *_ in events:
        if kind == "pick":
            start = t
        elif kind in ("deliver", "drop") and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, math.inf))
    return out


def state_at(result, t):
    """Everything the picture needs to show at time t (seconds into the run)."""
    car, world = result.car, result.world
    idx = _index(result)
    t = min(max(t, 0.0), car.path[-1][2])
    times, path = idx["path_t"], car.path
    i = bisect.bisect_right(times, t) - 1
    i = max(0, min(i, len(path) - 1))
    x, y, t0, h = path[i]
    if i + 1 < len(path) and path[i + 1][2] > t0:
        x1, y1, t1, h1 = path[i + 1]
        k = (t - t0) / (t1 - t0)
        x, y, h = x + (x1 - x) * k, y + (y1 - y) * k, _lerp_angle(h, h1, k)
    carrying = any(a <= t < b for a, b in idx["carry"])
    floor = [(b.x, b.y) for b in world.balls if b.created_at <= t and (b.picked_at is None or b.picked_at > t)]
    delivered = sum(1 for b in world.balls if b.delivered_at is not None and b.delivered_at <= t)
    bumps = bisect.bisect_right(idx["bump_t"], t)
    temp = [o for o in world.obstacles if not o.permanent and o.active(t)]
    log = result.brain.est_log
    gi = bisect.bisect_right(idx["guess_t"], t) - 1
    guess = None
    if gi >= 0:
        guess = (log[gi][1] + world.bin_xy[0], log[gi][2] + world.bin_xy[1])      # the brain's frame has the bin as its origin
    arm = None
    gi2 = bisect.bisect_right(idx["grip_t"], t) - 1
    if gi2 >= 0:
        g = car.grips[gi2]
        if g.start <= t < g.end:
            arm = Arm(g.kind, (t - g.start) / (g.end - g.start), g.ok, g.target)
    route = None
    ri = bisect.bisect_right(idx["route_t"], t) - 1
    if ri >= 0:
        route = [(px + world.bin_xy[0], py + world.bin_xy[1]) for px, py in result.brain.route_log[ri][1]]       # the brain's frame has the bin as its origin
    from sim.gridmap import CELL
    bx, by = world.bin_xy
    seen = [(cx * CELL + bx, cy * CELL + by) for cx, cy in result.brain.map.seen_at(t)]      # lower-left corners of 20 cm cells, in the arena's coordinates
    seen = [(x, y) for x, y in seen if x + CELL > 0 and y + CELL > 0 and x < world.width and y < world.height]     # (the camera can mark a little floor beyond a wall)
    ci = bisect.bisect_right(idx["cover_t"], t) - 1
    coverage = result.brain.coverage_log[ci][1] if ci >= 0 else 0.0
    waiting = {}
    last = bisect.bisect_right(idx["ball_t"], t)
    for _, action, ball_id, px, py in result.brain.ball_events[:last]:
        if action == "remove":
            waiting.pop(ball_id, None)
        else:
            waiting[ball_id] = (px + bx, py + by)
    pending = list(waiting.values())
    obstacles = [(cx * CELL + bx, cy * CELL + by) for cx, cy in result.brain.map.blocked_cells_at(t)]
    obstacles = [(x, y) for x, y in obstacles if x + CELL > 0 and y + CELL > 0 and x < world.width and y < world.height]
    return State(t, x, y, h, carrying, floor, delivered, bumps, temp, guess, arm, route, seen, coverage, pending, obstacles)


def events_of(result):
    """[(seconds, text)] in time order: arm moves (from when they start), bumps, and obstacles arriving and leaving."""
    out, picks, bumps, current = [], 0, 0, 0
    for g in result.car.grips:
        if g.kind == "pick":
            if g.ok:
                picks += 1
                current = picks
                out.append((g.start, f"Picking up ball {current}"))
            else:
                out.append((g.start, "Reaching for a ball and missing it"))
        elif g.ok:
            out.append((g.start, f"Dropping ball {current} in the bin"))
        else:
            out.append((g.start, f"Ball {current} fell on the floor (missed the bin)"))
    for kind, t, x, y, _ in result.car.events:
        if kind == "bump":
            bumps += 1
            if bumps <= MAX_BUMPS_LISTED:
                out.append((t, "Bumped into something"))
            elif bumps == MAX_BUMPS_LISTED + 1:
                out.append((t, "...and more bumps (not listed one by one)"))
    for o in result.world.obstacles:
        if o.permanent:
            continue
        name = o.label or "An obstacle"
        if 0 < o.appear <= result.seconds:
            out.append((o.appear, f"{name} appears"))
        if o.vanish <= result.seconds:
            out.append((o.vanish, f"{name} is taken away"))
    return sorted(out, key=lambda e: e[0])


def plain_summary(result):
    n, d = result.balls, result.delivered
    lines = [f"Collected {d} of {n} balls in {clock(result.seconds)}.", f"Search used: {STRATEGY_NAMES.get(result.strategy, result.strategy)}."]
    if result.bumps == 0:
        lines.append("It never bumped into anything.")
    else:
        lines.append(f"It bumped into something {result.bumps} time{'s' if result.bumps != 1 else ''}.")
    if d < n:
        left = n - d
        lines.append(f"{left} ball{'s' if left != 1 else ''} left on the floor (or in the gripper) when time ran out.")
    eff = result.return_efficiency
    if eff == eff:
        lines.append(f"Carrying a ball to the bin, it walked {eff:.1f} times the straight-line distance (1.0 = perfectly straight).")
    spent, total = result.car.spent, max(result.seconds, 1.0)
    lines.append(f"Where the time went: {100 * spent['drive'] / total:.0f}% driving, {100 * spent['turn'] / total:.0f}% turning on the spot, "
                 f"{100 * (spent['look'] + spent['ultrasonic']) / total:.0f}% looking and measuring, {100 * spent['grip'] / total:.0f}% with the gripper.")
    lines.append(f"At the end it thought it was {result.drift_cm:.0f} cm from where it really was.")
    return lines


def plain_batch(results):
    n = len(results)
    got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
    bumps = sum(r.bumps for r in results)
    mean_time = sum(r.seconds for r in results) / n
    return [f"{n} layouts: collected {got} of {total} balls ({100 * got / max(total, 1):.0f}%).",
            f"On average a layout took {clock(mean_time)} and the car bumped into something {bumps / n:.1f} times."]
