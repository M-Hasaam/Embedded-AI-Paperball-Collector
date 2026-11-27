"""The arena: a rectangle (centimetres, x to the right, y up, angles in degrees counter-clockwise from +x), the bin, paper balls and
obstacles. An obstacle is permanent or temporary (it appears and vanishes at given times) and may be 'thin' (a chair leg: the
ultrasonic often misses it)."""
import math
from dataclasses import dataclass

INF = float("inf")


@dataclass
class Obstacle:
    x1: float
    y1: float
    x2: float
    y2: float
    appear: float = 0.0              # seconds: it is there from this time ...
    vanish: float = INF              # ... until this time
    thin: bool = False
    label: str = ""                  # what it is, for the picture ("Table", "Bag" ...)

    def active(self, t):
        return self.appear <= t < self.vanish

    @property
    def permanent(self):
        return self.appear <= 0 and self.vanish == INF


@dataclass
class Ball:
    x: float
    y: float
    radius: float = 3.0
    picked: bool = False
    created_at: float = 0.0            # a ball that fell out of the gripper appears at this time
    picked_at: float = None            # when the gripper took it off the floor
    delivered_at: float = None         # when it went into the bin


def _ray_box(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distance along the ray to the box, or None. A ray that starts inside gets 0."""
    tmin, tmax = 0.0, INF
    for o, d, lo, hi in ((ox, dx, x1, x2), (oy, dy, y1, y2)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                return None
    return tmin


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    if c < 0:
        return 0.0
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t >= 0 else None


class World:
    def __init__(self, width, height, bin_xy, obstacles=(), balls=(), bin_radius=15.0):
        self.width, self.height = float(width), float(height)
        self.bin_xy, self.bin_radius = tuple(bin_xy), float(bin_radius)
        self.obstacles, self.balls = list(obstacles), list(balls)

    def active_obstacles(self, t):
        return [o for o in self.obstacles if o.active(t)]

    def blocked(self, x, y, r, t):
        """True when a round car of radius r centred here would touch a wall, the bin or an obstacle that exists at time t."""
        if x - r < 0 or y - r < 0 or x + r > self.width or y + r > self.height:
            return True
        if math.hypot(x - self.bin_xy[0], y - self.bin_xy[1]) < r + self.bin_radius:
            return True
        for o in self.active_obstacles(t):
            nx, ny = min(max(x, o.x1), o.x2), min(max(y, o.y1), o.y2)
            if math.hypot(x - nx, y - ny) < r:
                return True
        return False

    def cast(self, ox, oy, angle_deg, t, max_cm=1000.0, skip=()):
        """The nearest thing along a ray: (distance, kind) with kind wall / obstacle / thin / bin / ball, or (None, None) beyond max_cm.
        Kinds listed in `skip` are not seen."""
        a = math.radians(angle_deg)
        dx, dy = math.cos(a), math.sin(a)
        best, kind = INF, None
        ts = []
        if dx > 1e-12:
            ts.append((self.width - ox) / dx)
        elif dx < -1e-12:
            ts.append(-ox / dx)
        if dy > 1e-12:
            ts.append((self.height - oy) / dy)
        elif dy < -1e-12:
            ts.append(-oy / dy)
        if ts and "wall" not in skip:
            best, kind = max(0.0, min(ts)), "wall"
        for o in self.active_obstacles(t):
            k = "thin" if o.thin else "obstacle"
            if k in skip:
                continue
            d = _ray_box(ox, oy, dx, dy, o.x1, o.y1, o.x2, o.y2)
            if d is not None and d < best:
                best, kind = d, k
        if "bin" not in skip:
            d = _ray_circle(ox, oy, dx, dy, self.bin_xy[0], self.bin_xy[1], self.bin_radius)
            if d is not None and d < best:
                best, kind = d, "bin"
        if "ball" not in skip:
            for b in self.balls:
                if b.picked:
                    continue
                d = _ray_circle(ox, oy, dx, dy, b.x, b.y, b.radius)
                if d is not None and d < best:
                    best, kind = d, "ball"
        if best > max_cm or kind is None:
            return None, None
        return best, kind
