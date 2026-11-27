"""The car's own map: a coarse grid of what it has learned, in its own coordinates (the bin is the origin).

  free     cells an ultrasonic reading passed through
  blocked  cells where a reading hit something, or where the car bumped (a later clear reading removes the first kind: a box that was
           taken away; a bump is remembered)
  seen     cells the camera has looked at well enough that a ball there would have been noticed: a look from close counts fully, one
           from far only partly (the camera misses small, far balls), so far floor needs two looks (`evidence` keeps the running total)
  arena    the rectangle the car was told about: every cell of it is on the to-do list until it is seen; `coverage()` says how much is done

It plans paths with Dijkstra's shortest-path search around the blocked cells (which are grown by one cell for the car's size), pulls
the path straight where it can, and picks the next place to stop and look: where the most floor that was never looked at can be seen
from, balanced against how far away it is. No map of the arena is given: it only knows what it has measured.
"""
import heapq
import math

CELL = 20.0
RAY_STEP = 10.0
US_RANGE = 200.0
CAM_HALF_FOV = 18.0
UNKNOWN_COST = 1.5                  # a cell never seen costs this many times a known one when planning a path (3 gave dog-legs round known floor)
LOOK_NEAR_CM = 110.0                # a look from this close or closer counts fully ... (a little over a metre)
LOOK_FAR_CM = 220.0                 # ... and one from this far counts LOOK_FAR_WEIGHT
LOOK_FAR_WEIGHT = 0.5
DONE_EVIDENCE = 1.0                 # a cell is 'seen' once its looks add up to this
ARENA_STOP_INSET = 20.0             # places to stop and look from keep this far from the walls
FRONTIER_BONUS = 8                  # an edge of the known counts as this many unlooked-at cells


def look_weight(distance_cm):
    """How much one camera look at floor this far away counts toward 'seen' (1 = conclusive)."""
    if distance_cm <= LOOK_NEAR_CM:
        return 1.0
    if distance_cm >= LOOK_FAR_CM:
        return LOOK_FAR_WEIGHT
    return 1.0 + (LOOK_FAR_WEIGHT - 1.0) * (distance_cm - LOOK_NEAR_CM) / (LOOK_FAR_CM - LOOK_NEAR_CM)


class GridMap:
    def __init__(self):
        self.free, self.blocked, self.seen, self.hard = set(), set(), set(), set()
        self.scan_pts = []                                 # where the car has stopped and looked all round
        self.bumps = []                                    # where the car ran into something
        self.blocked_at = {}                               # when each blocked cell was last seen (only when the caller gives the time)
        self.unknown_cost = UNKNOWN_COST                   # what a cell never seen costs a path, against 1 for a known one
        self.evidence = {}                                 # how much each cell has been looked at (see look_weight)
        self.seen_events = []                              # (time, cell) each time a cell became seen (when the caller gives the time)
        self.seen_resets = []                              # times the looks were all forgotten
        self.arena, self.arena_cells = None, set()
        self.blocked_snapshots = []                        # (time, the obstacle cells then): for the picture
        self.ends = set()                                  # where a reading found nothing out to its range: more floor lies beyond
        self._inflated = None

    # -- cells --
    @staticmethod
    def cell(x, y):
        return (math.floor(x / CELL), math.floor(y / CELL))

    @staticmethod
    def center(c):
        return ((c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL)

    # -- what the sensors tell us --
    def ray(self, x, y, heading, r, max_range=US_RANGE, now=None):
        """An ultrasonic reading `r` cm along `heading` (None = no echo within max_range): the floor up to it is free, what it hit is not."""
        a = math.radians(heading)
        dx, dy = math.cos(a), math.sin(a)
        reach = max_range if r is None else r
        d = 0.0
        while d < reach:
            c = self.cell(x + d * dx, y + d * dy)
            self.free.add(c)
            if c not in self.hard:
                self.blocked.discard(c)
            d += RAY_STEP
        if r is not None and r < max_range - 5:
            hit = self.cell(x + (r + 4.0) * dx, y + (r + 4.0) * dy)
            self.blocked.add(hit)
            if now is not None:
                self.blocked_at[hit] = now
        else:
            self.ends.add(self.cell(x + (max_range - 5.0) * dx, y + (max_range - 5.0) * dy))
        self._inflated = None

    def block(self, x, y, now=None):
        """The car ran into something here: remembered even when a later reading misses it (a chair leg)."""
        c = self.cell(x, y)
        again = any(math.hypot(x - bx, y - by) < 45.0 for bx, by in self.bumps)
        self.bumps.append((x, y))
        cells = [(c[0] + i, c[1] + j) for i in (-1, 0, 1) for j in (-1, 0, 1)] if again else [c]     # a second bump here: seal the neighbourhood
        for cc in cells:
            self.blocked.add(cc)
            self.hard.add(cc)
            if now is not None:
                self.blocked_at[cc] = now
        self._inflated = None

    def wedge(self, x, y, heading, limit, now=None):
        """A camera picture from here: the floor in view, from 30 cm out to `limit`. Each cell in it gets evidence of having been looked at,
        more from close than from far (see look_weight); once it has enough it is 'seen'. `now` (the time) feeds the history."""
        nearest = {}
        for d in range(30, int(limit) + 1, 12):
            for off in (-18, -12, -6, 0, 6, 12, 18):
                a = math.radians(heading + off)
                c = self.cell(x + d * math.cos(a), y + d * math.sin(a))
                if d < nearest.get(c, math.inf):
                    nearest[c] = d
        for c, d in nearest.items():
            self.evidence[c] = self.evidence.get(c, 0.0) + look_weight(d)
            if self.evidence[c] >= DONE_EVIDENCE - 1e-9 and c not in self.seen:
                self.seen.add(c)
                if now is not None:
                    self.seen_events.append((now, c))
            if c not in self.blocked and d <= limit - 15:
                self.free.add(c)                                    # floor that was in the picture, in front of whatever blocks the view

    def forget_looks(self, now=None):
        """Start the looking afresh (a ball can be missed, or fall): everything is to do again."""
        self.seen.clear()
        self.evidence.clear()
        if now is not None:
            self.seen_resets.append(now)

    def seen_at(self, t):
        """The cells that were 'seen' at time t (in this round of looking), for the picture."""
        last = max([r for r in self.seen_resets if r <= t], default=-math.inf)
        return {c for tt, c in self.seen_events if last < tt <= t}

    # -- the arena the car was told about --
    def set_arena(self, x0, y0, x1, y1):
        """Tell the map the rectangle it is working in: all of it is to be looked at."""
        self.arena = (x0, y0, x1, y1)
        self.arena_cells = {(cx, cy) for cx in range(math.floor(x0 / CELL), math.ceil(x1 / CELL))
                            for cy in range(math.floor(y0 / CELL), math.ceil(y1 / CELL))
                            if x0 < (cx + 0.5) * CELL < x1 and y0 < (cy + 0.5) * CELL < y1}          # a cell counts if its centre is on the floor

    def coverage(self):
        """How much of the floor has been looked at, 0..1: of the arena (floor under known obstacles does not count), or, with no arena
        given, of the floor the sensors have found."""
        if self.arena:
            todo = self.arena_cells - self.blocked
            return len(todo & self.seen) / len(todo) if todo else 1.0
        floor = self.free - self.blocked
        return len(floor & self.seen) / len(floor) if floor else 0.0

    def _unseen_cells(self):
        """What is still to be looked at: the arena's cells if it is known, else the known floor."""
        pool = self.arena_cells if self.arena else self.free
        return [c for c in pool if c not in self.seen and c not in self.blocked]

    def clear_blocked_near(self, x, y, radius):
        """Drop obstacle cells within `radius` of a point: the place we are heading for (the bin) is not in our way."""
        gone = [c for c in self.blocked if math.hypot(self.center(c)[0] - x, self.center(c)[1] - y) <= radius]
        for c in gone:
            self.blocked.discard(c)
            self.hard.discard(c)
            self.blocked_at.pop(c, None)
        if gone:
            self._inflated = None

    def reset_obstacles(self):
        """Forget every obstacle (the map has boxed the car in, or its ghosts have piled up): fresh readings and bumps will re-learn them.
        What has been looked at and the known floor stay."""
        self.blocked, self.hard, self.blocked_at, self.bumps = set(), set(), {}, []
        self._inflated = None

    def forget_old(self, now, max_age=90.0, bump_age=240.0):
        """The position guess drifts, so an obstacle that has not been seen again for a while is probably in the wrong place on the map:
        forget it. A bump is trusted longer than a reading. Cells without a time stamp are never forgotten."""
        gone = [c for c, t in self.blocked_at.items() if now - t > (bump_age if c in self.hard else max_age)]
        for c in gone:
            self.blocked.discard(c)
            self.hard.discard(c)
            del self.blocked_at[c]
        if gone:
            self._inflated = None
        return len(gone)

    def record_blocked(self, now, every=3.0):
        """Keep a snapshot of the obstacles the car believes in, when they have changed (and at most once in `every` seconds)."""
        current = frozenset(self.blocked)
        if self.blocked_snapshots:
            last_t, last = self.blocked_snapshots[-1]
            if current == last or now - last_t < every:
                return
        elif not current:
            return
        self.blocked_snapshots.append((now, current))

    def blocked_cells_at(self, t):
        """The obstacle cells the car believed in at time t."""
        found = set()
        for tt, cells in self.blocked_snapshots:
            if tt > t:
                break
            found = cells
        return set(found)

    def scanned_at(self, x, y, radius=100.0):
        """The car has looked all round from here: edges of the known within `radius` have been explored."""
        self.ends = {c for c in self.ends if math.hypot(self.center(c)[0] - x, self.center(c)[1] - y) > radius}
        self.scan_pts.append((x, y))

    def forget_scans(self):
        self.scan_pts = []

    # -- where the car may go --
    def _blocked_grown(self):
        if self._inflated is None:
            grown = set()
            for bx, by in self.blocked:
                for ix, iy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):      # the cell and its four neighbours: the car's size
                    grown.add((bx + ix, by + iy))
            self._inflated = grown
        return self._inflated

    def in_the_way(self, x, y):
        return self.cell(x, y) in self._blocked_grown()

    def line_free(self, a, b, near=None):
        """Is the straight line a -> b clear of (grown) blocked cells? Cells close to `near` (where the car stands) are let through."""
        grown = self._blocked_grown()
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 8.0))
        start = self.cell(*(near or a))
        for i in range(n + 1):
            c = self.cell(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
            if c in self.blocked:
                return False                                        # never through something known to be there
            if c in grown and max(abs(c[0] - start[0]), abs(c[1] - start[1])) > 2:
                return False                                        # (the safety margin may be crossed to leave a tight spot)
        return True

    def _passable(self, c, start):
        if c in self.blocked:
            return False                                            # never through something known to be there
        return c not in self._blocked_grown() or max(abs(c[0] - start[0]), abs(c[1] - start[1])) <= 2     # the margin may be crossed near the start

    def _dijkstra(self, start_c, goal_c=None, bounds=None):
        dist, prev = {start_c: 0.0}, {}
        heap = [(0.0, start_c)]
        while heap:
            d, c = heapq.heappop(heap)
            if d > dist.get(c, math.inf):
                continue
            if c == goal_c:
                break
            for ix, iy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                n = (c[0] + ix, c[1] + iy)
                if bounds and not (bounds[0] <= n[0] <= bounds[2] and bounds[1] <= n[1] <= bounds[3]):
                    continue
                if n != goal_c and not self._passable(n, start_c):
                    continue
                step = 1.414 if ix and iy else 1.0
                if n not in self.free and n not in self.blocked:
                    step *= self.unknown_cost                       # never been there: possible, but prefer what is known
                nd = d + step
                if nd < dist.get(n, math.inf):
                    dist[n], prev[n] = nd, c
                    heapq.heappush(heap, (nd, n))
        return dist, prev

    def _bounds(self, *points):
        cells = list(self.free | self.blocked | self.arena_cells) + [self.cell(*p) for p in points]
        xs, ys = [c[0] for c in cells], [c[1] for c in cells]
        return (min(xs) - 6, min(ys) - 6, max(xs) + 6, max(ys) + 6)

    def plan(self, start, goal):
        """The shortest path from start to goal (x, y in cm) round the blocked cells, as a list of points; None when there is none."""
        s, g = self.cell(*start), self.cell(*goal)
        dist, prev = self._dijkstra(s, g, self._bounds(start, goal))
        if g not in dist:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(prev[cells[-1]])
        cells.reverse()
        pts = [self.center(c) for c in cells]
        pts[0], pts[-1] = tuple(start), tuple(goal)
        return pts

    def shortcut(self, path):
        """Pull a path straight: drop every point the car can skip because the line past it is clear."""
        if len(path) <= 2:
            return list(path)
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_free(path[i], path[j], near=path[0]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    # -- where to look next --
    def best_look_point(self, start, min_dist=0.0, radius=150.0, min_gain=8, avoid=(), scan_gap=90.0):
        """The place to stop and look round next: the cell from which the most never-looked-at free floor lies within `radius`,
        weighed against the path length to it. None when there is (almost) nothing left to look at."""
        unseen = [self.center(c) for c in self._unseen_cells()]
        edges = [] if self.arena else [self.center(c) for c in self.ends]            # with the arena known its edge is the walls, not a frontier
        if len(unseen) < min_gain and not edges:
            return None
        s = self.cell(*start)
        dist, _ = self._dijkstra(s, None, self._bounds(start))
        best, r2 = None, radius * radius
        for c, d in dist.items():
            if c in self._blocked_grown() or (c[0] % 2) or (c[1] % 2):
                continue
            if self.arena:
                if c not in self.arena_cells:
                    continue
                px, py = self.center(c)
                if not (self.arena[0] + ARENA_STOP_INSET <= px <= self.arena[2] - ARENA_STOP_INSET
                        and self.arena[1] + ARENA_STOP_INSET <= py <= self.arena[3] - ARENA_STOP_INSET):
                    continue                                         # too close to a wall to stop and spin
            elif c not in self.free:
                continue
            cm = d * CELL
            if cm < min_dist:
                continue
            cx, cy = self.center(c)
            if any(math.hypot(cx - ax, cy - ay) < 50.0 for ax, ay in avoid):
                continue
            if any(math.hypot(cx - px, cy - py) < scan_gap for px, py in self.scan_pts):
                continue                                            # we have just looked all round from near there
            gain = sum(1 for ux, uy in unseen if (ux - cx) ** 2 + (uy - cy) ** 2 <= r2)
            gain += FRONTIER_BONUS * sum(1 for ex, ey in edges if (ex - cx) ** 2 + (ey - cy) ** 2 <= 100.0 ** 2)
            if gain < min_gain:
                continue
            score = gain / (1.0 + cm / 150.0)
            if best is None or score > best[0]:
                best = (score, (cx, cy))
        return best[1] if best else None

    def look_direction(self, x, y, radius=160.0):
        """Which way to turn the camera from (x, y): toward the middle of the floor within `radius` that was never looked at (and the
        edges of the known). None when there is none."""
        sx = sy = w = 0.0
        for c in self._unseen_cells():
            cx, cy = self.center(c)
            if math.hypot(cx - x, cy - y) <= radius:
                sx, sy, w = sx + (cx - x), sy + (cy - y), w + 1.0
        for c in ([] if self.arena else self.ends):
            cx, cy = self.center(c)
            if math.hypot(cx - x, cy - y) <= radius + 60.0:
                sx, sy, w = sx + FRONTIER_BONUS * (cx - x), sy + FRONTIER_BONUS * (cy - y), w + FRONTIER_BONUS
        if w == 0.0 or (abs(sx) < 1e-6 and abs(sy) < 1e-6):
            return None
        return math.degrees(math.atan2(sy, sx))
