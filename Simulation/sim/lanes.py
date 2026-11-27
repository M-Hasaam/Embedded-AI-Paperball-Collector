"""The route for the lawnmower search: straight lanes across the arena, a lane-gap apart, driven back and forth. Pure geometry, in the
brain's frame (the bin is the origin). The camera sees a narrow wedge straight ahead, so lanes about 70 cm apart cover the floor."""
import math


def lane_route(bounds, gap=70.0, margin=25.0, along=None, offset=0.0, keepout=None, start=None):
    """Waypoints for driving the arena in lanes. Points come in pairs (the start and the end of a lane); between pairs the car steps over
    to the next lane.

    bounds    (x0, y0, x1, y1) of the arena
    gap       distance between lanes (the last lane is added at the far edge so the whole width is covered)
    margin    how far the lanes keep from the walls
    along     'x' or 'y': the direction the lanes run (default: along the longer side)
    offset    shifts the lanes over, so a second pass runs between the lanes of the first
    keepout   (x, y, radius): a place not to drive through (the bin); a lane it cuts keeps its longer part
    start     (x, y) of the car: the route begins at the lane end nearest to it
    """
    x0, y0, x1, y1 = bounds
    if along is None:
        along = "x" if (x1 - x0) >= (y1 - y0) else "y"
    lx0, lx1, ly0, ly1 = x0 + margin, x1 - margin, y0 + margin, y1 - margin
    lo, hi, a0, a1 = (ly0, ly1, lx0, lx1) if along == "x" else (lx0, lx1, ly0, ly1)

    def point(a, c):
        return (a, c) if along == "x" else (c, a)

    coords, c = [], lo + offset
    while c <= hi + 1e-9:
        coords.append(c)
        c += gap
    if offset == 0.0 and coords and coords[-1] < hi - 1e-9:
        coords.append(hi)                                           # the edge lane, so no strip wider than a gap is left

    def span(c):
        s, e = a0, a1
        if keepout is not None:
            kx, ky, kr = keepout
            across, centre = (c - ky, kx) if along == "x" else (c - kx, ky)
            if abs(across) < kr:
                half = math.sqrt(kr * kr - across * across)
                before, after = (s, centre - half), (centre + half, e)
                s, e = max((before, after), key=lambda p: p[1] - p[0])
        return s, e

    lanes = [(c, *span(c)) for c in coords]
    lanes = [ln for ln in lanes if ln[2] - ln[1] >= 20.0]
    if not lanes:
        return []
    forward = True
    if start is not None:
        sx, sy = start
        cross, pos = (sy, sx) if along == "x" else (sx, sy)
        if abs(cross - lanes[-1][0]) < abs(cross - lanes[0][0]):
            lanes.reverse()
        forward = abs(pos - lanes[0][1]) <= abs(pos - lanes[0][2])
    route = []
    for c, s, e in lanes:
        a, b = (s, e) if forward else (e, s)
        route += [point(a, c), point(b, c)]
        forward = not forward
    return route
