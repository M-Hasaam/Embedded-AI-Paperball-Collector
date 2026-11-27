"""Pictures of a run. `render_frame(result, t)` is the arena at one moment (for the playback): the car with its sensor cone, the balls on the
floor, the obstacles that are there at that moment (and the ones that will arrive, as dotted outlines), a fading trail and a small
status chip. `render_frame(result, None)` is the whole run at once: the car's route as a soft line and the balls numbered in the order they
were picked up. Everything is drawn at twice the size and shrunk, so the edges are smooth."""
import math
from collections import namedtuple
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from sim.car import CAM_HALF_FOV, CAM_RANGE, GRIP_CLOSE, GRIP_RELEASE, SENSOR_OFFSET
from sim.timeline import clock, state_at

SCALE = 2.0                      # picture pixels per cm
MARGIN = 20
SS = 2                           # draw this much bigger, then shrink
CLOSEUP_PX = 240                 # the arm close-up is a square this many pixels across
CLOSEUP_HALF_CM = 30.0           # and shows this many cm either side of its centre

BG = (15, 20, 26)
FLOOR = (27, 34, 43)
WALL = (58, 70, 84)
GRID = (255, 255, 255, 12)
OBSTACLE = (71, 85, 105)
OBSTACLE_EDGE = (110, 126, 148)
TEMP = (180, 83, 9)
TEMP_EDGE = (251, 191, 36)
LEG = (234, 179, 8)
BIN = (37, 99, 235)
BIN_EDGE = (147, 197, 253)
BALL = (241, 245, 249)
BALL_EDGE = (148, 163, 184)
TRAIL = (96, 165, 250)
CAR = (250, 204, 21)
PENDING_BLUE = (59, 130, 246)      # a ball the car has seen and will come back for
ROUTE_COLOUR = (167, 139, 250)     # the lane route the car is following (violet)
CAMERA_VIEW = (56, 189, 248)      # the camera's field of view (sky blue); the ultrasonic beam stays the car's yellow
CAR_EDGE = (120, 83, 9)
GOOD = (52, 211, 153)
BAD = (248, 113, 113)
TEXT = (226, 232, 240)
MUTED = (148, 163, 184)

_FONTS = {}

# ---- the arm ----
ARM_FOLDED = 6.0              # cm from the car's centre to the gripper with the arm folded
JAW_OPEN = 38.0               # degrees each jaw swings out from straight ahead when open
JAW_CLOSED = 4.0
ARM_LINK = (226, 232, 240)
JAW = (251, 146, 60)
ArmPose = namedtuple("ArmPose", "length jaw ball fall")     # ball: 'floor' (not touched yet), 'gripper', 'falling' (into the bin), 'none'


def _smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def _part(p, a, b):
    return min(1.0, max(0.0, (p - a) / (b - a)))


def arm_pose(arm, reach):
    """Where the arm is at this moment of a pick or a release: how far the gripper is from the car's centre (cm), how wide the jaws are
    open, and where the ball is. `reach` is how far out the arm goes (cm)."""
    p, f = arm.progress, ARM_FOLDED
    if arm.kind == "pick":
        if p < 0.35:                                                 # reach out, jaws open
            return ArmPose(f + (reach - f) * _smooth(p / 0.35), JAW_OPEN, "floor", 0.0)
        if p < GRIP_CLOSE:                                           # the jaws close on the ball
            return ArmPose(reach, JAW_OPEN + (JAW_CLOSED - JAW_OPEN) * _part(p, 0.35, GRIP_CLOSE), "floor", 0.0)
        ball = "gripper" if arm.ok else "none"                       # ... and lift it back (or come back empty)
        return ArmPose(reach + (f - reach) * _smooth(_part(p, GRIP_CLOSE, 1.0)), JAW_CLOSED, ball, 0.0)
    if p < 0.4:                                                      # a release: reach out over the bin with the ball
        return ArmPose(f + (reach - f) * _smooth(p / 0.4), JAW_CLOSED, "gripper", 0.0)
    if p < 0.6:                                                      # the jaws open
        jaw = JAW_CLOSED + (JAW_OPEN - JAW_CLOSED) * _part(p, 0.4, 0.6)
        if p < GRIP_RELEASE:
            return ArmPose(reach, jaw, "gripper", 0.0)
        return _released(arm, reach, jaw, p)
    jaw = JAW_OPEN + (JAW_CLOSED - JAW_OPEN) * _part(p, 0.6, 1.0)    # fold back
    pose = _released(arm, reach + (f - reach) * _smooth(_part(p, 0.6, 1.0)), jaw, p)
    return pose


def _released(arm, length, jaw, p):
    if arm.ok and p < 0.8:
        return ArmPose(length, jaw, "falling", (p - GRIP_RELEASE) / (0.8 - GRIP_RELEASE))
    return ArmPose(length, jaw, "none", 0.0)


def arm_caption(arm):
    """What the arm is doing, in plain words."""
    if arm is None:
        return ""
    p = arm.progress
    if arm.kind == "pick":
        if p < 0.35:
            return "Arm: reaching out to the ball"
        if p < GRIP_CLOSE:
            return "Arm: closing the gripper on the ball"
        return "Arm: lifting the ball" if arm.ok else "Arm: missed the ball, pulling back"
    if p < 0.4:
        return "Arm: reaching over the bin"
    if p < 0.8:
        return "Arm: letting go of the ball" if arm.ok else "Arm: letting go (the ball misses the bin)"
    return "Arm: folding back"


def _font(px, bold=False):
    key = (px, bold)
    if key not in _FONTS:
        names = ("segoeuib.ttf", "arialbd.ttf") if bold else ("segoeui.ttf", "arial.ttf")
        font = None
        for name in names:
            try:
                font = ImageFont.truetype(name, int(px))
                break
            except OSError:
                continue
        _FONTS[key] = font or ImageFont.load_default(int(px))
    return _FONTS[key]


def _draw_arm(d, s, centre, fwd, side, k, P, world):
    """The arm in front of the car: two links with a kink at the elbow while it is folded, a gripper with two jaws, the ball in it, and a
    ring on what it is reaching for. Folded and closed when nothing is happening."""
    (cx, cy), (fx, fy), (rx, ry) = centre, fwd, side

    def at(ahead, sideways=0.0):
        return (cx + (fx * ahead + rx * sideways) * k, cy + (fy * ahead + ry * sideways) * k)

    arm = s.arm
    reach = ARM_FOLDED + 12.0                                          # (what the kink at the elbow is measured against when idle)
    if arm is not None:
        tx, ty = arm.target
        dist = math.hypot(tx - s.x, ty - s.y)
        reach = max(ARM_FOLDED + 6.0, min(34.0, dist - 6.0 if arm.kind == "pick" else dist - world.bin_radius - 2.0))
        pose = arm_pose(arm, reach)
        ring_on = (arm.kind == "pick" and arm.progress < GRIP_CLOSE) or (arm.kind == "drop" and arm.progress < 0.8)
        if ring_on:                                                  # what it is reaching for
            ex, ey = P(tx, ty)
            r = (8.0 if arm.kind == "pick" else world.bin_radius + 5.0) * k * (1.0 + 0.12 * math.sin(arm.progress * 25.0))
            d.ellipse([ex - r, ey - r, ex + r, ey + r], outline=GOOD + (190,), width=2 * SS)
    else:
        pose = ArmPose(ARM_FOLDED, JAW_CLOSED, "gripper" if s.carrying else "none", 0.0)
    length = pose.length
    kink = 7.0 * (1.0 - _part(length, ARM_FOLDED, reach))
    shoulder, elbow, tip = at(2.0), at((2.0 + length) / 2.0, kink), at(length)
    d.line([shoulder, elbow, tip], fill=ARM_LINK + (255,), width=3 * SS, joint="curve")
    for joint in (shoulder, elbow):
        d.ellipse([joint[0] - 2 * k, joint[1] - 2 * k, joint[0] + 2 * k, joint[1] + 2 * k], fill=ARM_LINK, outline=CAR_EDGE, width=SS)
    theta = math.radians(pose.jaw)
    for sign in (1, -1):
        d.line([tip, at(length + 8.0 * math.cos(theta), sign * 8.0 * math.sin(theta))], fill=JAW + (255,), width=3 * SS)
    held = at(length + 6.0)
    r = 4.5 * k
    if pose.ball == "gripper":
        d.ellipse([held[0] - r, held[1] - r, held[0] + r, held[1] + r], fill=BALL, outline=BALL_EDGE, width=SS)
    elif pose.ball == "falling" and arm is not None:                 # dropping into the bin
        ex, ey = P(*arm.target)
        bx, by = held[0] + (ex - held[0]) * pose.fall, held[1] + (ey - held[1]) * pose.fall
        r *= 1.0 - 0.45 * pose.fall
        d.ellipse([bx - r, by - r, bx + r, by + r], fill=BALL + (int(255 * (1.0 - 0.6 * pose.fall)),), outline=BALL_EDGE, width=SS)


def camera_wedge():
    """(start angle, end angle, radius in cm) of the camera's view relative to the nose: 36 degrees wide, as far as it finds a ball."""
    return -CAM_HALF_FOV, CAM_HALF_FOV, CAM_RANGE


def camera_polygon(world, x, y, heading_deg, t, step=3.0):
    """The floor the camera can see from (x, y) facing `heading_deg` at time t, as world points starting at the camera: a 36 degree wedge
    up to its range, cut short wherever a wall, an obstacle that is there at that time, or the bin blocks the view (a ball does not)."""
    lo, hi, reach = camera_wedge()
    pts = [(x, y)]
    n = max(1, int(round((hi - lo) / step)))
    for i in range(n + 1):
        off = lo + (hi - lo) * i / n
        dist, _ = world.cast(x, y, heading_deg + off, t, reach, skip=("ball",))
        dist = reach if dist is None else min(dist, reach)
        a = math.radians(heading_deg + off)
        pts.append((x + dist * math.cos(a), y + dist * math.sin(a)))
    return pts


def picture_size(result):
    w = result.world
    return int(w.width * SCALE) + 2 * MARGIN, int(w.height * SCALE) + 2 * MARGIN


def _draw(result, t, trail_s, show_guess, view, show_camera=False, show_route=False, show_seen=False, show_blocked=False, show_pending=False):
    """Draw the arena at time t (None = the whole run). `view` = (x, y, half_cm) draws just that part, magnified, as a square."""
    w, car = result.world, result.car
    if view is None:
        W, H = picture_size(result)
        k = SCALE * SS
    else:
        W = H = CLOSEUP_PX
        k = CLOSEUP_PX * SS / (2.0 * view[2])
    img = Image.new("RGB", (W * SS, H * SS), BG)
    d = ImageDraw.Draw(img, "RGBA")
    overview = t is None
    s = state_at(result, car.path[-1][2] if overview else t)

    def P(x, y):
        if view is not None:
            return ((x - (view[0] - view[2])) * k, (view[1] + view[2] - y) * k)
        return ((MARGIN + 0) * SS + x * k, (MARGIN * SS + (w.height - y) * k))

    def text(xy, string, px, fill=TEXT, bold=False, anchor="mm"):
        d.text(xy, string, font=_font(px * SS, bold), fill=fill, anchor=anchor)

    def dashed_box(x1, y1, x2, y2, colour):
        corners = [P(x1, y2), P(x2, y2), P(x2, y1), P(x1, y1), P(x1, y2)]
        for a, b in zip(corners, corners[1:]):
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(length / (7 * SS)))
            for i in range(0, n, 2):
                p0 = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                p1 = (a[0] + (b[0] - a[0]) * min(i + 1, n) / n, a[1] + (b[1] - a[1]) * min(i + 1, n) / n)
                d.line([p0, p1], fill=colour, width=2 * SS)

    # floor, grid, walls
    d.rounded_rectangle([P(0, w.height), P(w.width, 0)], radius=12 * SS, fill=FLOOR)
    for gx in range(50, int(w.width), 50):
        d.line([P(gx, w.height), P(gx, 0)], fill=GRID, width=SS)
    for gy in range(50, int(w.height), 50):
        d.line([P(0, gy), P(w.width, gy)], fill=GRID, width=SS)
    d.rounded_rectangle([P(0, w.height), P(w.width, 0)], radius=12 * SS, outline=WALL, width=3 * SS)
    x0, y0 = P(12, 14)
    d.line([(x0, y0), (x0 + 100 * k, y0)], fill=MUTED, width=2 * SS)
    text((x0 + 50 * k, y0 - 9 * SS), "1 m", 10, MUTED)

    # the floor the car has already looked at (a faint green tint), under everything else
    if show_seen and t is not None and view is None:
        from sim.gridmap import CELL
        for sx, sy in state_at(result, t).seen:
            x0c, x1c = max(sx, 0.0), min(sx + CELL, w.width)
            y0c, y1c = max(sy, 0.0), min(sy + CELL, w.height)
            if x1c > x0c and y1c > y0c:
                d.rectangle([P(x0c, y1c), P(x1c, y0c)], fill=GOOD + (30,))

    # the lane route being followed (lane search only), under everything that moves
    if show_route and t is not None and view is None and state_at(result, t).route:
        route = [P(x, y) for x, y in state_at(result, t).route]
        for (ax, ay), (bx2, by2) in zip(route[0::2], route[1::2]):                      # each lane: a dashed line
            length = math.hypot(bx2 - ax, by2 - ay)
            n = max(1, int(length / (9 * SS)))
            for i in range(0, n, 2):
                d.line([(ax + (bx2 - ax) * i / n, ay + (by2 - ay) * i / n), (ax + (bx2 - ax) * min(i + 1, n) / n, ay + (by2 - ay) * min(i + 1, n) / n)],
                       fill=ROUTE_COLOUR + (150,), width=2 * SS)
        for (ax, ay), (bx2, by2) in zip(route[1:-1:2], route[2::2]):                    # and the step over to the next lane
            d.line([(ax, ay), (bx2, by2)], fill=ROUTE_COLOUR + (90,), width=SS)
        for px, py in route:
            d.ellipse([px - 3 * SS, py - 3 * SS, px + 3 * SS, py + 3 * SS], fill=ROUTE_COLOUR + (200,))

    # permanent obstacles (chair legs are drawn bigger than life so they can be seen)
    legs = []
    for o in w.obstacles:
        if not o.permanent:
            continue
        if o.thin:
            cx, cy = P((o.x1 + o.x2) / 2, (o.y1 + o.y2) / 2)
            r = 4.5 * k
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=LEG, outline=(120, 90, 4), width=SS)
            legs.append(((o.x1 + o.x2) / 2, (o.y1 + o.y2) / 2, o.label))
            continue
        d.rounded_rectangle([P(o.x1, o.y2), P(o.x2, o.y1)], radius=5 * SS, fill=OBSTACLE, outline=OBSTACLE_EDGE, width=2 * SS)
        if o.label:
            cx, cy = P((o.x1 + o.x2) / 2, (o.y1 + o.y2) / 2)
            text((cx, cy), o.label, 13, TEXT, bold=True)
    if legs:
        text(P(sum(l[0] for l in legs) / len(legs), max(l[1] for l in legs) + 18), "Chair legs (hard to see)", 11, LEG)

    # temporary obstacles: solid while they are there, a dotted outline before they arrive
    for o in w.obstacles:
        if o.permanent:
            continue
        name = o.label or "Obstacle"
        here = o.active(s.t) and not overview
        cx, cy = P((o.x1 + o.x2) / 2, (o.y1 + o.y2) / 2)
        if here:
            d.rounded_rectangle([P(o.x1, o.y2), P(o.x2, o.y1)], radius=5 * SS, fill=TEMP + (255,), outline=TEMP_EDGE, width=2 * SS)
            until = "" if o.vanish == math.inf else f"\nuntil {clock(o.vanish)}"
            text((cx, cy), f"{name}{until}", 12, (255, 244, 214), bold=True)
        elif overview or s.t < o.appear:
            dashed_box(o.x1, o.y1, o.x2, o.y2, TEMP_EDGE + (150,))
            text((cx, cy), f"{name}\nappears {clock(o.appear)}", 11, TEMP_EDGE + (200,))

    # what the car believes are obstacles and walls: a red tint (on top of the real shapes, so the two can be compared)
    if show_blocked and t is not None and view is None:
        from sim.gridmap import CELL
        for sx, sy in state_at(result, t).obstacles:
            x0c, x1c = max(sx, 0.0), min(sx + CELL, w.width)
            y0c, y1c = max(sy, 0.0), min(sy + CELL, w.height)
            if x1c > x0c and y1c > y0c:
                d.rectangle([P(x0c, y1c), P(x1c, y0c)], fill=BAD + (85,))

    # the bin
    bx, by = P(*w.bin_xy)
    br = w.bin_radius * k
    d.ellipse([bx - br, by - br, bx + br, by + br], fill=BIN, outline=BIN_EDGE, width=3 * SS)
    text((bx, by - 4 * SS), "BIN", 13, (255, 255, 255), bold=True)
    text((bx, by + 8 * SS), f"{s.delivered} inside", 10, (219, 234, 254))

    # the car's route
    pts = [(p[0], p[1], p[2]) for p in car.path]
    if overview:
        seg = [P(x, y) for x, y, _ in pts]
        for a, b in zip(seg, seg[1:]):
            d.line([a, b], fill=TRAIL + (70,), width=2 * SS)
    else:
        recent = [(x, y, tt) for x, y, tt in pts if s.t - trail_s <= tt <= s.t] + [(s.x, s.y, s.t)]
        for (xa, ya, ta), (xb, yb, tb) in zip(recent, recent[1:]):
            age = (s.t - tb) / trail_s
            d.line([P(xa, ya), P(xb, yb)], fill=TRAIL + (int(210 * (1 - age)),), width=3 * SS)

    # balls
    if overview:
        picked = sorted((b for b in w.balls if b.picked_at is not None), key=lambda b: b.picked_at)
        for n, b in enumerate(picked, 1):
            cx, cy = P(b.x, b.y)
            colour = GOOD if b.delivered_at is not None else BAD
            r = 8 * SS
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(15, 20, 26, 230), outline=colour, width=2 * SS)
            text((cx, cy), str(n), 11, colour, bold=True)
        for b in w.balls:
            if b.picked_at is None:
                cx, cy = P(b.x, b.y)
                r = 6 * SS
                d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=BAD + (230,), outline=(255, 255, 255, 200), width=SS)
                text((cx, cy - 12 * SS), "never found", 10, BAD)
    else:
        for bx_, by_ in s.floor_balls:
            cx, cy = P(bx_, by_)
            r = 5 * k
            d.ellipse([cx - r, cy - r + 2 * SS, cx + r, cy + r + 2 * SS], fill=(0, 0, 0, 70))
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=BALL, outline=BALL_EDGE, width=SS)

    # balls it has seen and left: a blue ring, to come back for
    if show_pending and t is not None and view is None:
        for px, py in state_at(result, t).pending:
            cx_, cy_ = P(px, py)
            r = 13 * k
            d.ellipse([cx_ - r, cy_ - r, cx_ + r, cy_ + r], fill=PENDING_BLUE + (50,), outline=PENDING_BLUE + (255,), width=3 * SS)
            text((cx_, cy_ - r - 8 * SS), "come back", 10, (147, 197, 253), bold=True)

    # start marker
    sx, sy = P(car.path[0][0], car.path[0][1])
    d.ellipse([sx - 5 * SS, sy - 5 * SS, sx + 5 * SS, sy + 5 * SS], outline=MUTED, width=2 * SS)
    text((sx, sy + 13 * SS), "start", 10, MUTED)

    if overview:
        ex, ey, eh = car.x, car.y, car.h
    else:
        ex, ey, eh = s.x, s.y, s.h
    cx, cy = P(ex, ey)
    a = math.radians(eh)
    fx, fy = math.cos(a), -math.sin(a)          # forward on the screen
    rx, ry = -fy, fx                            # to the right of forward

    # the car: sensor cone, body, nose, and the ball in the gripper
    if not overview:
        ox, oy = cx + fx * SENSOR_OFFSET * k, cy + fy * SENSOR_OFFSET * k
        if show_camera:                                              # what the camera can see: a narrow wedge, cut short by what blocks it
            a = math.radians(s.h)
            poly = camera_polygon(w, s.x + SENSOR_OFFSET * math.cos(a), s.y + SENSOR_OFFSET * math.sin(a), s.h, s.t)
            screen = [P(px, py) for px, py in poly]
            d.polygon(screen, fill=CAMERA_VIEW + (34,))
            d.line([screen[0], screen[1], *screen[2:-1], screen[-1], screen[0]], fill=CAMERA_VIEW + (110,), width=SS)
        rad = 60 * k                                                 # the distance sensor's beam (drawn short)
        d.pieslice([ox - rad, oy - rad, ox + rad, oy + rad], start=-eh - 12, end=-eh + 12, fill=CAR + (30,))
    half = car.RADIUS * k * 0.95
    body = [(cx + fx * half * sf + rx * half * sr, cy + fy * half * sf + ry * half * sr)
            for sf, sr in ((1, -0.8), (1, 0.8), (-1, 0.8), (-1, -0.8))]
    d.polygon(body, fill=CAR, outline=CAR_EDGE)
    d.line(body + [body[0]], fill=CAR_EDGE, width=2 * SS, joint="curve")
    nose = [(cx + fx * half * 1.55, cy + fy * half * 1.55), (cx + fx * half * 0.95 + rx * half * 0.45, cy + fy * half * 0.95 + ry * half * 0.45),
            (cx + fx * half * 0.95 - rx * half * 0.45, cy + fy * half * 0.95 - ry * half * 0.45)]
    d.polygon(nose, fill=(255, 255, 255), outline=CAR_EDGE)
    if not overview:
        _draw_arm(d, s, (cx, cy), (fx, fy), (rx, ry), k, P, w)

    # where the car THINKS it is
    if show_guess and not overview and s.guess is not None:
        gx, gy = P(*s.guess)
        d.line([(cx, cy), (gx, gy)], fill=BAD + (120,), width=2 * SS)
        r = 11 * k
        d.ellipse([gx - r, gy - r, gx + r, gy + r], outline=BAD + (230,), width=2 * SS)
        text((gx, gy - r - 8 * SS), "the car thinks it is here", 10, BAD)

    if view is None:
        # status chip
        label = (f"Whole run  {clock(result.seconds)}" if overview else f"{clock(s.t)}") + f"     In bin {s.delivered}/{result.balls}     Bumps {s.bumps}"
        if not overview and show_seen:
            label += f"     Looked at {s.coverage:.0%}"
        font = _font(13 * SS, True)
        tw = d.textlength(label, font=font)
        cx0, cy0 = (MARGIN + 10) * SS, (MARGIN + 10) * SS
        d.rounded_rectangle([cx0, cy0, cx0 + tw + 24 * SS, cy0 + 28 * SS], radius=14 * SS, fill=(8, 12, 18, 200))
        d.text((cx0 + 12 * SS, cy0 + 14 * SS), label, font=font, fill=TEXT, anchor="lm")
        caption = arm_caption(s.arm) if not overview else ""
        if caption:
            tw = d.textlength(caption, font=font)
            d.rounded_rectangle([cx0, cy0 + 34 * SS, cx0 + tw + 24 * SS, cy0 + 62 * SS], radius=14 * SS, fill=(30, 64, 175, 235))
            d.text((cx0 + 12 * SS, cy0 + 48 * SS), caption, font=font, fill=(255, 255, 255), anchor="lm")
    return img.resize((W, H), Image.LANCZOS)


def render_closeup(result, t, trail_s=25.0):
    """A magnified view of the car and what its arm reaches for, while the arm moves (None otherwise)."""
    s = state_at(result, t)
    if s.arm is None:
        return None
    tx, ty = s.arm.target
    img = _draw(result, t, trail_s, False, ((s.x + tx) / 2.0, (s.y + ty) / 2.0, CLOSEUP_HALF_CM))
    d = ImageDraw.Draw(img, "RGBA")
    d.rounded_rectangle([1, 1, CLOSEUP_PX - 2, CLOSEUP_PX - 2], radius=10, outline=(147, 197, 253), width=2)
    font = _font(11, True)
    d.rounded_rectangle([8, 8, 8 + d.textlength("Arm close-up", font=font) + 14, 28], radius=9, fill=(8, 12, 18, 215))
    d.text((15, 18), "Arm close-up", font=font, fill=TEXT, anchor="lm")
    return img


def render_frame(result, t=None, trail_s=25.0, show_guess=False, show_camera=False, show_route=False, show_seen=False,
                 show_blocked=False, show_pending=False):
    """The arena at time t (seconds), or the whole run when t is None. While the arm moves, a magnified close-up of it is pasted into the
    top-right corner. Returns a PIL image."""
    img = _draw(result, t, trail_s, show_guess, None, show_camera and t is not None, show_route, show_seen, show_blocked, show_pending)
    if t is not None:
        inset = render_closeup(result, t, trail_s)
        if inset is not None:
            img.paste(inset, (img.width - CLOSEUP_PX - 14, 14))
    return img


def render_image(result):
    """The whole-run overview picture."""
    return render_frame(result, None)


def render(result, path):
    img = render_image(result)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    img.save(path)
