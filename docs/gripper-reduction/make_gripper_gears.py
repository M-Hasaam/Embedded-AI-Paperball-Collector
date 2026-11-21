"""2:1 reduction gear pair for the SG90 gripper, as a laser-cut DXF (millimetres, 3 mm sheet).

    python make_gripper_gears.py          -> gripper_reduction_gears.dxf + gripper_reduction_preview.png

Needs: pip install ezdxf matplotlib

The servo turns the small PINION, which drives the big GEAR. The gear is fixed on the jaw shaft, so the jaws get
twice the force at half the speed (the servo has to turn twice as far for the same jaw movement).

Everything marked ASSUMED below is a guess that has to be checked against the real parts before cutting.
"""
import math
import os

import ezdxf

# ---- gear data -------------------------------------------------------------------------------------------------
MODULE = 1.5            # mm. Pitch diameter = MODULE * teeth. 2 mm is the size of the MeArm's own gripper gears.
PRESSURE_ANGLE = 20.0   # degrees
PINION_TEETH = 8
GEAR_TEETH = 24         # 3:1
BACKLASH = 0.2          # mm of play between the teeth (a laser cut also removes about 0.15 mm, so keep a little)

# ---- holes (ASSUMED: check them against the real horn and shaft) -----------------------------------------------
HORN_CENTER_HOLE = 4.0          # mm, clearance for the SG90 horn hub / spline boss
HORN_SCREW_HOLE = 2.0           # mm, for the 2 mm screws that hold the horn to the pinion
HORN_SCREW_SPACING = 10.0       # mm, centre to centre of the two horn screws (measure your horn; must stay inside the root circle)
SHAFT_HOLE = 3.2                # mm, for an M3 bolt as the jaw shaft
GEAR_BOLT_HOLE = 3.2            # mm, M3 bolts that clamp the gear to the jaw gear / spacer
GEAR_BOLT_RADIUS = 9.0          # mm from the centre, 3 holes 120 deg apart
# ----------------------------------------------------------------------------------------------------------------


def inv(a):
    return math.tan(a) - a


def gear_outline(teeth, module=MODULE, pressure=PRESSURE_ANGLE, backlash=BACKLASH, steps=8):
    """Closed outline of a spur gear as a list of (x, y) points, centred on the origin."""
    alpha = math.radians(pressure)
    rp = module * teeth / 2
    rb = rp * math.cos(alpha)
    ra = rp + module
    rf = rp - 1.25 * module
    # half the tooth thickness at the pitch circle, as an angle (thinned a bit to leave backlash)
    half_pitch = (math.pi * module / 2 - backlash / 2) / (2 * rp)

    def half_angle(r):
        return half_pitch + inv(alpha) - inv(math.acos(min(1.0, rb / r)))

    r_start = max(rb, rf)
    radii = [r_start + (ra - r_start) * k / steps for k in range(steps + 1)]
    pts = []
    for t in range(teeth):
        a0 = 2 * math.pi * t / teeth
        flank_right = [(r, a0 - half_angle(r)) for r in radii]
        flank_left = [(r, a0 + half_angle(r)) for r in reversed(radii)]
        right_root = (rf, flank_right[0][1])
        left_root = (rf, flank_left[-1][1])
        seq = [right_root] + flank_right + flank_left + [left_root]
        # root arc to the next tooth
        next_right = (rf, 2 * math.pi * (t + 1) / teeth - half_angle(r_start))
        arc_n = 3
        arc = [(rf, left_root[1] + (next_right[1] - left_root[1]) * k / arc_n) for k in range(1, arc_n)]
        seq += arc
        pts += [(r * math.cos(a), r * math.sin(a)) for r, a in seq]
    return pts


def add_circle(msp, x, y, d):
    msp.add_circle((x, y), d / 2, dxfattribs={"layer": "CUT"})


def build(path_dxf, path_png):
    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=7)
    doc.layers.add("NOTES", color=3)
    msp = doc.modelspace()

    rp_pin = MODULE * PINION_TEETH / 2
    root_pin = rp_pin - 1.25 * MODULE
    assert HORN_SCREW_SPACING / 2 + HORN_SCREW_HOLE / 2 + 1.0 < root_pin, "horn holes too close to the tooth roots"
    assert GEAR_BOLT_RADIUS + GEAR_BOLT_HOLE / 2 + 1.0 < MODULE * GEAR_TEETH / 2 - 1.25 * MODULE
    rp_gear = MODULE * GEAR_TEETH / 2
    ra_gear = rp_gear + MODULE
    centre_distance = rp_pin + rp_gear

    # parts are laid out side by side, not meshed: the PINION on the left, the GEAR on the right
    pin_c = (0.0, 0.0)
    gear_c = (rp_pin + MODULE + 6 + ra_gear, 0.0)

    msp.add_lwpolyline(
        [(x + pin_c[0], y + pin_c[1]) for x, y in gear_outline(PINION_TEETH)], close=True, dxfattribs={"layer": "CUT"})
    add_circle(msp, *pin_c, HORN_CENTER_HOLE)
    for s in (-1, 1):
        add_circle(msp, pin_c[0] + s * HORN_SCREW_SPACING / 2, pin_c[1], HORN_SCREW_HOLE)

    msp.add_lwpolyline(
        [(x + gear_c[0], y + gear_c[1]) for x, y in gear_outline(GEAR_TEETH)], close=True, dxfattribs={"layer": "CUT"})
    add_circle(msp, *gear_c, SHAFT_HOLE)
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 2
        add_circle(msp, gear_c[0] + GEAR_BOLT_RADIUS * math.cos(a), gear_c[1] + GEAR_BOLT_RADIUS * math.sin(a),
                   GEAR_BOLT_HOLE)

    msp.add_text(f"PINION {PINION_TEETH}T (on the servo)", dxfattribs={"layer": "NOTES", "height": 2.5}).set_placement(
        (pin_c[0] - 16, -rp_pin - MODULE - 8))
    msp.add_text(f"GEAR {GEAR_TEETH}T (on the jaw shaft)  centre distance {centre_distance:.0f} mm  module {MODULE:g}",
                 dxfattribs={"layer": "NOTES", "height": 2.5}).set_placement((gear_c[0] - 30, -ra_gear - 8))
    doc.saveas(path_dxf)

    # preview: the two parts as cut, plus the pair meshed at the right centre distance
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))
    for ax in (ax1, ax2):
        ax.set_aspect("equal")
        ax.grid(True, lw=0.3)
    for pts, c in ((gear_outline(PINION_TEETH), pin_c), (gear_outline(GEAR_TEETH), gear_c)):
        xs = [p[0] + c[0] for p in pts] + [pts[0][0] + c[0]]
        ys = [p[1] + c[1] for p in pts] + [pts[0][1] + c[1]]
        ax1.plot(xs, ys, "k", lw=1)
    for e in msp:
        if e.dxftype() == "CIRCLE":
            t = [i * math.pi / 30 for i in range(61)]
            ax1.plot([e.dxf.center.x + e.dxf.radius * math.cos(a) for a in t],
                     [e.dxf.center.y + e.dxf.radius * math.sin(a) for a in t], "b", lw=0.8)
    ax1.set_title("as cut (mm)")
    # meshed view: the pinion is turned half a tooth so a tooth sits in a gap
    def mesh(pts, c, rot):
        ca, sa = math.cos(rot), math.sin(rot)
        return [(c[0] + x * ca - y * sa, c[1] + x * sa + y * ca) for x, y in pts] + [(c[0] + pts[0][0] * ca - pts[0][1] * sa,
                                                                                     c[1] + pts[0][0] * sa + pts[0][1] * ca)]
    big = mesh(gear_outline(GEAR_TEETH), (0, 0), 0)
    # the big gear has a tooth pointing at the pinion, so the pinion shows a gap there
    small = mesh(gear_outline(PINION_TEETH), (centre_distance, 0), math.pi - math.pi / PINION_TEETH)
    ax2.plot(*zip(*big), "k", lw=1)
    ax2.plot(*zip(*small), "r", lw=1)
    ax2.set_title(f"meshed: centre distance {centre_distance:.0f} mm, ratio {GEAR_TEETH / PINION_TEETH:.0f}:1")
    fig.savefig(path_png, dpi=100)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    build(os.path.join(here, "gripper_reduction_gears.dxf"), os.path.join(here, "gripper_reduction_preview.png"))
    print("written")
