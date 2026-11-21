"""Design of the 2:1 gear reduction for the SG90 gripper, from the layout in gripper_before.blend.

    python build_after.py      ->  gripper_after_parts.dxf (cut file), stl/ big_gear, pinion, servo_bracket, bridge_plate, spacers,
                                   layout.json (positions for make_after_blend.py)

Needs ezdxf and the OpenSCAD command line (for the STL files). All numbers are millimetres in the Blender
world of gripper_before.blend, read from that file with Blender's Python.
"""
import json
import math
import os
import subprocess

import ezdxf

from make_gripper_gears import MODULE, PINION_TEETH, GEAR_TEETH, gear_outline

HERE = os.path.dirname(os.path.abspath(__file__))
OPENSCAD = r"C:\Program Files\OpenSCAD\openscad.exe"

# ---- measured from gripper_before.blend -----------------------------------------------------------------------
S = (-13.10, 0.12)             # the shaft / hub axis that carries gears 29 and 30
TOP_OF_PART_29 = 5.99          # z of the top face of the topmost gear (29)
OLD_PLATE_TOP = -4.89          # z of the top face of plate 24 (the frame plate over the old servo ears)
SHARED_HOLES = [(6.70, -0.30), (0.40, 6.80)]          # M2.5 holes shared by gears 29 and 30, relative to S
# plate 24 (the cage's top plate) has a big servo window x -30..-7, y -9.1..9.3 and tab slots at x -32.2 / -2.6 / 12.7,
# so the only solid place for the new bracket is the left strip x -35..-30: two NEW 2.2 mm holes are drilled there
PLATE_NEW_HOLES = [(-32.5, -4.0), (-32.5, 4.0)]
PLATE_TOP, PLATE_BOTTOM = -4.89, -7.89

# ---- design ---------------------------------------------------------------------------------------------------
T = 3.0                                          # sheet thickness
CENTRE_DISTANCE = MODULE * (PINION_TEETH + GEAR_TEETH) / 2
PINION_DIR = 110.0                                # direction from the big gear's axis S to the servo shaft (servo beside the frame)
P = (S[0] + CENTRE_DISTANCE * math.cos(math.radians(PINION_DIR)), S[1] + CENTRE_DISTANCE * math.sin(math.radians(PINION_DIR)))
SERVO_ROT = 0.0                                  # servo parallel to the arm axis, shaft at its front (+X) end
# Low build (your layout in gripper_after_v2.blend): the servo hangs under the bracket (ears directly under it), the big gear
# lies directly on gear 29 (no spacers) and the pinion sits on the servo's splined shaft in the same layer, on a small hub
# disc that rests on the servo's boss.
BRACKET_Z = PLATE_TOP                            # bracket lies on plate 24, 3 mm thick
BRACKET_TOP = BRACKET_Z + T
EARS_TOP = BRACKET_Z                             # the servo ears are right under the bracket
SHAFT_TOP = EARS_TOP + 11.5
GEAR_Z = TOP_OF_PART_29 + 0.01                   # big gear lies on gear 29
PINION_Z = GEAR_Z                                # pinion in the same layer
HUB_Z = PINION_Z - T                             # hub disc under the pinion: 3 mm more grip on the shaft
HUB_D = 8.6                                      # smaller than the big gear's tooth tips reach (4.5 from the pinion axis, 2*4.3)
GEAR_SPACER_H = 0.0
BIG_ROT = PINION_DIR % (360.0 / GEAR_TEETH)      # a tooth points at the pinion
PIN_ROT = PINION_DIR + 180.0 - 180.0 / PINION_TEETH   # a gap points at the big gear
TAP_M3, TAP_M2_5, TAP_M2 = 2.5, 2.1, 1.6          # undersized holes: the screws cut their own thread in the sheet (no nuts)
SHAFT_HOLE = 4.7                                 # press fit on the 4.8 mm splined shaft (epoxy it, plus an M2 screw + washer on top)
# the case centre is 5.25 mm from the shaft (the shaft is 6 mm from one end), along the servo's own -X
CASE_C = (P[0] - 5.25 * math.cos(math.radians(SERVO_ROT)), P[1] - 5.25 * math.sin(math.radians(SERVO_ROT)))
EAR_HOLE_DX = 13.75
STANDOFF_A = (-4.0, 25.0)                          # bridge standoffs: outside the servo ears, the big gear and the pinion
STANDOFF_B = (-28.0, 34.0)
KEEP_OUT_S = 10.5                                  # the bracket stays this far from S (gear-bolt heads sweep round it)
STANDOFF_H = 11.5                                  # M3 female-female standoffs
BRIDGE_Z = BRACKET_TOP + STANDOFF_H                # underside of the bridge plate
PIN_LEN = 6.0                                      # M3x14 pin: head on the bridge, through the big gear's centre hole

def rot(pts, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(x * c - y * s, x * s + y * c) for x, y in pts]


def circle_pts(cx, cy, d, n=48):
    return [(cx + d / 2 * math.cos(2 * math.pi * k / n), cy + d / 2 * math.sin(2 * math.pi * k / n)) for k in range(n)]


def hull(points):
    pts = sorted(set(points))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def blob(points, r, n=16):
    """Rounded outline round a set of points (convex hull grown by r) as a polygon."""
    from shapely.geometry import MultiPoint
    return list(MultiPoint(points).convex_hull.buffer(r, quad_segs=n).exterior.coords)[:-1]


def bracket_shape():
    """Small servo bracket: lies on the solid left strip of plate 24 (two NEW M2 holes), carries the servo by its ears
    and the two bridge standoffs.  Returns (outline, [hole rings])."""
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    cx, cy = CASE_C
    ear = [(cx + rot([(s * EAR_HOLE_DX, 0)], SERVO_ROT)[0][0], cy + rot([(s * EAR_HOLE_DX, 0)], SERVO_ROT)[0][1]) for s in (-1, 1)]
    body = unary_union([
        Polygon(blob(ear + PLATE_NEW_HOLES, 4.5)),
        Polygon(blob([ear[1], STANDOFF_A], 4.5)),
        Polygon(blob([ear[0], STANDOFF_B], 4.5)),
    ])
    body = body.difference(Point(S).buffer(KEEP_OUT_S))        # keep clear of the gear-bolt heads that sweep round S
    window = Polygon([(cx + x, cy + y) for x, y in rot([(-11.45, -6.2), (11.45, -6.2), (11.45, 6.2), (-11.45, 6.2)], SERVO_ROT)])
    body = body.difference(window)
    assert body.geom_type == "Polygon", "bracket fell into pieces"
    outline = list(body.exterior.coords)[:-1]
    holes = [circle_pts(x, y, TAP_M2) for x, y in ear] + [circle_pts(x, y, TAP_M2) for x, y in PLATE_NEW_HOLES]
    holes += [circle_pts(x, y, 3.2) for x, y in (STANDOFF_A, STANDOFF_B)]
    return outline, [list(body_in.coords)[:-1] for body_in in body.interiors] + holes


def bridge_shape():
    outline = blob([S, STANDOFF_A, STANDOFF_B], 5.5)
    holes = [circle_pts(S[0], S[1], TAP_M3)] + [circle_pts(x, y, 3.2) for x, y in (STANDOFF_A, STANDOFF_B)]
    return outline, holes


def scad_polygon(rings):
    pts, paths = [], []
    for ring in rings:
        start = len(pts)
        pts += ring
        paths.append(list(range(start, len(pts))))
    return "polygon(points=[%s], paths=%s);" % (",".join("[%.3f,%.3f]" % p for p in pts), paths)


def write_stl(name, rings, height):
    src = os.path.join(HERE, "stl", "_%s.scad" % name)
    open(src, "w").write("linear_extrude(%g) %s\n" % (height, scad_polygon(rings)))
    subprocess.run([OPENSCAD, "-o", os.path.join(HERE, "stl", name + ".stl"), src], check=True, capture_output=True)
    os.remove(src)


def write_hw():
    """Extra fasteners as STL (made from screws.scad)."""
    for name in ("m3x6", "m2x5", "m2x6", "m2_5x8", "standoff"):
        out = os.path.join(HERE, "stl", ("screw_" + name if name[0] == "m" else name) + ".stl")
        subprocess.run([OPENSCAD, "-D", 'which="%s"' % name, "-D", "standoff_h=%g" % STANDOFF_H, "-o", out, os.path.join(HERE, "screws.scad")], check=True, capture_output=True)


def main():
    root_pin = MODULE * (PINION_TEETH / 2 - 1.25)
    assert SHAFT_HOLE / 2 + 1.2 < root_pin, "pinion wall around the shaft too thin"
    big_holes = [rot([h], -BIG_ROT)[0] for h in SHARED_HOLES]
    big = [gear_outline(GEAR_TEETH), circle_pts(0, 0, 3.2)] + [circle_pts(x, y, TAP_M2_5) for x, y in big_holes]
    pin = [gear_outline(PINION_TEETH), circle_pts(0, 0, SHAFT_HOLE)]
    bracket_out, bracket_holes = bracket_shape()
    bracket = [bracket_out] + bracket_holes
    bridge_out, bridge_holes = bridge_shape()
    bridge = [bridge_out] + bridge_holes
    write_stl("big_gear", big, T)
    write_stl("pinion", pin, T)
    write_stl("servo_bracket", bracket, T)
    write_stl("bridge_plate", bridge, T)
    write_hw()
    write_stl("pinion_hub", [circle_pts(0, 0, HUB_D), circle_pts(0, 0, SHAFT_HOLE)], T)
    old = os.path.join(HERE, "stl", "spacer_1mm.stl")
    if os.path.exists(old):
        os.remove(old)

    doc = ezdxf.new("R2010")
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=7)
    doc.layers.add("NOTES", color=3)
    msp = doc.modelspace()

    def put(rings, dx, dy, label, ly=-26):
        for ring in rings:
            msp.add_lwpolyline([(x + dx, y + dy) for x, y in ring], close=True, dxfattribs={"layer": "CUT"})
        if label:
            msp.add_text(label, dxfattribs={"layer": "NOTES", "height": 2.5}).set_placement((dx - 18, dy + ly))

    put(big, 0, 0, "BIG GEAR 24T x1 (3 mm)")
    put(pin, 50, 0, "PINION %dT x1 (3 mm)" % PINION_TEETH)
    put([circle_pts(0, 0, HUB_D), circle_pts(0, 0, SHAFT_HOLE)], 85, 0, "HUB x1 (3 mm)")
    bc = (-35.0, 12.0)
    put([[(x - bc[0], y - bc[1] - 55) for x, y in ring] for ring in bracket], 0, 0, "SERVO BRACKET x1 (3 mm)", -38)
    sc = (-28.0, 20.0)
    put([[(x - sc[0] + 70, y - sc[1] - 55) for x, y in ring] for ring in bridge], 0, 0, "BRIDGE PLATE x1 (3 mm)", -38)
    msp.add_text("PLATE 24: drill two 2.2 mm holes at x=%.1f, y=%.1f and %.1f (Blender world coordinates)" %
                 (PLATE_NEW_HOLES[0][0], PLATE_NEW_HOLES[0][1], PLATE_NEW_HOLES[1][1]), dxfattribs={"layer": "NOTES", "height": 2.5}).set_placement((-20, -130))
    msp.add_text("HUB x1 (3 mm, d %.1f) glued under the pinion on the servo shaft; no spacers: the big gear lies on gear 29" % HUB_D, dxfattribs={"layer": "NOTES", "height": 2.5}).set_placement((-20, -122))
    doc.saveas(os.path.join(HERE, "gripper_after_parts.dxf"))

    ear = [[CASE_C[0] + rot([(s * EAR_HOLE_DX, 0)], SERVO_ROT)[0][0], CASE_C[1] + rot([(s * EAR_HOLE_DX, 0)], SERVO_ROT)[0][1]] for s in (-1, 1)]
    layout = dict(
        S=S, P=P, T=T, gear_z=GEAR_Z, pinion_z=PINION_Z, big_rot=BIG_ROT, pin_rot=PIN_ROT, ears_top=EARS_TOP,
        shaft_top=SHAFT_TOP, bracket_z=BRACKET_Z, bracket_top=BRACKET_TOP, bridge_z=BRIDGE_Z, standoff_h=STANDOFF_H,
        standoffs=[list(STANDOFF_A), list(STANDOFF_B)], ear_holes=ear, plate_holes=[list(h) for h in PLATE_NEW_HOLES],
        plate_top=PLATE_TOP, plate_bottom=PLATE_BOTTOM, pin_len=PIN_LEN,
        gear_top=GEAR_Z + T, gear30_bottom=-0.02, servo_rot=SERVO_ROT, centre_distance=CENTRE_DISTANCE,
        hub_z=HUB_z if False else HUB_Z, gear_spacer_z=TOP_OF_PART_29,
        gear_spacer_xy=[[S[0] + h[0], S[1] + h[1]] for h in SHARED_HOLES])
    json.dump(layout, open(os.path.join(HERE, "layout.json"), "w"), indent=1)
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in layout.items()}))


if __name__ == "__main__":
    main()
