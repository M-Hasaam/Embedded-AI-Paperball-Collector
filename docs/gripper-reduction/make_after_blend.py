"""Run with Blender:  blender -b gripper_before.blend --python make_after_blend.py
Builds gripper_after.blend: the before layout with the servo moved back and the 2:1 gears added, then checks collisions."""
import json, math, os, bpy
from mathutils.bvhtree import BVHTree

here = os.path.dirname(os.path.abspath(bpy.data.filepath))
L = json.load(open(os.path.join(here, "layout.json")))
stl = lambda n: os.path.join(here, "stl", n + ".stl")

for name in ("sg90_servo.002", "sg90_horn"):   # the old servo and horn are replaced
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)

def add(name, file, loc, rot_z=0.0, flip=False):
    bpy.ops.wm.stl_import(filepath=stl(file))
    o = bpy.context.selected_objects[0]
    o.name = name
    o.location = loc
    o.rotation_euler = (math.pi if flip else 0, 0, math.radians(rot_z))
    return o

new = []
S, P = L["S"], L["P"]
new.append(add("big_gear_NEW", "big_gear", (S[0], S[1], L["gear_z"]), L["big_rot"]))
new.append(add("pinion_NEW", "pinion", (P[0], P[1], L["pinion_z"]), L["pin_rot"]))
new.append(add("sg90_servo_NEW", "sg90_servo", (P[0], P[1], L["ears_top"]), L["servo_rot"]))
new.append(add("pinion_hub_NEW", "pinion_hub", (P[0], P[1], L["hub_z"])))
new.append(add("servo_bracket_NEW", "servo_bracket", (0, 0, L["bracket_z"])))
new.append(add("bridge_plate_NEW", "bridge_plate", (0, 0, L["bridge_z"])))
T = L["T"]
for i, (x, y) in enumerate(L["standoffs"]):
    new.append(add("standoff_NEW.%d" % i, "standoff", (x, y, L["bracket_top"])))
    new.append(add("standoff_bolt_top_NEW.%d" % i, "screw_m3x6", (x, y, L["bridge_z"] + T)))
    new.append(add("standoff_bolt_bottom_NEW.%d" % i, "screw_m3x6", (x, y, L["bracket_z"]), flip=True))
new.append(add("pin_NEW", "screw_m3x6", (S[0], S[1], L["bridge_z"] + T)))
for i, (x, y) in enumerate(L["ear_holes"]):
    new.append(add("ear_screw_NEW.%d" % i, "screw_m2x5", (x, y, L["ears_top"] - 2.5), flip=True))
for i, (x, y) in enumerate(L["plate_holes"]):
    new.append(add("plate_screw_NEW.%d" % i, "screw_m2x6", (x, y, L["plate_bottom"]), flip=True))
for i, (x, y) in enumerate(L["gear_spacer_xy"]):
    new.append(add("gear_bolt_NEW.%d" % i, "screw_m2_5x8", (x, y, L["gear30_bottom"]), flip=True))
for i, (x, y) in enumerate(L["gear_spacer_xy"]):
    pass

def bvh(o):
    dg = bpy.context.evaluated_depsgraph_get()
    me = o.evaluated_get(dg).to_mesh()
    me.transform(o.matrix_world)
    return BVHTree.FromPolygons([v.co.copy() for v in me.vertices], [p.vertices[:] for p in me.polygons])

others = [o for o in bpy.data.objects if o.type == "MESH" and o not in new]
print("COLLISIONS (new part vs other part):")
bad = 0
for n in new:
    tn = bvh(n)
    for o in others + [x for x in new if x is not n and x.name > n.name]:
        k = len(tn.overlap(bvh(o)))
        if k:
            print("  ", n.name, "<->", o.name, k, "face pairs")
            bad += 1
print("pairs touching/overlapping:", bad)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(here, "gripper_after_v4.blend"))
