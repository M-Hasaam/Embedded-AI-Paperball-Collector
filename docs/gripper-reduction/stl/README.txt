Gripper parts as 3D files (millimetres, 3 mm thick, lying flat). Import them all into a visual editor
(Blender: File > Import > STL, select all) and drag them into place.

Current gripper: sheet parts 29, 30, 28, 27, 23, 22, 24, 26, 21, 25 (traced from docs/MeArmV1.0.dxf)
  part_28.stl, part_27.stl   the two jaws with their gears (origin = gear centre)
  part_29.stl, part_30.stl   gears with 5-6 teeth (origin = gear / hub centre)
  part_26.stl                8-tooth gear with two big holes
  part_24.stl, part_21.stl   plates with a servo window
  part_25.stl, part_23.stl, part_22.stl   small plates / brackets
  (origin = the middle of the part unless noted)
  sg90_servo.stl                  approximate SG90, shaft at the origin

Fasteners (axis at the origin; z = 0 is the underside of the head, the shank points down)
  screw_m3x12.stl   pan-head M3 x 12 (jaw / gear pivots)     nut_m3.stl   washer_m3.stl
  screw_m2x8.stl    pan-head M2 x 8 (servo mounting)         nut_m2.stl
  numbers are in screws.scad (one level up) if your hardware differs

New reduction, v4 (8T pinion on the servo shaft -> 24T gear on gear 29, 3:1 extra, module 1.5 mm, centre distance 24 mm)
  pinion.stl, big_gear.stl, pinion_hub.stl   the gears and the hub disc under the pinion (3 mm)
  servo_bracket.stl                holds the servo by its ears; lies on plate 24 (two NEW 2.2 mm holes in its left strip)
  bridge_plate.stl                 3 mm plate on two 11.5 mm standoffs; an M3x6 pin through the big gear's centre
  standoff.stl, screw_m3x6.stl, screw_m2x5.stl, screw_m2x6.stl, screw_m2_5x8.stl
  No nuts: the screws cut their own thread in undersized holes (M2 1.6 mm, M2.5 2.1 mm, M3 2.5 mm).
  Positions: layout.json; rebuild with build_after.py + make_after_blend.py (Blender), output gripper_after_v4.blend.
