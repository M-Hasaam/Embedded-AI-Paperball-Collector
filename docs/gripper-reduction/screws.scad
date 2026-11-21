// Fasteners for the gripper (millimetres). Origin: the screw axis; z = 0 is the UNDERSIDE of the head, the shank
// points down (-z), so a screw drops through a plate just as it is. Nuts and washers have z = 0 as their bottom face.
// Change the numbers to match your hardware.

module pan_head_screw(d, length, head_d, head_h) {
    // round (pan) head with a cross slot, plain shank
    difference() {
        union() {
            translate([0, 0, 0]) cylinder(d = head_d, h = head_h, $fn = 40);
            translate([0, 0, -length]) cylinder(d = d, h = length + 0.01, $fn = 24);
        }
        translate([-head_d * 0.35, -0.3, head_h - 0.9]) cube([head_d * 0.7, 0.6, 1]);
        translate([-0.3, -head_d * 0.35, head_h - 0.9]) cube([0.6, head_d * 0.7, 1]);
    }
}

module hex_nut(af, h, hole_d) {
    difference() {
        cylinder(d = af / cos(30), h = h, $fn = 6);
        translate([0, 0, -1]) cylinder(d = hole_d, h = h + 2, $fn = 24);
    }
}

module washer(outer_d, hole_d, h) {
    difference() {
        cylinder(d = outer_d, h = h, $fn = 40);
        translate([0, 0, -1]) cylinder(d = hole_d, h = h + 2, $fn = 24);
    }
}

which = "m3x12";
if (which == "m3x12") pan_head_screw(3, 12, 5.6, 2.0);       // jaw / gear pivots ("12 mm" screw in the assembly photo)
if (which == "m2x8")  pan_head_screw(2, 8, 3.8, 1.5);        // servo mounting / horn
if (which == "nut_m3") hex_nut(5.5, 2.4, 3);
if (which == "nut_m2") hex_nut(4, 1.6, 2);
if (which == "washer_m3") washer(7, 3.2, 0.5);
if (which == "m3x6")  pan_head_screw(3, 6, 5.6, 2.0);        // bridge standoff bolts
if (which == "m3x10") pan_head_screw(3, 10, 5.6, 2.0);       // the pin through the big gear's centre
if (which == "m2_5x12") pan_head_screw(2.5, 12, 4.6, 1.5);   // gear 30 / gear 29 / big gear bolts
if (which == "nut_m2_5") hex_nut(5, 2.0, 2.5);
if (which == "standoff_m3x15") hex_nut(5.5, 15, 3);          // female-female hex standoff
if (which == "m3x14") pan_head_screw(3, 14, 5.6, 2.0);       // the pin through the big gear's centre
if (which == "m2_5x16") pan_head_screw(2.5, 16, 4.6, 1.5);   // gear 30 / gear 29 / big gear bolts
if (which == "standoff_m3x20") hex_nut(5.5, 20, 3);
if (which == "m3x8") pan_head_screw(3, 8, 5.6, 2.0);
if (which == "standoff_m3x14") hex_nut(5.5, 14, 3);
standoff_h = 12;
if (which == "m2x5") pan_head_screw(2, 5, 3.8, 1.5);        // servo ears -> bracket (self-tapping into a 1.6 mm hole)
if (which == "m2x6") pan_head_screw(2, 6, 3.8, 1.5);        // plate 24 -> bracket
if (which == "m2_5x8") pan_head_screw(2.5, 8, 4.6, 1.5);    // gear 30/29 -> big gear (self-tapping into 2.1 mm holes)
if (which == "standoff") hex_nut(5.5, standoff_h, 3);        // threaded hex standoff
