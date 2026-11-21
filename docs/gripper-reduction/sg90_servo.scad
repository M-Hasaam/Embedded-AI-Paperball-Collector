// SG90 / MG90 micro servo (millimetres). Typical SG90 dimensions, checked against the photo of a real one and the
// drawing in docs/servos/mg90-servo-motor.pdf (22.5 x 12 case, 32.5 over the ears).
// Origin: the output shaft axis, z = 0 is the TOP face of the mounting ears. The case hangs below (to z = -18.5)
// and continues about 4 mm above the ears. Cheap clones differ by up to about 0.5 mm: measure yours with callipers
// and change the numbers below.

BODY_L = 22.5;          // case length
BODY_W = 12;            // case width
EARS_L = 32.5;          // length over the mounting ears
EAR_T = 2.5;            // ear thickness
BELOW_EARS = 16;        // bottom of the case to the underside of the ears
ABOVE_EARS = 4.2;       // top of the case above the top face of the ears
SHAFT_FROM_END = 6;     // shaft axis to the nearest end of the case (the end with the closed ear hole)
EAR_HOLE_D = 2.0;       // ear screw holes
EAR_HOLE_SPACING = 27.5;
BOSS_D = 11.5;  BOSS_H = 3.5;       // round housing round the output shaft, on top of the case
SMALL_BOSS_D = 5.5; SMALL_BOSS_OFFSET = 5.5;   // the second, smaller bump next to it (towards the middle of the case)
SHAFT_D = 4.8; SHAFT_H = 11.5;      // splined output shaft, measured from z = 0

module servo() {
    cx = -(BODY_L / 2 - SHAFT_FROM_END);          // case centre, relative to the shaft
    difference() {
        union() {
            // case: below the ears, through the ears, and the cap above them
            translate([cx - BODY_L / 2, -BODY_W / 2, -(BELOW_EARS + EAR_T)]) cube([BODY_L, BODY_W, BELOW_EARS + EAR_T + ABOVE_EARS]);
            // ears
            translate([cx - EARS_L / 2, -BODY_W / 2, -EAR_T]) cube([EARS_L, BODY_W, EAR_T]);
            // bosses on top of the case
            translate([0, 0, ABOVE_EARS - 0.01]) cylinder(d = BOSS_D, h = BOSS_H, $fn = 48);
            translate([-SMALL_BOSS_OFFSET, 0, ABOVE_EARS - 0.01]) cylinder(d = SMALL_BOSS_D, h = BOSS_H - 0.7, $fn = 32);
            // output shaft
            cylinder(d = SHAFT_D, h = SHAFT_H, $fn = 30);
        }
        // ear screw holes: a round hole at the shaft end, an open slot at the other end
        translate([cx + EAR_HOLE_SPACING / 2, 0, -EAR_T - 1]) cylinder(d = EAR_HOLE_D, h = EAR_T + 2, $fn = 20);
        hull() {
            translate([cx - EAR_HOLE_SPACING / 2, 0, -EAR_T - 1]) cylinder(d = EAR_HOLE_D, h = EAR_T + 2, $fn = 20);
            translate([cx - EARS_L / 2 - 1, 0, -EAR_T - 1]) cylinder(d = EAR_HOLE_D, h = EAR_T + 2, $fn = 20);
        }
    }
}

servo();
