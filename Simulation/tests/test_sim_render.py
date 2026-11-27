import math
import sys
import unittest
from pathlib import Path

from PIL import ImageChops

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim.car import GRIP_CLOSE, Noise  # noqa: E402
from sim.render import ARM_FOLDED, CAMERA_VIEW, ROUTE_COLOUR, CLOSEUP_PX, camera_polygon, JAW, JAW_CLOSED, JAW_OPEN, arm_caption, arm_pose, render_closeup, render_frame  # noqa: E402
from sim.run import run_episode  # noqa: E402
from sim.timeline import Arm  # noqa: E402


def pick(progress, ok=True):
    return Arm("pick", progress, ok, (0, 0))


def drop(progress, ok=True):
    return Arm("drop", progress, ok, (0, 0))


class PoseTest(unittest.TestCase):
    """How far out the arm is, how wide the jaws are open, and where the ball is, at each moment of a pick or a release."""

    def test_a_pick_starts_folded_with_the_jaws_open_and_no_ball(self):
        p = arm_pose(pick(0.0), reach=24.0)
        self.assertAlmostEqual(p.length, ARM_FOLDED)
        self.assertAlmostEqual(p.jaw, JAW_OPEN)
        self.assertEqual(p.ball, "floor")

    def test_it_reaches_the_ball_before_it_closes_on_it(self):
        p = arm_pose(pick(0.35), reach=24.0)
        self.assertAlmostEqual(p.length, 24.0)
        self.assertAlmostEqual(p.jaw, JAW_OPEN)

    def test_the_jaws_close_and_the_ball_is_then_in_the_gripper(self):
        before, after = arm_pose(pick(GRIP_CLOSE - 0.02), 24.0), arm_pose(pick(GRIP_CLOSE + 0.02), 24.0)
        self.assertEqual(before.ball, "floor")
        self.assertEqual(after.ball, "gripper")
        self.assertLess(after.jaw, before.jaw)

    def test_it_lifts_the_ball_back_to_the_folded_position(self):
        p = arm_pose(pick(1.0), 24.0)
        self.assertAlmostEqual(p.length, ARM_FOLDED, places=3)
        self.assertEqual(p.ball, "gripper")
        self.assertAlmostEqual(p.jaw, JAW_CLOSED)

    def test_a_missed_pick_closes_on_nothing(self):
        p = arm_pose(pick(0.9, ok=False), 24.0)
        self.assertEqual(p.ball, "none")
        self.assertAlmostEqual(p.jaw, JAW_CLOSED)

    def test_a_release_starts_with_the_ball_in_the_gripper_and_reaches_out(self):
        start, reach = arm_pose(drop(0.0), 28.0), arm_pose(drop(0.4), 28.0)
        self.assertEqual(start.ball, "gripper")
        self.assertAlmostEqual(start.length, ARM_FOLDED)
        self.assertAlmostEqual(reach.length, 28.0)
        self.assertAlmostEqual(reach.jaw, JAW_CLOSED)

    def test_the_jaws_open_and_the_ball_falls_into_the_bin(self):
        opening = arm_pose(drop(0.5), 28.0)
        falling = arm_pose(drop(0.7), 28.0)
        self.assertGreater(opening.jaw, JAW_CLOSED)
        self.assertEqual(falling.ball, "falling")
        self.assertTrue(0.0 < falling.fall <= 1.0)
        self.assertEqual(arm_pose(drop(0.95), 28.0).ball, "none")

    def test_the_arm_folds_back_after_the_release(self):
        p = arm_pose(drop(1.0), 28.0)
        self.assertAlmostEqual(p.length, ARM_FOLDED, places=3)

    def test_a_ball_that_misses_the_bin_just_drops_to_the_floor(self):
        p = arm_pose(drop(0.7, ok=False), 28.0)
        self.assertEqual(p.ball, "none")                       # it is a floor ball from the moment of release

    def test_the_length_never_goes_beyond_the_reach_or_below_folded(self):
        for kind in (pick, drop):
            for i in range(0, 101):
                p = arm_pose(kind(i / 100), reach=24.0)
                self.assertTrue(ARM_FOLDED - 1e-9 <= p.length <= 24.0 + 1e-9, (kind.__name__, i, p.length))
                self.assertTrue(JAW_CLOSED - 1e-9 <= p.jaw <= JAW_OPEN + 1e-9)


class CaptionTest(unittest.TestCase):
    def test_every_stage_is_described_in_plain_words(self):
        seen = {arm_caption(pick(x)) for x in (0.1, 0.45, 0.8)} | {arm_caption(drop(x)) for x in (0.1, 0.5, 0.85)}
        self.assertEqual(len(seen), 6)                          # six different stages, six different sentences
        for text in seen:
            self.assertTrue(text.startswith("Arm:"), text)

    def test_a_miss_says_so(self):
        self.assertIn("miss", arm_caption(pick(0.8, ok=False)).lower())

    def test_no_arm_move_no_caption(self):
        self.assertEqual(arm_caption(None), "")


class PictureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 300, Noise.perfect())
        cls.pick = [g for g in cls.res.car.grips if g.kind == "pick" and g.ok][0]
        cls.drop = [g for g in cls.res.car.grips if g.kind == "drop" and g.ok][0]

    def at(self, grip, progress):
        return render_frame(self.res, grip.start + progress * (grip.end - grip.start))

    def test_the_picture_changes_as_the_arm_reaches_out(self):
        folded, extended = self.at(self.pick, 0.0), self.at(self.pick, 0.35)
        self.assertIsNotNone(ImageChops.difference(folded.convert("RGB"), extended.convert("RGB")).getbbox())

    def test_the_picture_changes_as_the_jaws_close_and_lift(self):
        extended, lifting = self.at(self.pick, 0.35), self.at(self.pick, 0.8)
        self.assertIsNotNone(ImageChops.difference(extended.convert("RGB"), lifting.convert("RGB")).getbbox())

    def test_every_moment_of_a_pick_and_a_release_can_be_drawn(self):
        for grip in (self.pick, self.drop):
            for i in range(0, 21):
                img = self.at(grip, min(i / 20, 0.999))
                self.assertEqual(img.size, render_frame(self.res, 0.0).size)

    def test_the_arm_is_drawn_only_while_it_moves(self):
        # just before and just after the move the car is the same folded-arm car: the two pictures differ only in time-dependent bits
        before = render_frame(self.res, self.pick.start - 0.01)
        mid = self.at(self.pick, 0.35)
        self.assertIsNotNone(ImageChops.difference(before.convert("RGB"), mid.convert("RGB")).getbbox())


class CloseupTest(unittest.TestCase):
    """While the arm moves, a magnified view of the car and the ball shows in a corner, because the arm is small in the whole arena."""

    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 300, Noise.perfect())
        cls.pick = [g for g in cls.res.car.grips if g.kind == "pick" and g.ok][0]
        cls.drop = [g for g in cls.res.car.grips if g.kind == "drop" and g.ok][0]

    def mid(self, g, p=0.45):
        return g.start + p * (g.end - g.start)

    def test_there_is_a_closeup_during_an_arm_move_and_not_otherwise(self):
        self.assertIsNotNone(render_closeup(self.res, self.mid(self.pick)))
        self.assertIsNotNone(render_closeup(self.res, self.mid(self.drop)))
        self.assertIsNone(render_closeup(self.res, 1.0))

    def test_it_is_a_square_of_the_set_size(self):
        img = render_closeup(self.res, self.mid(self.pick))
        self.assertEqual(img.size, (CLOSEUP_PX, CLOSEUP_PX))

    def test_the_car_and_the_open_jaws_are_in_it(self):
        img = render_closeup(self.res, self.mid(self.pick, 0.3)).convert("RGB")
        pixels = list(img.getdata())
        near = lambda c, t: all(abs(a - b) < 30 for a, b in zip(c, t))      # noqa: E731
        self.assertTrue(any(near(px, (250, 204, 21)) for px in pixels), "the car's body")
        self.assertTrue(any(near(px, JAW) for px in pixels), "the jaws")

    def test_the_jaws_are_much_bigger_in_the_closeup_than_in_the_arena_view(self):
        mid = self.mid(self.pick, 0.3)
        close = render_closeup(self.res, mid).convert("RGB")
        whole = render_frame(self.res, mid).convert("RGB")
        whole = whole.crop((0, 0, whole.width - CLOSEUP_PX - 14, whole.height))          # the arena view without the close-up in its corner
        count = lambda im: sum(1 for px in im.getdata() if all(abs(a - b) < 30 for a, b in zip(px, JAW)))      # noqa: E731
        self.assertGreater(count(close), 1.5 * count(whole))

    def test_the_main_picture_has_the_closeup_pasted_in_a_corner_during_a_move(self):
        mid = self.mid(self.pick)
        with_arm = render_frame(self.res, mid).convert("RGB")
        w, h = with_arm.size
        inset = render_closeup(self.res, mid).convert("RGB")
        # the top-right corner of the picture is the closeup (somewhere inside it matches pixel for pixel)
        box = (w - CLOSEUP_PX - 14, 14, w - 14, 14 + CLOSEUP_PX)
        corner = with_arm.crop(box)
        self.assertEqual(corner.getpixel((CLOSEUP_PX // 2, CLOSEUP_PX // 2)), inset.getpixel((CLOSEUP_PX // 2, CLOSEUP_PX // 2)))

    def test_no_closeup_in_the_overview(self):
        w, h = render_frame(self.res, None).size
        self.assertEqual((w, h), render_frame(self.res, 0.0).size)


class CameraViewTest(unittest.TestCase):
    """The camera sees a narrow wedge straight ahead (36 degrees, about 2.2 m): drawing it shows what the car can and cannot see."""

    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 120, Noise.perfect())

    def count(self, img, colour=CAMERA_VIEW, tol=40):
        return sum(1 for px in img.convert("RGB").getdata() if all(abs(a - b) < tol for a, b in zip(px, colour)))

    def test_it_is_drawn_only_when_asked_for(self):
        off = render_frame(self.res, 5.0, show_camera=False)
        on = render_frame(self.res, 5.0, show_camera=True)
        self.assertIsNotNone(ImageChops.difference(off.convert("RGB"), on.convert("RGB")).getbbox())
        self.assertEqual(ImageChops.difference(off.convert("RGB"), render_frame(self.res, 5.0).convert("RGB")).getbbox(), None)   # off by default

    def test_the_wedge_covers_a_large_area_and_leans_blue(self):
        on = render_frame(self.res, 0.0, show_camera=True).convert("RGB")          # at the start the car faces open floor
        off = render_frame(self.res, 0.0, show_camera=False).convert("RGB")
        changed = [(a, b) for a, b in zip(on.getdata(), off.getdata()) if a != b]
        self.assertGreater(len(changed), 5000)                                     # a wedge 2.2 m long is a big part of the picture
        bluer = sum(1 for a, b in changed if (a[2] - a[0]) > (b[2] - b[0]))        # and it makes what it covers bluer
        self.assertGreater(bluer, 0.75 * len(changed))

    def test_the_wedge_is_not_drawn_in_the_overview(self):
        a, b = render_frame(self.res, None, show_camera=True), render_frame(self.res, None)
        self.assertIsNone(ImageChops.difference(a.convert("RGB"), b.convert("RGB")).getbbox())

    def test_it_is_as_long_as_the_cameras_range(self):
        from sim.car import CAM_HALF_FOV, CAM_RANGE
        from sim.render import camera_wedge
        a, b, radius = camera_wedge()
        self.assertEqual(radius, CAM_RANGE)
        self.assertAlmostEqual(b - a, 2 * CAM_HALF_FOV)


class CameraPolygonTest(unittest.TestCase):
    """The camera cannot see through things: its drawn view must stop at the first wall, obstacle or bin."""

    def world(self, *obstacles):
        from sim.world import World
        return World(400, 300, (40, 150), obstacles=list(obstacles))

    def far(self, poly, x, y):
        return max(math.hypot(px - x, py - y) for px, py in poly)

    def test_in_the_open_it_reaches_the_cameras_range(self):
        from sim.car import CAM_RANGE
        poly = camera_polygon(self.world(), 100, 150, 0.0, 0.0)
        self.assertAlmostEqual(self.far(poly, 100, 150), CAM_RANGE, delta=1.0)

    def test_it_starts_at_the_camera(self):
        poly = camera_polygon(self.world(), 100, 150, 0.0, 0.0)
        self.assertEqual(poly[0], (100, 150))

    def test_a_wall_ends_it(self):
        poly = camera_polygon(self.world(), 300, 150, 0.0, 0.0)                    # 100 cm from the east wall
        self.assertLessEqual(max(px for px, py in poly), 400.0 + 1e-6)
        self.assertAlmostEqual(self.far(poly, 300, 150), 100.0 / math.cos(math.radians(18.0)), delta=1.0)     # the edges slant to the wall

    def test_an_obstacle_ends_it_and_what_is_behind_is_in_shadow(self):
        from sim.world import Obstacle
        poly = camera_polygon(self.world(Obstacle(200, 100, 220, 200)), 100, 150, 0.0, 0.0)
        self.assertLessEqual(max(px for px, py in poly), 200.0 + 1e-6)

    def test_an_obstacle_that_has_not_arrived_does_not_block_it(self):
        from sim.world import Obstacle
        w = self.world(Obstacle(200, 100, 220, 200, appear=50))
        self.assertGreater(max(px for px, py in camera_polygon(w, 100, 150, 0.0, 10.0)), 250.0)
        self.assertLessEqual(max(px for px, py in camera_polygon(w, 100, 150, 0.0, 60.0)), 200.0 + 1e-6)

    def test_a_ball_does_not_block_the_view_of_what_is_behind_it(self):
        from sim.world import Ball
        w = self.world()
        w.balls.append(Ball(160, 150))
        self.assertGreater(self.far(camera_polygon(w, 100, 150, 0.0, 0.0), 100, 150), 200.0)

    def test_it_is_36_degrees_wide(self):
        poly = camera_polygon(self.world(), 100, 150, 0.0, 0.0)
        angles = [math.degrees(math.atan2(py - 150, px - 100)) for px, py in poly[1:]]
        self.assertAlmostEqual(min(angles), -18.0, delta=0.5)
        self.assertAlmostEqual(max(angles), 18.0, delta=0.5)


class RouteDrawingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lanes = run_episode("empty", 0, 200, Noise.perfect(), 3, strategy="lanes")

    def test_the_route_is_drawn_only_when_asked_for(self):
        off = render_frame(self.lanes, 30.0, show_route=False).convert("RGB")
        on = render_frame(self.lanes, 30.0, show_route=True).convert("RGB")
        self.assertIsNotNone(ImageChops.difference(off, on).getbbox())
        self.assertIsNone(ImageChops.difference(off, render_frame(self.lanes, 30.0).convert("RGB")).getbbox())           # off by default

    def test_it_is_drawn_across_the_arena(self):
        off = render_frame(self.lanes, 30.0).convert("RGB")
        on = render_frame(self.lanes, 30.0, show_route=True).convert("RGB")
        box = ImageChops.difference(off, on).getbbox()
        self.assertGreater(box[2] - box[0], 400)                                   # lanes run most of the way across

    def test_nothing_changes_for_the_map_search(self):
        mapped = run_episode("empty", 0, 60, Noise.perfect(), 3)
        a = render_frame(mapped, 30.0, show_route=True).convert("RGB")
        b = render_frame(mapped, 30.0).convert("RGB")
        self.assertIsNone(ImageChops.difference(a, b).getbbox())

    def test_it_is_not_drawn_in_the_overview(self):
        a = render_frame(self.lanes, None, show_route=True).convert("RGB")
        b = render_frame(self.lanes, None).convert("RGB")
        self.assertIsNone(ImageChops.difference(a, b).getbbox())


class SeenOverlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 200, Noise.perfect(), 3)

    def test_it_is_drawn_only_when_asked_for(self):
        off = render_frame(self.res, 150.0, show_seen=False).convert("RGB")
        on = render_frame(self.res, 150.0, show_seen=True).convert("RGB")
        self.assertIsNotNone(ImageChops.difference(off, on).getbbox())
        self.assertIsNone(ImageChops.difference(off, render_frame(self.res, 150.0).convert("RGB")).getbbox())            # off by default

    def test_it_covers_a_big_part_of_the_arena_late_in_the_run(self):
        off = render_frame(self.res, 150.0).convert("RGB")
        on = render_frame(self.res, 150.0, show_seen=True).convert("RGB")
        changed = sum(1 for a, b in zip(on.getdata(), off.getdata()) if a != b)
        self.assertGreater(changed, 20000)

    def test_nothing_is_drawn_at_the_very_start(self):
        a = render_frame(self.res, 0.0, show_seen=True).convert("RGB")
        b = render_frame(self.res, 0.0).convert("RGB")
        box = ImageChops.difference(a, b).getbbox()                  # only the status chip differs (it says 'Looked at 0%')
        self.assertTrue(box is None or box[1] < 70 and box[3] < 70, box)

    def test_it_is_not_drawn_in_the_overview(self):
        a = render_frame(self.res, None, show_seen=True).convert("RGB")
        b = render_frame(self.res, None).convert("RGB")
        self.assertIsNone(ImageChops.difference(a, b).getbbox())

    def test_it_makes_what_it_covers_greener(self):
        off = render_frame(self.res, 150.0).convert("RGB")
        on = render_frame(self.res, 150.0, show_seen=True).convert("RGB")
        changed = [(a, b) for a, b in zip(on.getdata(), off.getdata()) if a != b]
        greener = sum(1 for a, b in changed if (a[1] - a[0]) > (b[1] - b[0]))
        self.assertGreater(greener, 0.8 * len(changed))


class BeliefOverlayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from sim.world import Ball, Obstacle
        from test_sim_timeline import custom_result
        cls.res = custom_result([Ball(150, 150), Ball(150, 100)], [Obstacle(250, 200, 290, 260)])
        first_pick = [e for e in cls.res.car.events if e[0] == "pick"][0][1]
        cls.t_waiting = first_pick + 1.0

    def test_a_waiting_ball_is_marked_when_asked_for(self):
        off = render_frame(self.res, self.t_waiting, show_pending=False).convert("RGB")
        on = render_frame(self.res, self.t_waiting, show_pending=True).convert("RGB")
        self.assertIsNotNone(ImageChops.difference(off, on).getbbox())
        self.assertIsNone(ImageChops.difference(off, render_frame(self.res, self.t_waiting).convert("RGB")).getbbox())        # off by default

    def test_the_mark_is_blue(self):
        off = render_frame(self.res, self.t_waiting).convert("RGB")
        on = render_frame(self.res, self.t_waiting, show_pending=True).convert("RGB")
        changed = [(a, b) for a, b in zip(on.getdata(), off.getdata()) if a != b]
        self.assertGreater(len(changed), 100)
        bluer = sum(1 for a, b in changed if (a[2] - a[0]) > (b[2] - b[0]))
        self.assertGreater(bluer, 0.8 * len(changed))

    def test_obstacles_it_has_found_are_tinted_red_when_asked_for(self):
        off = render_frame(self.res, self.res.seconds - 1).convert("RGB")
        on = render_frame(self.res, self.res.seconds - 1, show_blocked=True).convert("RGB")
        changed = [(a, b) for a, b in zip(on.getdata(), off.getdata()) if a != b]
        self.assertGreater(len(changed), 500)
        redder = sum(1 for a, b in changed if (a[0] - a[2]) > (b[0] - b[2]))
        self.assertGreater(redder, 0.8 * len(changed))

    def test_neither_is_drawn_in_the_overview(self):
        a = render_frame(self.res, None, show_pending=True, show_blocked=True).convert("RGB")
        b = render_frame(self.res, None).convert("RGB")
        self.assertIsNone(ImageChops.difference(a, b).getbbox())


if __name__ == "__main__":
    unittest.main()
