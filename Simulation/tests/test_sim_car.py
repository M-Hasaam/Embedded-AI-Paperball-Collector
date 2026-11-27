import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.car import DROP_S, GRIP_CLOSE, GRIP_RELEASE, PICK_S, SENSOR_OFFSET, CarAPI, Noise, SimCar, wrap  # noqa: E402
from sim.world import Ball, Obstacle, World  # noqa: E402


def make(*obstacles, balls=(), start=(100, 150), heading=0.0, noise=None, seed=1):
    w = World(400, 300, (40, 150), obstacles=list(obstacles), balls=list(balls))
    return SimCar(w, start, heading, noise or Noise.perfect(), seed)


class MotionTest(unittest.TestCase):
    def test_a_perfect_turn_is_exact(self):
        car = make()
        car.turn(90)
        self.assertAlmostEqual(car.h, 90.0, places=6)
        self.assertAlmostEqual(car.heading(), 90.0, places=6)

    def test_a_perfect_drive_goes_the_distance_in_the_time_the_car_expects(self):
        car = make()
        elapsed = car.drive(60)
        self.assertAlmostEqual(elapsed * car.noise.speed_cm_s, 60.0, delta=3.1)       # whole 0.1 s ticks
        self.assertAlmostEqual(car.x, 100 + 60, delta=3.1)
        self.assertAlmostEqual(car.y, 150, places=6)

    def test_driving_backwards(self):
        car = make()
        car.drive(-30)
        self.assertLess(car.x, 100)

    def test_the_gyro_drifts_while_time_passes(self):
        n = Noise.perfect()
        n.gyro_bias_dps = 0.5
        car = make(noise=n, seed=3)
        before = car.heading()
        car._tick(120)
        self.assertGreater(abs(car.heading() - before), 5.0)

    def test_a_turn_follows_the_gyro_so_the_true_heading_carries_the_drift(self):
        n = Noise.perfect()
        n.gyro_bias_dps = 0.5
        car = make(noise=n, seed=3)
        car._tick(120)
        start_read, start_true = car.heading(), car.h
        car.turn(90)
        self.assertAlmostEqual(wrap(car.heading() - start_read), 90.0, places=6)     # what the car reads is right ...
        self.assertAlmostEqual(car.h - start_true, 90.0, delta=1.0)                   # ... and it really turned 90
        self.assertGreater(abs(wrap(car.heading() - car.h)), 5.0)                     # but it reads 5+ degrees more than the truth

    def test_a_wall_stops_the_car_and_counts_a_bump(self):
        n = Noise.perfect()
        n.us_dropout = 1.0                                              # blind: the guard cannot see the wall
        car = make(start=(350, 150), noise=n)
        car.drive(100)
        self.assertEqual(car.bumps, 1)
        self.assertLess(car.x, 400 - car.RADIUS + 0.01)

    def test_the_guard_stops_the_car_before_an_obstacle_without_a_bump(self):
        car = make(Obstacle(200, 100, 220, 200))
        car.drive(200)
        self.assertEqual(car.bumps, 0)
        self.assertLess(car.x, 200 - car.RADIUS)                        # stopped short of it
        self.assertGreater(car.x, 100)

    def test_a_temporary_obstacle_that_appears_while_driving_is_met(self):
        car = make(Obstacle(200, 100, 220, 200, appear=2.0))
        car.drive(200)
        self.assertEqual(car.bumps, 0)
        self.assertLess(car.x, 200 - car.RADIUS)

    def test_veer_bends_the_path(self):
        n = Noise.perfect()
        n.veer_deg_per_cm = 0.2
        car = make(noise=n)
        car.drive(60)
        self.assertGreater(car.y, 150 + 2)

    def test_time_passes(self):
        car = make()
        t0 = car.time()
        car.drive(30)
        self.assertGreater(car.time(), t0 + 0.9)


class UltrasonicTest(unittest.TestCase):
    def test_reads_the_distance_from_the_sensor_not_the_centre(self):
        car = make(Obstacle(200, 100, 220, 200))
        self.assertAlmostEqual(car.ultrasonic(), 100 - SENSOR_OFFSET, delta=0.5)

    def test_nothing_in_range_is_none(self):
        car = make(start=(100, 150))
        self.assertIsNone(car.ultrasonic())                              # the far wall is 300 cm away

    def test_a_thin_chair_leg_is_missed_when_the_sensor_is_unlucky(self):
        n = Noise.perfect()
        n.thin_hit = 0.0
        car = make(Obstacle(150, 148, 153, 152, thin=True), noise=n)
        self.assertIsNone(car.ultrasonic())

    def test_a_thin_chair_leg_is_seen_when_the_sensor_is_lucky(self):
        car = make(Obstacle(150, 148, 153, 152, thin=True))
        self.assertAlmostEqual(car.ultrasonic(), 50 - SENSOR_OFFSET, delta=0.5)

    def test_dropouts_give_none(self):
        n = Noise.perfect()
        n.us_dropout = 1.0
        car = make(Obstacle(200, 100, 220, 200), noise=n)
        self.assertIsNone(car.ultrasonic())


class CameraTest(unittest.TestCase):
    def test_sees_a_ball_in_front(self):
        car = make(balls=[Ball(180, 150)])
        look = car.look()
        self.assertEqual(len(look.balls), 1)
        bearing, dist = look.balls[0]
        self.assertAlmostEqual(bearing, 0.0, places=3)
        self.assertAlmostEqual(dist, 80 - SENSOR_OFFSET, delta=0.5)

    def test_bearing_is_positive_to_the_left(self):
        car = make(balls=[Ball(180, 170)])                              # above the car's line: to its left when it faces +x
        self.assertGreater(car.look().balls[0][0], 5.0)

    def test_does_not_see_outside_its_view(self):
        car = make(balls=[Ball(100, 250)])                              # straight to the left of the car
        self.assertEqual(car.look().balls, [])

    def test_does_not_see_too_far(self):
        car = make(start=(20, 100), balls=[Ball(390, 100)], heading=0.0)
        self.assertEqual(car.look().balls, [])

    def test_a_box_in_the_way_hides_the_ball(self):
        car = make(Obstacle(130, 100, 140, 200), balls=[Ball(180, 150)])
        self.assertEqual(car.look().balls, [])

    def test_a_ball_that_was_picked_is_gone(self):
        car = make(balls=[Ball(180, 150, picked=True)])
        self.assertEqual(car.look().balls, [])

    def test_a_missed_detection_is_possible(self):
        n = Noise.perfect()
        n.cam_detect = 0.0
        car = make(balls=[Ball(180, 150)], noise=n)
        self.assertEqual(car.look().balls, [])

    def test_a_false_ball_can_appear(self):
        n = Noise.perfect()
        n.cam_false_pos = 1.0
        car = make(noise=n)
        self.assertEqual(len(car.look().balls), 1)

    def test_sees_the_bin_tag(self):
        car = make(start=(150, 150), heading=180.0)
        look = car.look()
        self.assertIsNotNone(look.tag)
        self.assertAlmostEqual(look.tag[0], 0.0, places=3)
        self.assertAlmostEqual(look.tag[1], 110 - SENSOR_OFFSET, delta=0.5)

    def test_does_not_see_the_bin_when_facing_away(self):
        car = make(start=(150, 150), heading=0.0)
        self.assertIsNone(car.look().tag)

    def test_the_bin_is_hidden_behind_a_box(self):
        car = make(Obstacle(100, 100, 110, 200), start=(150, 150), heading=180.0)
        self.assertIsNone(car.look().tag)


class GripperTest(unittest.TestCase):
    def test_picks_a_ball_in_front_of_the_jaws(self):
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 16, 150)])
        car.pick()
        self.assertTrue(car.in_jaws())
        self.assertTrue(car.world.balls[0].picked)

    def test_a_ball_too_far_is_not_picked(self):
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 40, 150)])
        car.pick()
        self.assertFalse(car.in_jaws())

    def test_the_pick_can_fail(self):
        n = Noise.perfect()
        n.pick_success = 0.0
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 16, 150)], noise=n)
        car.pick()
        self.assertFalse(car.in_jaws())

    def test_dropping_near_the_bin_delivers(self):
        car = make(start=(95, 150), heading=180.0, balls=[Ball(95 - SENSOR_OFFSET - 16, 150)])
        car.pick()
        car.drop()
        self.assertEqual(car.delivered, 1)
        self.assertFalse(car.in_jaws())

    def test_dropping_far_from_the_bin_leaves_the_ball_on_the_floor(self):
        car = make(start=(300, 150), balls=[Ball(300 + SENSOR_OFFSET + 16, 150)])
        car.pick()
        car.drop()
        self.assertEqual(car.delivered, 0)
        self.assertEqual(sum(1 for b in car.world.balls if not b.picked), 1)


class RecordingTest(unittest.TestCase):
    """What the playback needs to know afterwards."""

    def test_the_path_keeps_the_heading(self):
        car = make()
        car.drive(30)
        car.turn(90)
        x, y, t, h = car.path[-1]
        self.assertAlmostEqual(h, car.h, places=6)
        self.assertEqual(len(car.path[0]), 4)

    def test_a_pick_records_when_the_ball_left_the_floor(self):
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 16, 150)])
        car.pick()
        ball = car.world.balls[0]
        grip = car.grips[0]
        self.assertTrue(grip.start < ball.picked_at < grip.end)                       # it leaves the floor when the jaws close, mid-move
        self.assertAlmostEqual(ball.picked_at, grip.start + GRIP_CLOSE * (grip.end - grip.start))

    def test_a_delivery_records_when(self):
        car = make(start=(95, 150), heading=180.0, balls=[Ball(95 - SENSOR_OFFSET - 16, 150)])
        car.pick()
        car.drop()
        grip = car.grips[-1]
        self.assertAlmostEqual(car.world.balls[0].delivered_at, grip.start + GRIP_RELEASE * (grip.end - grip.start))

    def test_a_ball_dropped_on_the_floor_appears_when_it_falls(self):
        car = make(start=(300, 150), balls=[Ball(300 + SENSOR_OFFSET + 16, 150)])
        car.pick()
        car.drop()
        fallen = car.world.balls[-1]
        grip = car.grips[-1]
        self.assertAlmostEqual(fallen.created_at, grip.start + GRIP_RELEASE * (grip.end - grip.start))
        self.assertIsNone(fallen.picked_at)

    def test_a_bump_is_an_event_with_a_time_and_place(self):
        n = Noise.perfect()
        n.us_dropout = 1.0
        car = make(start=(350, 150), noise=n)
        car.drive(100)
        bumps = [e for e in car.events if e[0] == "bump"]
        self.assertEqual(len(bumps), 1)
        self.assertGreater(bumps[0][1], 0.0)

    def test_it_keeps_account_of_where_the_time_goes(self):
        car = make(Obstacle(300, 100, 310, 200))
        car.turn(90)
        car.drive(30)
        car.look()
        car.ultrasonic()
        for key in ("turn", "drive", "look", "ultrasonic"):
            self.assertGreater(car.spent[key], 0.0, key)
        self.assertAlmostEqual(sum(car.spent.values()), car.time(), places=6)

    def test_an_obstacle_can_have_a_name(self):
        self.assertEqual(Obstacle(0, 0, 1, 1, label="Table").label, "Table")
        self.assertEqual(Obstacle(0, 0, 1, 1).label, "")


class TimingTest(unittest.TestCase):
    def test_a_turn_takes_its_start_up_time_plus_the_time_to_swing(self):
        n = Noise.perfect()
        n.turn_start_s, n.turn_rate_dps = 0.2, 100.0
        car = make(noise=n)
        car.turn(50)
        self.assertAlmostEqual(car.time(), 0.2 + 0.5, places=6)

    def test_a_camera_picture_takes_the_time_it_is_set_to(self):
        n = Noise.perfect()
        n.look_s = 0.1
        car = make(noise=n)
        car.look()
        self.assertAlmostEqual(car.time(), 0.1, places=6)

    def test_the_defaults_are_the_old_fixed_numbers(self):
        n = Noise()
        self.assertEqual((n.turn_start_s, n.turn_rate_dps, n.look_s), (0.15, 120.0, 0.3))


class ArmMoveTest(unittest.TestCase):
    """Each pick and each release is a timed arm move the playback can show: when it started, when it ended, what it aimed at."""

    def test_a_pick_is_recorded_with_its_time_and_target(self):
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 16, 150)])
        t0 = car.time()
        car.pick()
        g = car.grips[0]
        self.assertEqual((g.kind, g.ok), ("pick", True))
        self.assertAlmostEqual(g.start, t0)
        self.assertAlmostEqual(g.end - g.start, PICK_S)
        self.assertAlmostEqual(g.target[0], 100 + SENSOR_OFFSET + 16)               # it reached for the ball
        self.assertAlmostEqual(g.target[1], 150.0)

    def test_a_missed_pick_is_recorded_as_a_miss(self):
        n = Noise.perfect()
        n.pick_success = 0.0
        car = make(balls=[Ball(100 + SENSOR_OFFSET + 16, 150)], noise=n)
        car.pick()
        self.assertEqual((car.grips[0].kind, car.grips[0].ok), ("pick", False))

    def test_a_pick_with_no_ball_in_reach_still_reaches_out(self):
        car = make()
        car.pick()
        self.assertFalse(car.grips[0].ok)
        self.assertGreater(car.grips[0].target[0], 100)                             # it aimed at the spot ahead of the jaws

    def test_a_release_over_the_bin_is_recorded_with_the_bin_as_target(self):
        car = make(start=(95, 150), heading=180.0, balls=[Ball(95 - SENSOR_OFFSET - 16, 150)])
        car.pick()
        car.drop()
        g = car.grips[-1]
        self.assertEqual((g.kind, g.ok), ("drop", True))
        self.assertAlmostEqual(g.end - g.start, DROP_S)
        self.assertEqual(g.target, car.world.bin_xy)

    def test_a_release_away_from_the_bin_is_recorded_as_failed(self):
        car = make(start=(300, 150), balls=[Ball(300 + SENSOR_OFFSET + 16, 150)])
        car.pick()
        car.drop()
        self.assertEqual((car.grips[-1].kind, car.grips[-1].ok), ("drop", False))

    def test_nothing_is_recorded_when_there_is_nothing_in_the_gripper_to_release(self):
        car = make()
        car.drop()
        self.assertEqual([g for g in car.grips if g.kind == "drop"], [])

    def test_the_moves_never_overlap_and_follow_each_other(self):
        car = make(start=(95, 150), heading=180.0, balls=[Ball(95 - SENSOR_OFFSET - 16, 150)])
        car.pick()
        car.drop()
        for a, b in zip(car.grips, car.grips[1:]):
            self.assertLessEqual(a.end, b.start + 1e-9)


class NoEncoderTest(unittest.TestCase):
    def test_the_brain_gets_no_position_and_no_encoder(self):
        api = CarAPI(make())
        for name in ("x", "y", "h", "world", "encoder", "ticks", "odometry", "pose", "gyro_err", "bumps"):
            self.assertFalse(hasattr(api, name), name)

    def test_the_api_offers_only_what_the_real_car_has(self):
        api = CarAPI(make())
        for name in ("time", "heading", "turn", "drive", "ultrasonic", "look", "in_jaws", "pick", "drop"):
            self.assertTrue(callable(getattr(api, name)), name)


if __name__ == "__main__":
    unittest.main()
