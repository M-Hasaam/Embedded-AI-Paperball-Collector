import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.world import Ball, Obstacle, World  # noqa: E402


def world(*obstacles, balls=()):
    return World(400, 300, (40, 150), obstacles=list(obstacles), balls=list(balls))


class RayTest(unittest.TestCase):
    def test_a_ray_hits_the_wall_in_front(self):
        dist, kind = world().cast(100, 150, 0, t=0)
        self.assertAlmostEqual(dist, 300.0)
        self.assertEqual(kind, "wall")

    def test_a_ray_in_every_direction_ends_at_a_wall(self):
        w = world()
        self.assertAlmostEqual(w.cast(100, 250, 90, 0)[0], 50.0)
        self.assertAlmostEqual(w.cast(100, 250, 180, 0)[0], 100.0)
        self.assertAlmostEqual(w.cast(100, 250, -90, 0)[0], 250.0)

    def test_a_box_in_front_is_hit_before_the_wall(self):
        dist, kind = world(Obstacle(200, 100, 220, 200)).cast(100, 150, 0, 0)
        self.assertAlmostEqual(dist, 100.0)
        self.assertEqual(kind, "obstacle")

    def test_a_box_beside_the_ray_is_missed(self):
        dist, kind = world(Obstacle(200, 200, 220, 250)).cast(100, 150, 0, 0)
        self.assertEqual(kind, "wall")

    def test_the_bin_is_hit(self):
        dist, kind = world().cast(100, 150, 180, 0)
        self.assertEqual(kind, "bin")
        self.assertAlmostEqual(dist, 100 - 40 - 15)

    def test_a_ball_is_hit_unless_skipped(self):
        w = world(balls=[Ball(150, 150)])
        self.assertEqual(w.cast(100, 150, 0, 0)[1], "ball")
        self.assertEqual(w.cast(100, 150, 0, 0, skip=("ball",))[1], "wall")

    def test_a_picked_ball_is_not_there(self):
        w = world(balls=[Ball(150, 150, picked=True)])
        self.assertEqual(w.cast(100, 150, 0, 0)[1], "wall")

    def test_a_thin_obstacle_has_its_own_kind(self):
        w = world(Obstacle(200, 149, 203, 152, thin=True))
        self.assertEqual(w.cast(100, 150, 0, 0)[1], "thin")


class TemporaryObstacleTest(unittest.TestCase):
    def test_it_exists_only_between_appear_and_vanish(self):
        w = world(Obstacle(200, 100, 220, 200, appear=50, vanish=120))
        self.assertEqual(w.cast(100, 150, 0, t=10)[1], "wall")
        self.assertEqual(w.cast(100, 150, 0, t=60)[1], "obstacle")
        self.assertEqual(w.cast(100, 150, 0, t=130)[1], "wall")

    def test_it_blocks_the_car_only_while_it_is_there(self):
        w = world(Obstacle(200, 100, 220, 200, appear=50, vanish=120))
        self.assertFalse(w.blocked(205, 150, 10, t=10))
        self.assertTrue(w.blocked(205, 150, 10, t=60))

    def test_permanent_and_temporary_are_told_apart(self):
        self.assertTrue(Obstacle(0, 0, 1, 1).permanent)
        self.assertFalse(Obstacle(0, 0, 1, 1, appear=5).permanent)
        self.assertFalse(Obstacle(0, 0, 1, 1, vanish=5).permanent)


class BlockedTest(unittest.TestCase):
    def test_walls_block_a_car_that_would_poke_through(self):
        w = world()
        self.assertTrue(w.blocked(5, 150, 10, 0))
        self.assertFalse(w.blocked(200, 150, 10, 0))

    def test_the_bin_blocks_the_car(self):
        w = world()
        self.assertTrue(w.blocked(60, 150, 10, 0))
        self.assertFalse(w.blocked(100, 150, 10, 0))


if __name__ == "__main__":
    unittest.main()
