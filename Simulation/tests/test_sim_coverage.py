import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.gridmap import CELL, DONE_EVIDENCE, LOOK_FAR_WEIGHT, LOOK_NEAR_CM, GridMap, look_weight  # noqa: E402

ARENA = (-40.0, -150.0, 360.0, 150.0)          # the brain's frame: a 400 x 300 arena with the bin at the origin


class LookWeightTest(unittest.TestCase):
    """A look from close is conclusive; a look from far is not (the camera misses small, far balls), so far floor needs two."""

    def test_a_near_look_counts_fully(self):
        self.assertEqual(look_weight(30.0), 1.0)
        self.assertEqual(look_weight(LOOK_NEAR_CM), 1.0)

    def test_a_far_look_counts_less(self):
        self.assertAlmostEqual(look_weight(250.0), LOOK_FAR_WEIGHT)
        self.assertLess(look_weight(180.0), 1.0)

    def test_it_falls_steadily_with_distance(self):
        values = [look_weight(d) for d in range(0, 260, 10)]
        for a, b in zip(values, values[1:]):
            self.assertGreaterEqual(a, b)

    def test_two_far_looks_are_enough(self):
        self.assertGreaterEqual(2 * look_weight(250.0), DONE_EVIDENCE)


class CountedLooksTest(unittest.TestCase):
    def test_floor_close_to_the_camera_is_done_after_one_look(self):
        m = GridMap()
        m.wedge(0, 0, 0, 200)
        self.assertIn(m.cell(60, 0), m.seen)

    def test_floor_far_from_the_camera_is_not_done_after_one_look(self):
        m = GridMap()
        m.wedge(0, 0, 0, 220)
        self.assertNotIn(m.cell(200, 0), m.seen)
        self.assertGreater(m.evidence[m.cell(200, 0)], 0.0)

    def test_a_second_far_look_finishes_it(self):
        m = GridMap()
        m.wedge(0, 0, 0, 220)
        m.wedge(0, 0, 0, 220)
        self.assertIn(m.cell(200, 0), m.seen)

    def test_a_look_from_nearer_finishes_it_at_once(self):
        m = GridMap()
        m.wedge(0, 0, 0, 220)                                      # far: half done
        m.wedge(120, 0, 0, 220)                                    # now only 80 cm away
        self.assertIn(m.cell(200, 0), m.seen)

    def test_one_picture_counts_a_cell_once_however_many_samples_fall_in_it(self):
        m = GridMap()
        m.wedge(0, 0, 0, 220)
        self.assertLessEqual(max(m.evidence.values()), 1.0 + 1e-9)

    def test_forgetting_the_looks_clears_the_evidence_too(self):
        m = GridMap()
        m.wedge(0, 0, 0, 220)
        m.forget_looks()
        self.assertEqual((m.seen, m.evidence), (set(), {}))


class LookHistoryTest(unittest.TestCase):
    """What had been looked at, at any moment: for the picture."""

    def test_the_floor_done_by_a_time_is_known(self):
        m = GridMap()
        m.wedge(0, 0, 0, 100, now=5.0)
        m.wedge(0, 0, 90, 100, now=20.0)
        early, late = m.seen_at(10.0), m.seen_at(30.0)
        self.assertIn(m.cell(60, 0), early)
        self.assertNotIn(m.cell(0, 60), early)
        self.assertIn(m.cell(0, 60), late)
        self.assertTrue(early < late)

    def test_before_anything_nothing_is_done(self):
        m = GridMap()
        m.wedge(0, 0, 0, 100, now=5.0)
        self.assertEqual(m.seen_at(1.0), set())

    def test_starting_a_new_round_wipes_the_picture_from_then_on(self):
        m = GridMap()
        m.wedge(0, 0, 0, 100, now=5.0)
        m.forget_looks(now=50.0)
        self.assertTrue(m.seen_at(40.0))                           # before the reset it was all still there
        self.assertEqual(m.seen_at(55.0), set())
        m.wedge(0, 0, 180, 100, now=60.0)
        self.assertIn(m.cell(-60, 0), m.seen_at(70.0))
        self.assertNotIn(m.cell(60, 0), m.seen_at(70.0))

    def test_looks_without_a_time_leave_no_history(self):
        m = GridMap()
        m.wedge(0, 0, 0, 100)
        self.assertEqual(m.seen_at(100.0), set())


class ArenaTest(unittest.TestCase):
    def make(self):
        m = GridMap()
        m.set_arena(*ARENA)
        return m

    def test_the_whole_rectangle_is_the_to_do_list(self):
        m = self.make()
        self.assertAlmostEqual(len(m.arena_cells), 400 * 300 / (CELL * CELL), delta=30)          # about 300 cells (the grid does not line up with the walls)
        for cx, cy in m.arena_cells:
            x, y = m.center((cx, cy))
            self.assertTrue(-40 < x < 360 and -150 < y < 150)

    def test_nothing_is_covered_at_first(self):
        self.assertEqual(self.make().coverage(), 0.0)

    def test_looking_raises_the_coverage(self):
        m = self.make()
        m.wedge(100, 0, 0, 100)
        first = m.coverage()
        for h in range(0, 360, 30):
            m.wedge(100, 0, h, 100)
        self.assertGreater(first, 0.0)
        self.assertGreater(m.coverage(), first)

    def test_looking_everywhere_covers_it_all(self):
        m = self.make()
        for cx in range(-20, 360, 40):
            for cy in range(-130, 150, 40):
                for h in range(0, 360, 30):
                    m.wedge(cx, cy, h, 100)
        self.assertGreater(m.coverage(), 0.95)
        self.assertLessEqual(m.coverage(), 1.0)

    def test_floor_under_a_known_obstacle_does_not_count_against_it(self):
        m = self.make()
        for x in range(100, 200, 10):
            for y in range(-50, 50, 10):
                m.block(x, y)                                      # a table: the car can never look at that floor
        for cx in range(-20, 360, 40):
            for cy in range(-130, 150, 40):
                for h in range(0, 360, 30):
                    m.wedge(cx, cy, h, 100)
        self.assertGreater(m.coverage(), 0.95)

    def test_without_an_arena_the_coverage_is_of_the_known_floor(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        m.wedge(0, 0, 0, 200)
        self.assertGreater(m.coverage(), 0.0)


class ArenaSearchTest(unittest.TestCase):
    def make(self):
        m = GridMap()
        m.set_arena(*ARENA)
        return m

    def test_with_the_arena_known_there_is_somewhere_to_go_even_on_an_empty_map(self):
        m = self.make()
        goal = m.best_look_point((40.0, 0.0))
        self.assertIsNotNone(goal)

    def test_the_goal_is_inside_the_arena_away_from_the_walls(self):
        m = self.make()
        goal = m.best_look_point((40.0, 0.0))
        self.assertTrue(-40 + 20 <= goal[0] <= 360 - 20, goal)
        self.assertTrue(-150 + 20 <= goal[1] <= 150 - 20, goal)

    def test_it_heads_for_the_part_not_yet_looked_at(self):
        m = self.make()
        for cx in range(-20, 160, 40):                             # the west half has been looked at
            for cy in range(-130, 150, 40):
                for h in range(0, 360, 30):
                    m.wedge(cx, cy, h, 100)
        goal = m.best_look_point((100.0, 0.0))
        self.assertGreater(goal[0], 140)

    def test_a_scan_just_made_is_not_made_again(self):
        m = self.make()
        m.scanned_at(40.0, 0.0)
        goal = m.best_look_point((40.0, 0.0))
        self.assertGreaterEqual(math.hypot(goal[0] - 40.0, goal[1]), 90.0)

    def test_nothing_left_to_look_at_gives_none(self):
        m = self.make()
        for cx in range(-20, 360, 40):
            for cy in range(-130, 150, 40):
                for h in range(0, 360, 30):
                    m.wedge(cx, cy, h, 100)
        self.assertIsNone(m.best_look_point((40.0, 0.0)))

    def test_the_edge_of_what_the_sensor_reached_does_not_pull_it_out_of_the_arena(self):
        m = self.make()
        m.ray(300.0, 0.0, 0, None)                                 # no echo to 200 cm: a 'frontier' at x = 495, beyond the east wall
        goal = m.best_look_point((300.0, 0.0))
        self.assertLessEqual(goal[0], 360.0)

    def test_it_can_say_which_way_to_turn_the_camera_on_an_empty_map(self):
        m = self.make()
        heading = m.look_direction(300.0, 0.0)
        self.assertIsNotNone(heading)
        self.assertGreater(abs(heading), 90)                       # at the east side the unlooked floor is to the west


class ObstacleHistoryTest(unittest.TestCase):
    """The obstacles the car believes in, at any moment: for the picture (red)."""

    def test_the_obstacles_at_a_time_are_known(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)
        m.record_blocked(now=10.0)
        m.ray(0, 0, 90, 100)
        m.record_blocked(now=20.0)
        self.assertEqual(m.blocked_cells_at(5.0), set())
        self.assertEqual(m.blocked_cells_at(15.0), {m.cell(104, 0)})
        self.assertEqual(m.blocked_cells_at(25.0), {m.cell(104, 0), m.cell(0, 104)})

    def test_an_obstacle_that_was_forgotten_leaves_the_picture(self):
        m = GridMap()
        m.ray(0, 0, 0, 100, now=0.0)
        m.record_blocked(now=1.0)
        m.forget_old(now=500.0)
        m.record_blocked(now=501.0)
        self.assertTrue(m.blocked_cells_at(100.0))
        self.assertEqual(m.blocked_cells_at(502.0), set())

    def test_nothing_is_stored_when_nothing_changed_and_a_change_waits_for_the_interval(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)
        m.record_blocked(now=10.0)
        m.record_blocked(now=11.0)                                 # nothing new
        m.record_blocked(now=40.0)                                 # nothing new
        self.assertEqual(len(m.blocked_snapshots), 1)
        m.ray(0, 0, 90, 100)
        m.record_blocked(now=14.0)                                 # a change, 4 s after the last snapshot: stored
        self.assertEqual(len(m.blocked_snapshots), 2)
        m.ray(0, 0, 180, 100)
        m.record_blocked(now=15.0)                                 # another change one second later: too soon, not stored yet
        self.assertEqual(len(m.blocked_snapshots), 2)
        m.record_blocked(now=20.0)                                 # now it is
        self.assertEqual(len(m.blocked_snapshots), 3)


if __name__ == "__main__":
    unittest.main()
