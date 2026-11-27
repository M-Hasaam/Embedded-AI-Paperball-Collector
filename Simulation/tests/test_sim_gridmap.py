import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.gridmap import CELL, GridMap  # noqa: E402


class RayTest(unittest.TestCase):
    def test_a_reading_marks_the_floor_in_front_as_free_and_the_thing_it_hit_as_blocked(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)                                        # something 100 cm ahead along +x
        self.assertIn(m.cell(50, 0), m.free)
        self.assertIn(m.cell(103, 0), m.blocked)
        self.assertNotIn(m.cell(50, 0), m.blocked)

    def test_no_echo_marks_everything_in_range_free_and_blocks_nothing(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        self.assertIn(m.cell(150, 0), m.free)
        self.assertEqual(m.blocked, set())

    def test_a_later_clear_reading_removes_a_stale_obstacle(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)                                        # a box
        m.ray(0, 0, 0, None)                                       # it has been taken away
        self.assertNotIn(m.cell(103, 0), m.blocked)

    def test_a_bump_blocks_the_place(self):
        m = GridMap()
        m.block(120, 40)
        self.assertIn(m.cell(120, 40), m.blocked)


class SeenTest(unittest.TestCase):
    def test_a_picture_marks_the_wedge_ahead_as_looked_at(self):
        m = GridMap()
        m.wedge(0, 0, 0, 200)
        self.assertIn(m.cell(100, 0), m.seen)
        self.assertNotIn(m.cell(-100, 0), m.seen)                  # behind the camera
        self.assertNotIn(m.cell(100, 150), m.seen)                 # far off to the side

    def test_the_wedge_stops_at_the_limit(self):
        m = GridMap()
        m.wedge(0, 0, 0, 80)
        self.assertNotIn(m.cell(150, 0), m.seen)


class PlanTest(unittest.TestCase):
    def test_a_straight_path_when_nothing_is_in_the_way(self):
        m = GridMap()
        path = m.plan((0, 0), (200, 0))
        self.assertIsNotNone(path)
        self.assertLess(math.hypot(path[-1][0] - 200, path[-1][1]), CELL * 1.5)

    def test_it_goes_round_a_wall_of_blocked_cells(self):
        m = GridMap()
        for y in range(-100, 101, 10):
            m.block(100, y)
        path = m.plan((0, 0), (200, 0))
        self.assertIsNotNone(path)
        for x, y in path:
            self.assertFalse(m.in_the_way(x, y), (x, y))
        self.assertGreater(max(abs(y) for _, y in path), 90)       # it detoured round the end of the wall

    def test_the_path_is_the_short_way_round(self):
        m = GridMap()
        for y in range(0, 201, 10):
            m.block(100, y)                                        # a wall from y=0 up to y=200
        path = m.plan((0, 100), (200, 100))
        self.assertIsNotNone(path)
        self.assertLess(min(y for _, y in path), -5)               # it went round the near (south) end, not the far one

    def test_no_path_when_the_goal_is_walled_in(self):
        m = GridMap()
        for a in range(-60, 61, 10):
            for b in (-60, 60):
                m.block(200 + a, b)
                m.block(200 + b, a)
        self.assertIsNone(m.plan((0, 0), (200, 0)))

    def test_the_start_may_touch_an_obstacle(self):
        m = GridMap()
        m.block(25, 0)                                             # right next to where the car is
        path = m.plan((0, 0), (0, 200))
        self.assertIsNotNone(path)

    def test_shortcut_removes_the_staircase(self):
        m = GridMap()
        path = m.plan((0, 0), (200, 100))
        short = m.shortcut(path)
        self.assertLessEqual(len(short), 3)
        self.assertAlmostEqual(short[0][0], path[0][0])
        self.assertAlmostEqual(short[-1][0], path[-1][0])

    def test_shortcut_keeps_the_corner_round_an_obstacle(self):
        m = GridMap()
        for y in range(-100, 101, 10):
            m.block(100, y)
        short = m.shortcut(m.plan((0, 0), (200, 0)))
        self.assertGreaterEqual(len(short), 3)
        for a, b in zip(short, short[1:]):
            self.assertTrue(m.line_free(a, b))


class GoalTest(unittest.TestCase):
    def test_it_picks_a_place_that_overlooks_a_lot_of_unseen_floor(self):
        m = GridMap()
        m.ray(0, 0, 0, None)                                       # a known strip along +x, 4 m long ...
        m.ray(200, 0, 0, None)
        m.wedge(0, 0, 0, 200)                                      # ... of which only the first two metres were looked at
        m.wedge(0, 0, 0, 200)                                      # (twice: far floor needs two looks to count)
        m.scanned_at(0, 0)
        goal = m.best_look_point((0, 0))
        self.assertIsNotNone(goal)
        self.assertGreater(goal[0], 90)                            # somewhere toward the unlooked part, not where we just looked

    def test_nothing_left_to_look_at_gives_none(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)
        for h in range(0, 360, 30):
            m.wedge(0, 0, h, 200)
            m.wedge(0, 0, h, 200)
        self.assertIsNone(m.best_look_point((0, 0)))


class FrontierTest(unittest.TestCase):
    """The edge of what has been measured: where a reading found nothing out to its range, there is more beyond."""

    def test_a_reading_with_no_echo_leaves_a_frontier_at_its_end(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        self.assertIn(m.cell(195, 0), m.ends)

    def test_a_reading_that_hit_something_leaves_none(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)
        self.assertEqual(m.ends, set())

    def test_standing_near_a_frontier_explores_it(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        m.scanned_at(150, 0)
        self.assertEqual(m.ends, set())

    def test_a_frontier_far_from_where_we_stood_stays(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        m.scanned_at(-200, 0)
        self.assertIn(m.cell(195, 0), m.ends)

    def test_everything_looked_at_but_a_frontier_still_gives_a_goal(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        for h in range(0, 360, 30):
            m.wedge(0, 0, h, 200)
            m.wedge(0, 0, h, 200)                                  # (far floor needs two looks to count as looked at)
        goal = m.best_look_point((0, 0))
        self.assertIsNotNone(goal)
        self.assertGreater(goal[0], 80)                            # toward the edge of the known

    def test_a_picture_also_proves_the_floor_it_saw_is_free(self):
        m = GridMap()
        m.wedge(0, 0, 0, 200)
        self.assertIn(m.cell(100, 0), m.free)

    def test_a_picture_does_not_clear_an_obstacle(self):
        m = GridMap()
        m.block(100, 0)
        m.wedge(0, 0, 0, 200)
        self.assertIn(m.cell(100, 0), m.blocked)

    def test_unreachable_goals_are_skipped(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        first = m.best_look_point((0, 0))
        second = m.best_look_point((0, 0), avoid=[first])
        self.assertTrue(second is None or math.hypot(second[0] - first[0], second[1] - first[1]) >= 50)


class LookDirectionTest(unittest.TestCase):
    def test_it_points_toward_the_floor_that_was_not_looked_at(self):
        m = GridMap()
        m.ray(0, 0, 0, None)                                       # known floor along +x ...
        m.ray(0, 0, 180, None)                                     # ... and along -x
        m.wedge(0, 0, 180, 200)                                    # but only -x was looked at
        m.wedge(0, 0, 180, 200)                                    # (twice: far floor needs two looks to count)
        h = m.look_direction(0, 0)
        self.assertIsNotNone(h)
        self.assertLess(abs(h), 40)                                # roughly +x

    def test_nothing_unlooked_gives_none(self):
        m = GridMap()
        self.assertIsNone(m.look_direction(0, 0))

    def test_a_frontier_counts_even_when_everything_known_was_looked_at(self):
        m = GridMap()
        m.ray(0, 0, 90, None)
        m.wedge(0, 0, 90, 200)
        h = m.look_direction(0, 0, radius=250)
        self.assertIsNotNone(h)
        self.assertLess(abs(h - 90), 30)


class NoRescanTest(unittest.TestCase):
    def test_a_place_that_was_just_scanned_is_not_chosen_again(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        m.ray(200, 0, 0, None)
        m.scanned_at(0, 0)
        goal = m.best_look_point((0, 0))
        self.assertIsNotNone(goal)
        self.assertGreaterEqual(math.hypot(goal[0], goal[1]), 90)

    def test_a_close_frontier_can_still_be_chosen(self):
        m = GridMap()
        m.ray(0, 0, 0, None)                                       # the car stands 40 cm from the frontier's cell
        goal = m.best_look_point((120, 0))
        self.assertIsNotNone(goal)

    def test_forgetting_the_scans_allows_going_round_again(self):
        m = GridMap()
        m.ray(0, 0, 0, None)
        m.scanned_at(0, 0)
        m.forget_scans()
        self.assertEqual(m.scan_pts, [])


class RepeatedBumpTest(unittest.TestCase):
    def test_a_second_bump_close_to_the_first_seals_the_whole_neighbourhood(self):
        m = GridMap()
        m.block(100, 100)
        self.assertNotIn(m.cell(120, 100), m.blocked)             # one bump blocks just its own cell
        m.block(110, 105)                                         # bumped again, close by
        for dx in (-20, 0, 20):
            for dy in (-20, 0, 20):
                self.assertIn(m.cell(110 + dx, 105 + dy), m.blocked, (dx, dy))

    def test_a_bump_far_from_the_first_does_not_seal_anything(self):
        m = GridMap()
        m.block(100, 100)
        m.block(300, 100)
        self.assertEqual(len(m.blocked), 2)

    def test_a_sealed_gap_is_not_planned_through(self):
        m = GridMap()
        for y in (-40, 40):                                       # two posts 80 cm apart: a gap the planner would use
            m.block(100, y)
        before = m.plan((0, 0), (200, 0))
        self.assertTrue(any(abs(y) < 20 for x, y in before if 80 < x < 120))
        m.block(100, -35)                                         # the car bumps there again: it learns the gap is not for it
        m.block(100, 35)
        after = m.plan((0, 0), (200, 0))
        self.assertTrue(after is None or all(abs(y) >= 20 or not (80 < x < 120) for x, y in after))


class NeverThroughAnObstacleTest(unittest.TestCase):
    def test_the_path_never_goes_through_a_blocked_cell_even_right_next_to_the_start(self):
        m = GridMap()
        m.block(25, 0)                                             # the thing we just bumped, 25 cm ahead
        blocked_cells = set(m.blocked)
        for goal in ((200, 0), (100, 60), (60, -80)):
            path = m.plan((0, 0), goal)
            self.assertIsNotNone(path, goal)
            for x, y in path[1:-1]:
                self.assertNotIn(m.cell(x, y), blocked_cells, (goal, (x, y)))

    def test_the_straight_line_through_it_is_not_free(self):
        m = GridMap()
        m.block(25, 0)
        self.assertFalse(m.line_free((0, 0), (100, 0), near=(0, 0)))


class ForgettingTest(unittest.TestCase):
    """The car's position guess drifts, so an old obstacle is probably in the wrong place on the map: it must be forgotten."""

    def test_an_obstacle_not_seen_again_is_forgotten_after_a_while(self):
        m = GridMap()
        m.ray(0, 0, 0, 100, now=10.0)
        self.assertIn(m.cell(103, 0), m.blocked)
        m.forget_old(now=10.0 + 60.0, max_age=90.0)
        self.assertIn(m.cell(103, 0), m.blocked)                   # still fresh enough
        m.forget_old(now=10.0 + 120.0, max_age=90.0)
        self.assertNotIn(m.cell(103, 0), m.blocked)

    def test_seeing_it_again_keeps_it(self):
        m = GridMap()
        m.ray(0, 0, 0, 100, now=10.0)
        m.ray(0, 0, 0, 100, now=100.0)                             # seen again
        m.forget_old(now=150.0, max_age=90.0)
        self.assertIn(m.cell(103, 0), m.blocked)

    def test_a_bump_is_forgotten_too_but_later(self):
        m = GridMap()
        m.block(100, 100, now=10.0)
        m.forget_old(now=10.0 + 100.0, max_age=90.0)
        self.assertIn(m.cell(100, 100), m.blocked)                 # bumps are kept longer than readings
        m.forget_old(now=10.0 + 250.0, max_age=90.0)
        self.assertNotIn(m.cell(100, 100), m.blocked)
        self.assertNotIn(m.cell(100, 100), m.hard)

    def test_forgetting_opens_the_way_again(self):
        m = GridMap()
        for y in range(-100, 101, 10):
            m.block(100, y, now=0.0)
        self.assertGreater(max(abs(y) for _, y in m.plan((0, 0), (200, 0))), 90)       # round the wall
        m.forget_old(now=1000.0, max_age=90.0)
        path = m.plan((0, 0), (200, 0))
        self.assertLess(max(abs(y) for _, y in path), 40)                              # straight through where the wall was

    def test_without_a_time_nothing_is_forgotten(self):
        m = GridMap()
        m.ray(0, 0, 0, 100)
        m.forget_old(now=1e6, max_age=90.0)
        self.assertIn(m.cell(103, 0), m.blocked)


class ResetTest(unittest.TestCase):
    def test_forgetting_all_obstacles_frees_a_car_that_the_map_has_boxed_in(self):
        m = GridMap()
        for a in range(-60, 61, 10):
            for b in (-60, 60):
                m.block(b, a)
                m.block(a, b)
        self.assertIsNone(m.plan((0, 0), (200, 0)))
        m.reset_obstacles()
        self.assertIsNotNone(m.plan((0, 0), (200, 0)))
        self.assertEqual(m.blocked, set())
        self.assertEqual(m.hard, set())

    def test_what_has_been_looked_at_is_kept(self):
        m = GridMap()
        m.wedge(0, 0, 0, 200)
        m.reset_obstacles()
        self.assertTrue(m.seen)


class DestinationTest(unittest.TestCase):
    def test_the_destination_can_be_cleared_from_the_obstacles(self):
        m = GridMap()
        m.ray(100, 0, 180, 80)                                     # the ultrasonic hit the bin's near side, 80 cm ahead -> x = 20
        m.block(30, 10)
        m.block(200, 0)                                            # a real obstacle far away
        m.clear_blocked_near(0, 0, 45)
        self.assertNotIn(m.cell(20, 0), m.blocked)
        self.assertNotIn(m.cell(30, 10), m.blocked)
        self.assertIn(m.cell(200, 0), m.blocked)

    def test_a_path_can_then_end_right_beside_it(self):
        m = GridMap()
        m.ray(100, 0, 180, 85)
        m.clear_blocked_near(0, 0, 45)
        self.assertTrue(m.line_free((100, 0), (45, 0), near=(100, 0)))


class StraightPathTest(unittest.TestCase):
    def test_it_cuts_straight_across_unexplored_floor_instead_of_following_a_long_known_corridor(self):
        # known floor makes a long U-shaped detour (up, across, down: 400 cm) while the straight way (200 cm) has never been seen;
        # obstacles in unexplored floor are found by the distance sensor on the way, so a straight line is the better bet
        m = GridMap()
        for y in range(0, 101, 10):
            m.free.add(m.cell(0, y))
            m.free.add(m.cell(200, y))
        for x in range(0, 201, 10):
            m.free.add(m.cell(x, 100))
        path = m.plan((0, 0), (200, 0))
        self.assertIsNotNone(path)
        self.assertLess(max(abs(y) for _, y in path), 40, "it took the long way round through known floor")

    def test_unexplored_floor_still_costs_a_little_more_than_known_floor(self):
        from sim.gridmap import UNKNOWN_COST
        self.assertGreater(UNKNOWN_COST, 1.0)
        self.assertLess(UNKNOWN_COST, 2.5)


if __name__ == "__main__":
    unittest.main()
