import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.lanes import lane_route  # noqa: E402

BOUNDS = (-40.0, -150.0, 360.0, 150.0)          # the brain's frame: the bin is the origin; the arena is 400 x 300


def lanes_of(route):
    """The route as [(start, end)] lanes: points come in pairs (lane start, lane end), the car steps over between pairs."""
    return [(route[i], route[i + 1]) for i in range(0, len(route) - 1, 2)]


class RouteTest(unittest.TestCase):
    def test_every_point_is_inside_the_arena_with_a_margin_from_the_walls(self):
        for x, y in lane_route(BOUNDS, margin=25.0):
            self.assertTrue(-40 + 25 - 1e-9 <= x <= 360 - 25 + 1e-9, (x, y))
            self.assertTrue(-150 + 25 - 1e-9 <= y <= 150 - 25 + 1e-9, (x, y))

    def test_lanes_run_along_the_longer_side_by_default(self):
        for (x0, y0), (x1, y1) in lanes_of(lane_route(BOUNDS)):
            self.assertAlmostEqual(y0, y1)                         # each lane is horizontal in a 400 wide, 300 high arena
            self.assertGreater(abs(x1 - x0), 150)

    def test_lanes_can_be_made_to_run_the_other_way(self):
        for (x0, y0), (x1, y1) in lanes_of(lane_route(BOUNDS, along="y")):
            self.assertAlmostEqual(x0, x1)

    def test_the_lanes_are_no_further_apart_than_the_gap_and_reach_both_edges(self):
        ys = sorted({round(a[1], 6) for a, b in lanes_of(lane_route(BOUNDS, gap=70.0))})
        self.assertAlmostEqual(ys[0], -125.0)
        self.assertAlmostEqual(ys[-1], 125.0)
        for a, b in zip(ys, ys[1:]):
            self.assertLessEqual(b - a, 70.0 + 1e-6)

    def test_the_lanes_are_driven_back_and_forth(self):
        lanes = lanes_of(lane_route(BOUNDS))
        directions = [1 if b[0] > a[0] else -1 for a, b in lanes]
        for d1, d2 in zip(directions, directions[1:]):
            self.assertEqual(d1, -d2)

    def test_the_car_steps_over_one_gap_between_lanes(self):
        route = lane_route(BOUNDS, gap=70.0)
        for i in range(1, len(route) - 1, 2):                      # the end of one lane to the start of the next
            (x0, y0), (x1, y1) = route[i], route[i + 1]
            self.assertAlmostEqual(x0, x1)
            self.assertLessEqual(abs(y1 - y0), 70.0 + 1e-6)

    def test_an_offset_moves_the_lanes_over(self):
        plain = {round(a[1], 3) for a, b in lanes_of(lane_route(BOUNDS))}
        moved = {round(a[1], 3) for a, b in lanes_of(lane_route(BOUNDS, offset=35.0))}
        self.assertTrue(plain.isdisjoint(moved))

    def test_it_starts_at_the_lane_end_nearest_the_car(self):
        up = lane_route(BOUNDS, start=(100.0, 120.0))
        down = lane_route(BOUNDS, start=(100.0, -120.0))
        self.assertGreater(up[0][1], 100.0)                        # the car is near the top: start with the top lane
        self.assertLess(down[0][1], -100.0)

    def test_it_starts_at_the_nearer_end_of_that_lane(self):
        left = lane_route(BOUNDS, start=(0.0, -120.0))
        right = lane_route(BOUNDS, start=(340.0, -120.0))
        self.assertLess(left[0][0], right[0][0])

    def test_the_bin_is_not_driven_through(self):
        route = lane_route(BOUNDS, keepout=(0.0, 0.0, 50.0))
        for (x0, y0), (x1, y1) in lanes_of(route):
            n = 40
            for i in range(n + 1):
                x, y = x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n
                self.assertGreaterEqual(math.hypot(x, y), 50.0 - 1.0, (x, y))

    def test_a_lane_that_the_bin_cuts_keeps_its_longer_part(self):
        route = lane_route(BOUNDS, keepout=(0.0, 0.0, 50.0), offset=125.0)          # a lane at y = 0
        middle = [(a, b) for a, b in lanes_of(route) if abs(a[1]) < 1e-6]
        self.assertEqual(len(middle), 1)
        a, b = middle[0]
        self.assertGreater(abs(b[0] - a[0]), 150)                                    # the long part beyond the bin, not the strip behind it


if __name__ == "__main__":
    unittest.main()
