import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.car import Noise  # noqa: E402
from sim.run import run_episode  # noqa: E402
from sim.timeline import clock, events_of, plain_batch, plain_summary, state_at  # noqa: E402


class StateAtTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 300, Noise.perfect())

    def test_at_the_start_the_car_is_at_the_start_with_nothing_collected(self):
        s = state_at(self.res, 0.0)
        self.assertAlmostEqual(s.x, 80.0)
        self.assertAlmostEqual(s.y, 150.0)
        self.assertFalse(s.carrying)
        self.assertEqual(s.delivered, 0)
        self.assertEqual(len(s.floor_balls), self.res.balls)

    def test_at_the_end_the_counts_match_the_result(self):
        s = state_at(self.res, self.res.seconds)
        self.assertEqual(s.delivered, self.res.delivered)
        self.assertEqual(len(s.floor_balls), self.res.balls - self.res.delivered)

    def test_the_car_is_carrying_between_a_pick_and_its_delivery(self):
        picks = [e for e in self.res.car.events if e[0] == "pick"]
        delivers = [e for e in self.res.car.events if e[0] == "deliver"]
        self.assertTrue(picks and delivers)
        mid = (picks[0][1] + delivers[0][1]) / 2
        self.assertTrue(state_at(self.res, mid).carrying)
        self.assertFalse(state_at(self.res, picks[0][1] - 1).carrying)
        self.assertFalse(state_at(self.res, delivers[0][1] + 0.01).carrying)

    def test_a_picked_ball_leaves_the_floor_when_the_jaws_close(self):
        g = [g for g in self.res.car.grips if g.kind == "pick" and g.ok][0]
        early = state_at(self.res, g.start + 0.2 * (g.end - g.start))
        late = state_at(self.res, g.start + 0.8 * (g.end - g.start))
        self.assertEqual(len(early.floor_balls) - len(late.floor_balls), 1)

    def test_the_position_is_interpolated_between_two_recorded_points(self):
        (x0, y0, t0, _), (x1, y1, t1, _) = self.res.car.path[10], self.res.car.path[11]
        s = state_at(self.res, (t0 + t1) / 2)
        self.assertAlmostEqual(s.x, (x0 + x1) / 2, places=6)
        self.assertAlmostEqual(s.y, (y0 + y1) / 2, places=6)

    def test_a_time_outside_the_run_is_clamped(self):
        self.assertAlmostEqual(state_at(self.res, -5).x, state_at(self.res, 0).x)
        self.assertAlmostEqual(state_at(self.res, 1e9).x, self.res.car.path[-1][0])


class ArmStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 300, Noise.perfect())
        cls.pick = [g for g in cls.res.car.grips if g.kind == "pick" and g.ok][0]
        cls.drop = [g for g in cls.res.car.grips if g.kind == "drop" and g.ok][0]

    def test_there_is_no_arm_move_while_driving(self):
        self.assertIsNone(state_at(self.res, 1.0).arm)

    def test_during_a_pick_it_says_so_with_the_progress(self):
        mid = (self.pick.start + self.pick.end) / 2
        arm = state_at(self.res, mid).arm
        self.assertEqual(arm.kind, "pick")
        self.assertAlmostEqual(arm.progress, 0.5, places=3)
        self.assertTrue(arm.ok)
        self.assertEqual(arm.target, self.pick.target)

    def test_during_a_release_it_says_so(self):
        arm = state_at(self.res, self.drop.start + 0.3 * (self.drop.end - self.drop.start)).arm
        self.assertEqual(arm.kind, "drop")
        self.assertAlmostEqual(arm.progress, 0.3, places=3)

    def test_the_move_is_over_when_it_ends(self):
        self.assertIsNone(state_at(self.res, self.pick.end + 0.01).arm)

    def test_the_bin_counter_goes_up_when_the_jaws_open_not_at_the_end(self):
        before = state_at(self.res, self.drop.start + 0.3 * (self.drop.end - self.drop.start)).delivered
        after = state_at(self.res, self.drop.start + 0.7 * (self.drop.end - self.drop.start)).delivered
        self.assertEqual(after, before + 1)

    def test_the_events_are_timed_from_the_start_of_the_arm_move(self):
        events = dict((line, t) for t, line in events_of(self.res))
        self.assertAlmostEqual(events["Picking up ball 1"], self.pick.start)
        self.assertAlmostEqual(events["Dropping ball 1 in the bin"], self.drop.start)


class RouteStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lanes = run_episode("empty", 0, 200, Noise.perfect(), 3, strategy="lanes")
        cls.mapped = run_episode("empty", 0, 60, Noise.perfect(), 3)

    def test_the_lane_search_shows_its_route(self):
        route = state_at(self.lanes, 30.0).route
        self.assertTrue(route and len(route) >= 4)

    def test_the_route_is_in_the_arenas_own_coordinates(self):
        for x, y in state_at(self.lanes, 30.0).route:
            self.assertTrue(0 <= x <= 400 and 0 <= y <= 300, (x, y))

    def test_the_map_search_has_no_route_to_show(self):
        self.assertIsNone(state_at(self.mapped, 30.0).route)
        self.assertIsNone(state_at(self.mapped, 0.0).route)

    def test_the_lane_search_lays_out_its_first_pass_at_the_start(self):
        self.assertTrue(state_at(self.lanes, 0.0).route)

    def test_the_route_is_the_pass_that_was_laid_out_at_that_time(self):
        late = state_at(self.lanes, self.lanes.seconds).route
        self.assertTrue(late)


class CoverageStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("empty", 0, 200, Noise.perfect(), 3)

    def test_at_the_start_nothing_has_been_looked_at(self):
        self.assertEqual(state_at(self.res, 0.0).seen, [])
        self.assertEqual(state_at(self.res, 0.0).coverage, 0.0)

    def test_later_a_good_part_has_been_looked_at(self):
        late = state_at(self.res, 150.0)
        self.assertGreater(len(late.seen), 20)                          # (the car spends much of the 200 s collecting its three balls)
        self.assertGreater(late.coverage, 0.08)

    def test_the_looked_at_floor_only_grows_while_a_round_lasts(self):
        a, b = state_at(self.res, 20.0), state_at(self.res, 60.0)
        self.assertTrue(set(a.seen) <= set(b.seen))
        self.assertLessEqual(a.coverage, b.coverage)

    def test_the_cells_are_in_the_arenas_own_coordinates(self):
        for x, y in state_at(self.res, 150.0).seen:
            self.assertTrue(-20 <= x <= 400 and -20 <= y <= 300, (x, y))          # lower-left corners of 20 cm cells

    def test_it_is_a_fraction(self):
        for t in (0.0, 30.0, 100.0, 199.0):
            self.assertTrue(0.0 <= state_at(self.res, t).coverage <= 1.0)


def custom_result(balls, obstacles=(), max_time=240):
    from sim.brain import Brain
    from sim.car import CarAPI, SimCar
    from sim.run import Result
    from sim.world import World
    world = World(400, 300, (40.0, 150.0), obstacles=list(obstacles), balls=list(balls))
    car = SimCar(world, (80.0, 150.0), 0.0, Noise.perfect(), 1)
    brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1, bounds=(-40.0, -150.0, 360.0, 150.0))
    brain.run(max_time)
    return Result("custom", 1, len(balls), car.delivered, car.t, car.bumps, car.driven_cm, float("nan"), 0.0, "map", car, brain, world)


class MapBeliefStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from sim.world import Ball, Obstacle
        cls.res = custom_result([Ball(150, 150), Ball(150, 100)], [Obstacle(250, 200, 290, 260)])

    def test_a_ball_it_has_seen_but_not_collected_is_waiting(self):
        first_pick = [e for e in self.res.car.events if e[0] == "pick"][0][1]
        waiting = state_at(self.res, first_pick + 1.0).pending
        self.assertEqual(len(waiting), 1)
        x, y = waiting[0]
        self.assertAlmostEqual(x, 150, delta=25)                       # in the arena's coordinates
        self.assertAlmostEqual(y, 100, delta=25)

    def test_when_it_has_been_collected_nothing_is_waiting(self):
        self.assertEqual(state_at(self.res, self.res.seconds).pending, [])

    def test_nothing_is_waiting_at_the_very_start(self):
        self.assertEqual(state_at(self.res, 0.0).pending, [])

    def test_the_obstacles_it_has_found_are_listed_in_the_arenas_coordinates(self):
        cells = state_at(self.res, self.res.seconds).obstacles
        self.assertTrue(cells)
        for x, y in cells:
            self.assertTrue(-20 < x < 420 and -20 < y < 320, (x, y))

    def test_the_walls_it_has_found_are_obstacles_too(self):
        cells = state_at(self.res, self.res.seconds).obstacles
        near_wall = [c for c in cells if c[0] < 25 or c[0] > 375 or c[1] < 25 or c[1] > 275]
        self.assertTrue(near_wall)

    def test_it_knows_no_more_about_the_obstacles_at_the_start_than_at_the_end(self):
        self.assertLessEqual(len(state_at(self.res, 3.0).obstacles), len(state_at(self.res, self.res.seconds).obstacles))


class TemporaryObstacleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("clutter", 8, 400)

    def test_the_bag_is_there_only_between_its_times(self):
        def labels(t):
            return [o.label for o in state_at(self.res, t).temp_active]
        self.assertEqual(labels(10), [])
        self.assertEqual(labels(100), ["Bag"])
        self.assertEqual(labels(200), ["Bag", "Box"])
        self.assertEqual(labels(300), ["Box"])

    def test_the_bumps_so_far_grow_to_the_total(self):
        self.assertEqual(state_at(self.res, 0).bumps, 0)
        self.assertEqual(state_at(self.res, self.res.seconds).bumps, self.res.bumps)

    def test_the_cars_own_guess_is_available_over_time(self):
        s = state_at(self.res, 50.0)
        self.assertIsNotNone(s.guess)
        self.assertEqual(len(s.guess), 2)


class EventsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = run_episode("clutter", 8, 400)
        cls.events = events_of(cls.res)

    def test_they_are_in_time_order(self):
        times = [t for t, _ in self.events]
        self.assertEqual(times, sorted(times))

    def test_pickups_and_deliveries_are_numbered(self):
        text = "\n".join(line for _, line in self.events)
        self.assertIn("Picking up ball 1", text)
        self.assertIn("Dropping ball 1 in the bin", text)

    def test_obstacles_arriving_and_leaving_are_listed(self):
        text = "\n".join(line for _, line in self.events)
        self.assertIn("Bag appears", text)
        self.assertIn("Bag is taken away", text)

    def test_the_clock_is_minutes_and_seconds(self):
        self.assertEqual(clock(0), "0:00")
        self.assertEqual(clock(65.4), "1:05")
        self.assertEqual(clock(600), "10:00")


class SummaryTest(unittest.TestCase):
    def test_it_says_it_in_plain_words(self):
        r = run_episode("empty", 0, 300, Noise.perfect())
        lines = plain_summary(r)
        text = " ".join(lines)
        self.assertIn(f"{r.delivered} of {r.balls}", text)
        self.assertIn("bump", text.lower())
        self.assertNotIn("x1.", text)                       # no 'return path x1.3 of straight' jargon

    def test_it_says_how_the_time_was_spent(self):
        r = run_episode("empty", 0, 300, Noise.perfect())
        text = " ".join(plain_summary(r))
        self.assertIn("driving", text)
        self.assertIn("%", text)


    def test_it_says_which_search_the_car_used(self):
        a = " ".join(plain_summary(run_episode("empty", 0, 120, Noise.perfect(), 3)))
        b = " ".join(plain_summary(run_episode("empty", 0, 120, Noise.perfect(), 3, strategy="lanes")))
        self.assertIn("map", a.lower())
        self.assertIn("lawnmower", b.lower())

    def test_a_batch_is_summed_up_in_plain_words(self):
        results = [run_episode("empty", i, 300, Noise.perfect()) for i in range(2)]
        text = " ".join(plain_batch(results))
        got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
        self.assertIn("2 layouts", text)
        self.assertIn(f"{got} of {total}", text)


if __name__ == "__main__":
    unittest.main()
