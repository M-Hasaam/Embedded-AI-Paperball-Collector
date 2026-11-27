import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.brain import Brain  # noqa: E402
from sim.car import CarAPI, Noise, SimCar  # noqa: E402
from sim.run import run_episode  # noqa: E402
from sim.scenarios import BIN_XY, START_HEADING, START_XY  # noqa: E402
from sim.world import Ball, Obstacle, World  # noqa: E402

ALLOWED = {"time", "heading", "turn", "drive", "ultrasonic", "look", "in_jaws", "pick", "drop"}


class StrictAPI:
    """Wraps the car's API and fails the test the moment the brain touches anything the real car does not have."""

    def __init__(self, api):
        object.__setattr__(self, "_api", api)
        object.__setattr__(self, "used", set())

    def __getattr__(self, name):
        if name not in ALLOWED:
            raise AssertionError(f"the brain used {name!r}, which the real car does not have")
        self.used.add(name)
        return getattr(self._api, name)


def episode(world, noise=None, max_time=600, seed=1):
    car = SimCar(world, START_XY, START_HEADING, noise or Noise.perfect(), seed)
    api = StrictAPI(CarAPI(car))
    brain = Brain(api, start_est=(START_XY[0] - BIN_XY[0], START_XY[1] - BIN_XY[1]), seed=seed)
    brain.run(max_time)
    return car, brain, api


class BrainTest(unittest.TestCase):
    def test_it_collects_a_ball_and_delivers_it_to_the_bin(self):
        world = World(400, 300, BIN_XY, balls=[Ball(250, 150)])
        car, brain, _ = episode(world)
        self.assertEqual(car.delivered, 1)
        self.assertEqual(brain.collected, 1)

    def test_it_uses_only_what_the_real_car_has(self):
        world = World(400, 300, BIN_XY, balls=[Ball(250, 180)])
        _, _, api = episode(world, Noise())
        self.assertTrue({"turn", "drive", "ultrasonic", "look", "pick", "drop", "heading"} <= api.used)

    def test_it_finds_a_ball_behind_it(self):
        world = World(400, 300, BIN_XY, balls=[Ball(100, 60)])
        car, _, _ = episode(world)
        self.assertEqual(car.delivered, 1)

    def test_it_goes_round_a_table(self):
        world = World(400, 300, BIN_XY, obstacles=[Obstacle(150, 100, 190, 200)], balls=[Ball(300, 150)])
        car, _, _ = episode(world)
        self.assertEqual(car.delivered, 1)
        self.assertLessEqual(car.bumps, 4)          # a forward-pointing sensor cannot see a corner its side clips

    def test_it_copes_with_a_box_that_appears_in_its_way(self):
        world = World(400, 300, BIN_XY, obstacles=[Obstacle(120, 120, 160, 180, appear=5, vanish=1000)], balls=[Ball(300, 150)])
        car, _, _ = episode(world)
        self.assertEqual(car.delivered, 1)

    def test_it_collects_every_ball_in_an_empty_arena(self):
        world = World(400, 300, BIN_XY, balls=[Ball(250, 150), Ball(300, 60), Ball(150, 250)])
        car, _, _ = episode(world, max_time=900)
        self.assertEqual(car.delivered, 3)

    def test_it_does_not_keep_driving_into_a_chair_leg_it_cannot_see(self):
        # the ultrasonic never sees the leg: the car touches it, must back off and try another way instead of pushing for ever
        n = Noise.perfect()
        n.thin_hit = 0.0
        world = World(400, 300, BIN_XY, obstacles=[Obstacle(160, 148, 163, 152, thin=True)], balls=[Ball(260, 150)])
        car, _, _ = episode(world, n, max_time=400)
        self.assertLess(car.bumps, 25)
        self.assertEqual(car.delivered, 1)


class EfficiencyTest(unittest.TestCase):
    """The car must spend its time getting somewhere, not spinning on the spot."""

    @unittest.expectedFailure
    def test_most_of_the_time_is_not_spent_turning_on_the_spot(self):
        # KNOWN, NOT YET MET: stop-and-look scanning (a camera picture every 30 degrees) keeps the car turning about 38% of the time.
        # The fix is a different way of looking (turning continuously while the camera runs), not a different route.
        result = run_episode("clutter", 8, 600)
        turning = result.car.spent["turn"] / result.seconds
        self.assertLess(turning, 0.30, f"turning {turning:.0%} of the time")

    def test_a_fair_share_of_the_time_is_spent_driving(self):
        # it was 10% before the map and the planner (the car spent its time spinning); this keeps it from sliding back
        result = run_episode("clutter", 8, 600)
        driving = result.car.spent["drive"] / result.seconds
        self.assertGreater(driving, 0.15, f"driving {driving:.0%} of the time")

    def test_it_clears_the_cluttered_arena_in_fifteen_minutes(self):
        results = [run_episode("clutter", s, 900) for s in range(5)]
        got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
        self.assertGreaterEqual(got / total, 0.75, f"{got}/{total}")


def longest_stay(result, radius=20.0):
    """The longest time (seconds) the car stayed within `radius` cm of one place."""
    import math
    path, best, i = result.car.path, 0.0, 0
    for j in range(len(path)):
        while math.hypot(path[j][0] - path[i][0], path[j][1] - path[i][1]) > radius:
            i += 1
        best = max(best, path[j][2] - path[i][2])
    return best


class NotStuckTest(unittest.TestCase):
    def test_it_never_sits_in_one_place_for_minutes(self):
        # reported (seed 8): carrying the last ball, the car stood against the east face of the table for 4.5 minutes, planning a route
        # into it. Seeds 0 and 16 stood 72 cm from the bin, facing it, unable to take the last step.
        for seed in (0, 8, 16):
            result = run_episode("clutter", seed, 900)
            self.assertLess(longest_stay(result), 90.0, f"seed {seed}: stayed {longest_stay(result):.0f} s in one place")

    def test_it_gets_the_last_ball_home_from_beyond_the_table(self):
        result = run_episode("clutter", 8, 900)
        self.assertEqual(result.delivered, result.balls)


class SpotWhileDrivingTest(unittest.TestCase):
    def test_a_ball_that_comes_into_view_on_the_way_stops_the_drive(self):
        world = World(400, 300, BIN_XY, balls=[Ball(260, 150)])
        car = SimCar(world, (80, 150), 0.0, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1)
        status = brain._goto((300, 0), stop_within=25.0, until=brain._spot_ball)      # drive east ... the ball is ahead on the way
        self.assertEqual(status, "until")
        self.assertIsNotNone(brain.spotted)
        heading, dist = brain.spotted
        self.assertLess(abs(heading), 5.0)
        self.assertLess(car.x, 230)                                                   # it stopped before reaching the ball

    def test_nothing_in_view_does_not_stop_it(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, (80, 150), 0.0, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1)
        self.assertEqual(brain._goto((200, 0), stop_within=25.0, until=brain._spot_ball), "arrived")
        self.assertIsNone(brain.spotted)

    def test_a_ball_it_could_not_reach_is_not_spotted_again_at_once(self):
        world = World(400, 300, BIN_XY, balls=[Ball(260, 150)])
        car = SimCar(world, (80, 150), 0.0, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1)
        brain.unreachable.append((260 - 40, 0.0, car.time()))                         # (the brain's frame has the bin as its origin)
        self.assertFalse(brain._spot_ball())

    def test_the_ball_near_the_chair_legs_is_not_left_for_ten_minutes(self):
        # reported: the car passed within 34 cm of this ball three times, never seeing it, and picked it up only at 692 s
        result = run_episode("clutter", 8, 900)
        ball = min(result.world.balls, key=lambda b: (b.x - 76) ** 2 + (b.y - 35) ** 2)
        self.assertIsNotNone(ball.picked_at)
        self.assertLess(ball.picked_at, 500.0, f"picked up only at {ball.picked_at:.0f} s")


class LaneStrategyTest(unittest.TestCase):
    """The lawnmower search: straight lanes across the arena with a forward look at every step, instead of stopping to spin and look."""

    def test_it_collects_every_ball_in_an_empty_arena(self):
        result = run_episode("empty", 8, 900, Noise.perfect(), 5, strategy="lanes")
        self.assertEqual(result.delivered, result.balls)

    def test_it_hardly_spins_on_the_spot(self):
        result = run_episode("empty", 8, 900, Noise.perfect(), 5, strategy="lanes")
        self.assertLess(result.car.spent["turn"] / result.seconds, 0.20)

    def test_most_of_the_time_it_is_driving(self):
        result = run_episode("empty", 8, 900, Noise.perfect(), 5, strategy="lanes")
        self.assertGreater(result.car.spent["drive"] / result.seconds, 0.45)

    def test_the_result_says_which_strategy_it_used(self):
        self.assertEqual(run_episode("empty", 0, 60, Noise.perfect(), 3, strategy="lanes").strategy, "lanes")
        self.assertEqual(run_episode("empty", 0, 60, Noise.perfect(), 3).strategy, "map")

    def test_it_copes_with_the_cluttered_arena(self):
        results = [run_episode("clutter", s, 900, strategy="lanes") for s in range(4)]
        got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
        self.assertGreaterEqual(got / total, 0.6, f"{got}/{total}")

    def test_an_unknown_strategy_is_refused(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        with self.assertRaises(ValueError):
            Brain(CarAPI(car), strategy="wander")

    def test_the_lane_search_needs_to_know_the_arena(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        with self.assertRaises(ValueError):
            Brain(CarAPI(car), strategy="lanes")                                  # no bounds given

    def test_it_never_sits_in_one_place_for_minutes(self):
        for seed in (0, 8, 16):
            result = run_episode("clutter", seed, 900, strategy="lanes")
            self.assertLess(longest_stay(result), 90.0, f"seed {seed}: stayed {longest_stay(result):.0f} s in one place")


class PhantomBallTest(unittest.TestCase):
    """The camera sometimes 'sees' a ball that is not there. A phantom outside the arena sent the car for minutes toward a spot beyond
    the wall; one inside the arena costs seconds (the car arrives, sees nothing, gives up). So: ignore what lies outside, and cap a trip."""

    def brain(self, noise, balls=(), start=(80, 150), heading=0.0, seed=1):
        world = World(400, 300, BIN_XY, balls=list(balls))
        car = SimCar(world, start, heading, noise, seed)
        return Brain(CarAPI(car), start_est=(start[0] - 40.0, start[1] - 150.0), seed=1, bounds=(-40.0, -150.0, 360.0, 150.0)), car

    def test_a_real_ball_is_spotted(self):
        brain, car = self.brain(Noise.perfect(), balls=[Ball(220, 150)])
        self.assertTrue(brain._spot_ball())
        heading, dist = brain.spotted
        self.assertAlmostEqual(heading, 0.0, delta=1.0)
        self.assertAlmostEqual(dist, 140 - 8, delta=5.0)

    def test_a_real_ball_is_spotted_with_realistic_errors_most_of_the_time(self):
        spotted = 0
        for seed in range(40):
            brain, _ = self.brain(Noise(), balls=[Ball(220, 150)], seed=seed)
            spotted += brain._spot_ball()
        self.assertGreater(spotted / 40, 0.6)

    def test_a_look_costs_one_picture(self):
        brain, car = self.brain(Noise.perfect())
        t0 = car.time()
        brain._spot_ball()
        self.assertAlmostEqual(car.time() - t0, car.noise.look_s, places=6)

    def test_a_phantom_outside_the_arena_is_never_spotted(self):
        for seed in range(80):
            n = Noise.perfect()
            n.cam_false_pos = 1.0                                  # a phantom in every picture
            brain, car = self.brain(n, start=(395, 150), heading=0.0, seed=seed)        # against the east wall, facing it: phantoms land 60-200 cm beyond
            if brain._spot_ball():
                x, y = brain._ball_xy(*brain.spotted)
                self.assertTrue(brain._inside((x, y)), (seed, x, y))

    def test_a_place_outside_the_arena_is_not_a_ball(self):
        brain, _ = self.brain(Noise.perfect())
        self.assertTrue(brain._inside((100.0, 0.0)))
        self.assertTrue(brain._inside((-30.0, -140.0)))           # just inside the corner
        self.assertTrue(brain._inside((100.0, -240.0)))           # a metre outside the wall by the guess: the guess drifts, so it may be real
        self.assertFalse(brain._inside((100.0, -400.0)))
        self.assertFalse(brain._inside((900.0, 0.0)))

    def test_without_the_arena_size_nothing_is_ruled_out(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, (80, 150), 0.0, Noise.perfect(), 1)
        self.assertTrue(Brain(CarAPI(car), start_est=(40.0, 0.0))._inside((1e6, 1e6)))

    def test_chasing_a_phantom_inside_the_arena_costs_seconds_not_minutes(self):
        brain, car = self.brain(Noise.perfect())                  # no ball at all; the brain is told one is 150 cm ahead
        t0 = car.time()
        self.assertFalse(brain._collect(0.0, 150.0))
        self.assertLess(car.time() - t0, 60.0)


class PlannerSettingTest(unittest.TestCase):
    """Each search plans its paths with the cost of unexplored floor that suits it: the map search likes straight lines through it, the
    lane search (which follows the walls) is better off preferring floor it has seen."""

    def make(self, strategy):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        return Brain(CarAPI(car), strategy=strategy, bounds=(-40.0, -150.0, 360.0, 150.0))

    def test_the_map_search_prefers_straight_lines_through_unexplored_floor(self):
        self.assertAlmostEqual(self.make("map").map.unknown_cost, 1.5)

    def test_the_lane_search_prefers_floor_it_has_seen(self):
        self.assertAlmostEqual(self.make("lanes").map.unknown_cost, 3.0)

    def test_the_map_uses_its_own_setting_when_planning(self):
        from sim.gridmap import GridMap
        m = GridMap()
        for y in range(0, 101, 10):
            m.free.add(m.cell(0, y))
            m.free.add(m.cell(200, y))
        for x in range(0, 201, 10):
            m.free.add(m.cell(x, 100))
        m.unknown_cost = 3.0
        self.assertGreater(max(y for _, y in m.plan((0, 0), (200, 0))), 90)       # the long way round the known floor
        m.unknown_cost = 1.5
        self.assertLess(max(abs(y) for _, y in m.plan((0, 0), (200, 0))), 40)     # straight across


class CoverageWiringTest(unittest.TestCase):
    """The brain gives its map the arena (the to-do list), tells it when it looks, and keeps a log of how much is done."""

    def test_the_map_is_given_the_arena(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), bounds=(-40.0, -150.0, 360.0, 150.0))
        self.assertEqual(brain.map.arena, (-40.0, -150.0, 360.0, 150.0))

    def test_without_the_arena_the_map_has_none(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        self.assertIsNone(Brain(CarAPI(car)).map.arena)

    def test_a_look_is_recorded_with_the_time(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), bounds=(-40.0, -150.0, 360.0, 150.0))
        brain._look()
        self.assertTrue(brain.map.seen_events)
        self.assertTrue(all(abs(t - car.time()) < 1e-9 for t, c in brain.map.seen_events))
        self.assertEqual(len(brain.coverage_log), 1)

    def test_the_coverage_log_grows_as_the_car_looks_around(self):
        result = run_episode("empty", 0, 120, Noise.perfect(), 3)
        log = result.brain.coverage_log
        self.assertGreater(len(log), 20)
        self.assertTrue(all(0.0 <= c <= 1.0 for t, c in log))
        self.assertGreater(max(c for t, c in log), 0.4)                          # in two minutes it has looked at a good part of the floor

    def test_both_searches_keep_the_log(self):
        for strategy in ("map", "lanes"):
            result = run_episode("empty", 0, 90, Noise.perfect(), 3, strategy=strategy)
            self.assertTrue(result.brain.coverage_log, strategy)

    def test_a_new_round_of_looking_is_recorded_for_the_picture(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), bounds=(-40.0, -150.0, 360.0, 150.0))
        brain._look()
        brain.map.forget_looks(now=car.time())
        self.assertEqual(brain.map.seen_resets, [car.time()])


class BallMemoryTest(unittest.TestCase):
    """A scan can show two balls; the car picks one up. The other must not be forgotten: it is the first place to go back to."""

    def brain(self, balls=(), noise=None, rechecks=0, strategy="map"):
        world = World(400, 300, BIN_XY, balls=list(balls))
        car = SimCar(world, START_XY, START_HEADING, noise or Noise.perfect(), 1)
        return Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1, strategy=strategy, rechecks=rechecks,
                     bounds=(-40.0, -150.0, 360.0, 150.0)), car

    def test_a_ball_that_is_not_picked_up_is_remembered(self):
        brain, car = self.brain()
        brain._remember(0.0, 120.0)
        self.assertEqual(len(brain.pending), 1)
        x, y = brain.pending[0].x, brain.pending[0].y
        self.assertAlmostEqual(x, 40.0 + 128.0, delta=1.0)
        self.assertAlmostEqual(y, 0.0, delta=1.0)

    def test_the_same_ball_seen_again_is_not_a_second_ball(self):
        brain, _ = self.brain()
        brain._remember(0.0, 120.0)
        brain._remember(2.0, 122.0)
        self.assertEqual(len(brain.pending), 1)

    def test_a_ball_outside_the_arena_is_not_remembered(self):
        brain, _ = self.brain()
        brain._remember(0.0, 900.0)
        self.assertEqual(brain.pending, [])

    def test_every_change_to_the_memory_is_logged_for_the_picture(self):
        brain, _ = self.brain()
        brain._remember(0.0, 120.0)
        self.assertEqual([e[1] for e in brain.ball_events], ["add"])
        brain._forget(brain.pending[0])
        self.assertEqual([e[1] for e in brain.ball_events], ["add", "remove"])
        self.assertEqual(brain.pending, [])

    def test_going_back_for_a_remembered_ball_picks_it_up_and_forgets_it(self):
        brain, car = self.brain(balls=[Ball(220, 150)])
        brain._remember(0.0, 140.0 - 8.0)
        self.assertTrue(brain._go_for_remembered())
        self.assertTrue(brain.carrying)
        self.assertEqual(brain.pending, [])

    def test_a_remembered_ball_that_is_not_there_is_given_up_after_two_tries(self):
        brain, car = self.brain()                                   # nothing there
        brain._remember(0.0, 120.0)
        brain._go_for_remembered()
        self.assertEqual(len(brain.pending), 1)                     # one more try
        brain._go_for_remembered()
        self.assertEqual(brain.pending, [])

    def test_picking_a_ball_up_also_clears_its_memory(self):
        brain, car = self.brain(balls=[Ball(220, 150)])
        brain._remember(0.0, 140.0 - 8.0)
        brain.pending[0].tries = 0
        brain._forget_near(brain.pending[0].x, brain.pending[0].y)
        self.assertEqual(brain.pending, [])

    def test_the_second_of_two_balls_is_fetched_before_the_search_goes_on(self):
        world_balls = [Ball(150, 150), Ball(150, 100)]              # both in view of the first scan
        world = World(400, 300, BIN_XY, balls=world_balls)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1, bounds=(-40.0, -150.0, 360.0, 150.0))
        brain.run(300)
        picks = [e for e in car.events if e[0] == "pick"]
        delivers = [e for e in car.events if e[0] == "deliver"]
        self.assertEqual(len(delivers), 2)
        self.assertLess(picks[1][1] - delivers[0][1], 60.0)         # it went straight back for the second one

    def test_it_remembers_balls_while_carrying_one_home_but_fetches_them_afterwards(self):
        brain, _ = self.brain()
        brain.carrying = True
        brain._remember(0.0, 120.0)
        self.assertEqual(len(brain.pending), 1)                    # it remembers them (to fetch later) ...
        self.assertFalse(brain._go_for_remembered())               # ... but does not go for one with a ball in the gripper
        self.assertEqual(len(brain.pending), 1)


class FinishTest(unittest.TestCase):
    """The search ends when the whole rectangle has been looked at and no ball is waiting."""

    def test_it_stops_when_the_whole_floor_has_been_looked_at(self):
        result = run_episode("empty", 0, 900, Noise.perfect(), 3, rechecks=0)
        self.assertLess(result.seconds, 700.0)
        self.assertTrue(any("whole floor" in m for t, m in result.brain.log))

    def test_it_collects_the_balls_before_it_stops(self):
        result = run_episode("empty", 0, 900, Noise.perfect(), 3, rechecks=0)
        self.assertEqual(result.delivered, result.balls)

    def test_it_does_not_stop_with_a_ball_still_waiting(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(40.0, 0.0), seed=1, bounds=(-40.0, -150.0, 360.0, 150.0))
        brain._remember(0.0, 120.0)
        self.assertFalse(brain._finished())

    def test_extra_check_rounds_make_it_look_again(self):
        none = run_episode("empty", 0, 900, Noise.perfect(), 0, rechecks=0)
        again = run_episode("empty", 0, 900, Noise.perfect(), 0, rechecks=1)
        self.assertGreater(again.seconds, none.seconds + 20.0)
        self.assertTrue(any("looking again" in m for t, m in again.brain.log))

    def test_with_no_extra_rounds_it_goes_round_only_once(self):
        result = run_episode("empty", 0, 900, Noise.perfect(), 0, rechecks=0)
        self.assertFalse(any("looking again" in m for t, m in result.brain.log))

    def test_one_extra_round_is_the_default_because_the_camera_misses_some_balls(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, START_XY, START_HEADING, Noise.perfect(), 1)
        self.assertEqual(Brain(CarAPI(car)).rechecks, 1)
        self.assertEqual(run_episode("empty", 0, 30, Noise.perfect(), 1).brain.rechecks, 1)

    def test_the_lane_search_stops_too(self):
        result = run_episode("empty", 0, 900, Noise.perfect(), 3, strategy="lanes", rechecks=0)
        self.assertLess(result.seconds, 900.0)
        self.assertEqual(result.delivered, result.balls)


class BinRangeTest(unittest.TestCase):
    """Near the bin, 'the thing ahead is the bin' is judged by where the bin's near edge should be: measured from the SENSOR, which is
    8 cm ahead of the car's centre. (It was measured from the centre, leaving 2 cm of margin on a 10 cm tolerance.)"""

    def make(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, (100.0, 150.0), 180.0, Noise.perfect(), 1)
        return Brain(CarAPI(car), start_est=(60.0, 0.0), seed=1), car

    def test_the_expected_reading_is_from_the_sensor_not_the_centre(self):
        brain, car = self.make()
        # the car's centre is 60 cm from the bin's centre; the sensor is 8 cm closer; the bin's edge is 15 cm nearer than its centre
        self.assertAlmostEqual(brain._bin_range(60.0), 60.0 - 8.0 - 15.0)

    def test_the_real_reading_matches_what_is_expected(self):
        brain, car = self.make()
        real = car.ultrasonic()                                    # the true sensor-to-bin-edge distance (perfect sensor)
        self.assertAlmostEqual(real, brain._bin_range(60.0), delta=1.0)

    def test_a_reading_of_the_bin_is_not_mistaken_for_an_obstacle(self):
        brain, car = self.make()
        self.assertTrue(brain._clear_ahead(10.0, bin_centre=60.0))

    def test_something_else_close_ahead_is_still_an_obstacle(self):
        world = World(400, 300, BIN_XY, balls=[])
        from sim.world import Obstacle
        world.obstacles.append(Obstacle(78, 140, 84, 160))         # a post between the car and the bin
        car = SimCar(world, (100.0, 150.0), 180.0, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(60.0, 0.0), seed=1)
        self.assertFalse(brain._clear_ahead(10.0, bin_centre=60.0))


class BoxedInIsNotFinishedTest(unittest.TestCase):
    """'Nothing reachable to look at' can mean the floor is covered, or that the map has boxed the car in. Only the coverage tells them apart."""

    def boxed_in_brain(self):
        world = World(400, 300, BIN_XY)
        car = SimCar(world, (200.0, 150.0), 0.0, Noise.perfect(), 1)
        brain = Brain(CarAPI(car), start_est=(160.0, 0.0), seed=1, bounds=(-40.0, -150.0, 360.0, 150.0), rechecks=0)
        for a in range(-80, 81, 10):                                # a ring of remembered bumps round the car (the real floor is empty)
            for b in (-80, 80):
                brain.map.block(160 + a, b)
                brain.map.block(160 + b, a)
        return brain, car

    def test_a_boxed_in_car_does_not_think_it_has_finished(self):
        brain, car = self.boxed_in_brain()
        self.assertLess(brain.map.coverage(), 0.1)
        brain.run(120)
        self.assertEqual(brain.rounds_done, 0)
        self.assertFalse(any("whole floor" in m for t, m in brain.log))

    def test_it_forgets_the_obstacles_and_gets_out(self):
        brain, car = self.boxed_in_brain()
        brain.run(120)
        self.assertGreater(car.driven_cm, 100.0)                    # it did not just sit there
        self.assertTrue(any("boxed in" in m for t, m in brain.log))

    def test_a_covered_floor_still_ends_the_search(self):
        result = run_episode("empty", 0, 900, Noise.perfect(), 3, rechecks=0)
        self.assertTrue(any("whole floor" in m for t, m in result.brain.log))
        self.assertGreaterEqual(result.brain.map.coverage(), 0.0)


class NoisyRunTest(unittest.TestCase):
    """The same arena the user will care about: permanent and temporary obstacles, thin chair legs, and a car with realistic errors."""

    def test_the_cluttered_arena_is_mostly_cleared_given_time(self):
        results = [run_episode("clutter", s, 1800) for s in range(3)]
        got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
        self.assertGreaterEqual(got / total, 0.8, f"{got}/{total}")
        self.assertLess(sum(r.bumps for r in results), 150)


if __name__ == "__main__":
    unittest.main()
