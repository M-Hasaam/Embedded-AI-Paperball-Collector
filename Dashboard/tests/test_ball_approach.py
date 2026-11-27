import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from helpers.ball_approach import Approach, Ball, BallTracker, Reading, Telemetry  # noqa: E402
from helpers.ball_detector import Detection, DetectionResult  # noqa: E402
from helpers.ball_distance import turn_to_ball  # noqa: E402
from helpers.grab_spot import GrabSpot  # noqa: E402
from test_moves import FakeCar  # noqa: E402


def result(frame_id, boxes, stamp=None):
    """A detector result on a 640 x 480 frame from (x1, y1, x2, y2) boxes."""
    dets = [Detection(0.9, *b) for b in boxes]
    return DetectionResult(frame_id, dets, 30.0, float(frame_id) if stamp is None else stamp, 640, 480)


BOX = (300, 300, 340, 340)                 # centre x 320 = 0.5 of 640, bottom edge 340


class TrackerTest(unittest.TestCase):
    def test_steady_after_three_frames(self):
        tracker = BallTracker()
        tracker.update(result(1, [BOX]))
        tracker.update(result(2, [BOX]))
        self.assertIsNone(tracker.ball())
        tracker.update(result(3, [BOX], stamp=12.5))
        self.assertEqual(tracker.ball(), Ball(0.5, 340.0, 12.5))

    def test_a_jump_restarts_the_count(self):
        tracker = BallTracker()
        tracker.update(result(1, [BOX]))
        tracker.update(result(2, [BOX]))
        far = (340, 300, 380, 340)                                    # 40 px to the right
        tracker.update(result(3, [far]))
        self.assertIsNone(tracker.ball())
        tracker.update(result(4, [far]))
        self.assertIsNone(tracker.ball())
        tracker.update(result(5, [far]))
        self.assertIsNotNone(tracker.ball())

    def test_a_few_empty_results_do_not_restart_the_count(self):
        # the model flickers on a hard ball (67%, then below the threshold, then 67% again): the ball is still there
        tracker = BallTracker()
        tracker.update(result(1, [BOX]))
        tracker.update(result(2, []))
        tracker.update(result(3, [BOX]))
        tracker.update(result(4, []))
        tracker.update(result(5, [BOX], stamp=7.0))
        self.assertEqual(tracker.ball(), Ball(0.5, 340.0, 7.0))

    def test_a_longer_gap_restarts_the_count(self):
        tracker = BallTracker()
        for i in (1, 2):
            tracker.update(result(i, [BOX]))
        for i in (3, 4, 5):                                           # three empty results in a row: the ball is gone
            tracker.update(result(i, []))
        tracker.update(result(6, [BOX]))
        tracker.update(result(7, [BOX]))
        self.assertIsNone(tracker.ball())
        tracker.update(result(8, [BOX]))
        self.assertIsNotNone(tracker.ball())

    def test_a_steady_ball_is_lost_after_a_long_gap(self):
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [BOX]))
        for i in (4, 5, 6):
            tracker.update(result(i, []))
        self.assertIsNone(tracker.ball())

    def test_a_ball_seen_before_a_gap_is_not_reported_as_new(self):
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [BOX], stamp=float(i)))
        tracker.update(result(4, [], stamp=4.0))
        self.assertEqual(tracker.ball().stamp, 3.0)                   # still the old picture's time, so a decision waits for a new one

    def test_takes_the_lowest_ball(self):
        tracker = BallTracker()
        high, low = (100, 100, 140, 140), (500, 300, 540, 360)
        for i in (1, 2, 3):
            tracker.update(result(i, [high, low]))
        ball = tracker.ball()
        self.assertEqual(ball.bottom_y, 360.0)
        self.assertAlmostEqual(ball.x_ratio, 520 / 640)

    def test_the_same_frame_counts_once(self):
        tracker = BallTracker()
        for _ in range(3):
            tracker.update(result(1, [BOX]))
        self.assertIsNone(tracker.ball())

    def test_a_ball_in_the_gripper_zone_is_ignored(self):
        held = (300, 20, 340, 100)                                    # the bottom edge is in the top fifth of a 480 px picture
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [held]))
        self.assertIsNone(tracker.ball())
        self.assertEqual(tracker.seen_since(0.0), 0)
        for i in (4, 5, 6):
            tracker.update(result(i, [held, BOX]))                    # the floor ball is still found
        self.assertEqual(tracker.ball().bottom_y, 340.0)

    def test_a_far_ball_just_below_the_horizon_is_a_floor_ball(self):
        # a ball ~2 m away has its bottom edge about 34% down a 480 px picture (row 165): it was dropped as "held in the gripper",
        # so Collect said "lost the ball" while the detector showed it at 91%
        far = (270, 130, 300, 165)
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [far]))
        self.assertEqual(tracker.ball().bottom_y, 165.0)

    def test_the_whole_ball_and_its_top_half_flickering_is_still_one_steady_ball(self):
        # close up the model sometimes boxes the whole ball and sometimes only its upper half: the centre jumps ~90 px
        whole, half = (500, 280, 650, 365), (515, 180, 655, 278)
        tracker = BallTracker()
        for i, box in enumerate((whole, half, whole, half, whole), start=1):
            tracker.update(result(i, [box], stamp=i * 0.1))
        self.assertIsNotNone(tracker.ball())

    def test_a_ball_missed_for_a_moment_is_kept_when_the_results_come_fast(self):
        # a moving car blurs the ball: 4 results in a row without it, but only 0.4 s apart
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [BOX], stamp=i * 0.1))
        for i in (4, 5, 6, 7):
            tracker.update(result(i, [], stamp=i * 0.1))
        self.assertIsNotNone(tracker.ball())

    def test_a_ball_missed_for_over_a_second_is_lost(self):
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [BOX], stamp=i * 0.1))
        for i in range(4, 20):
            tracker.update(result(i, [], stamp=i * 0.1))
        self.assertIsNone(tracker.ball())

    def test_counts_the_results_that_showed_a_ball_since_a_time(self):
        tracker = BallTracker()
        tracker.update(result(1, [BOX], stamp=1.0))
        tracker.update(result(2, [], stamp=2.0))
        tracker.update(result(3, [BOX], stamp=3.0))
        tracker.update(result(4, [BOX], stamp=4.0))
        self.assertEqual(tracker.seen_since(0.0), 3)
        self.assertEqual(tracker.seen_since(2.5), 2)
        self.assertEqual(tracker.seen_since(5.0), 0)

    def test_the_ball_carries_its_box_confidence_and_how_many_are_in_view(self):
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(DetectionResult(i, [Detection(0.82, 300, 300, 340, 336), Detection(0.9, 10, 200, 30, 220)], 30.0,
                                           float(i), 640, 480))
        ball = tracker.ball()
        self.assertEqual((ball.width, ball.height, ball.confidence, ball.in_view), (40.0, 36.0, 0.82, 2))
        self.assertAlmostEqual(ball.bottom_ratio, 336 / 480)

    def test_where_the_ball_was_across_the_pictures_since_a_time(self):
        tracker = BallTracker()
        for i, left in ((1, 300), (2, 292), (3, 316), (4, 300)):
            tracker.update(result(i, [(left, 300, left + 40, 340)], stamp=float(i)))
        low, high, n = tracker.spread_since(2.0)
        self.assertEqual(n, 3)
        self.assertAlmostEqual(low, 312 / 640)
        self.assertAlmostEqual(high, 336 / 640)
        self.assertIsNone(tracker.spread_since(9.0))

    def test_reset(self):
        tracker = BallTracker()
        for i in (1, 2, 3):
            tracker.update(result(i, [BOX]))
        tracker.reset()
        self.assertIsNone(tracker.ball())
        tracker.update(result(3, [BOX]))                              # the same frame id as before is new after a reset
        self.assertIsNone(tracker.ball())


class FakeWorld:
    def __init__(self):
        self.ball_value, self.frame, self.reading_value, self.camera = None, None, None, lambda ball: None

    def ball(self):
        return self.ball_value

    def frame_stamp(self):
        return self.frame

    def reading(self):
        return self.reading_value

    def camera_cm(self, ball):
        return self.camera(ball)

    def balls_seen_since(self, stamp):
        return sum(1 for t in getattr(self, "seen", []) if t >= stamp)

    def ball_spread(self, since):
        return getattr(self, "spread", None)

    def telemetry(self):
        return getattr(self, "sensors", Telemetry())


class FakePlayer:
    def __init__(self):
        self.active, self.ok, self.status = False, None, ""
        self.started, self.aborted, self.ticks = [], [], 0

    def start(self, move, now):
        self.started.append(move)
        self.active = True
        return True

    def tick(self, now):
        self.ticks += 1

    def abort(self, reason):
        self.aborted.append(reason)
        self.active, self.ok, self.status = False, False, f"Stopped: {reason}"

    def finish(self, ok):
        self.active, self.ok = False, ok
        self.status = "Done" if ok else "Stopped: the arm failed"


MOVE = {"name": "pick_down_up", "steps": []}


class ApproachTest(unittest.TestCase):
    def setUp(self):
        self.world, self.car, self.player = FakeWorld(), FakeCar(), FakePlayer()
        self.approach = Approach(self.world, self.car, self.player, MOVE, GrabSpot(0.5, 16.0))
        self.now = 0.0

    def see(self, r, x=0.5, camera=None, at=None):
        """Fresh data at time `at` (default now): the ball at x, the sensor at r, the camera model's answer."""
        at = self.now if at is None else at
        self.world.ball_value = Ball(x, 300.0, at)
        self.world.frame = at
        self.world.reading_value = Reading(r, at)
        self.world.camera = lambda ball: camera

    def advance(self, seconds=1.0):
        """Tick every 0.1 s for `seconds`, keeping the data fresh like the real loop does."""
        for _ in range(int(round(seconds * 10))):
            self.now += 0.1
            if self.world.ball_value is not None:
                self.world.ball_value = replace(self.world.ball_value, stamp=self.now)
                self.world.seen = getattr(self.world, "seen", []) + [self.now]
            if self.world.reading_value is not None:
                self.world.reading_value = Reading(self.world.reading_value.cm, self.now)
            self.world.frame = self.now
            self.approach.tick(self.now)

    def go(self, r, x=0.5, camera=None):
        self.see(r, x, camera)
        self.assertTrue(self.approach.start(self.now))
        self.advance(1.0)

    def drives(self):
        return [c for c in self.car.calls if c[0] == "start_drive"]

    def asked(self, x, cm):
        """The turn the car should be asked for: what the ball needs, plus the 1 degree the car stops short."""
        need = turn_to_ball(x, cm, 0.5, 16.0)
        return need + (1.0 if need > 0 else -1.0)

    def test_turns_toward_a_ball_off_the_taught_direction(self):
        self.go(40, x=0.2)
        self.assertAlmostEqual(self.car.calls[0][1], self.asked(0.2, 40), places=1)
        self.assertGreater(self.car.calls[0][1], 5)                   # to the left (+)
        self.assertEqual(self.drives(), [])
        self.setUp()
        self.go(40, x=0.8)
        self.assertAlmostEqual(self.car.calls[0][1], self.asked(0.8, 40), places=1)
        self.assertLess(self.car.calls[0][1], -5)

    def test_asks_for_a_degree_more_than_needed_because_the_car_stops_short(self):
        self.go(100, x=0.44)
        need = turn_to_ball(0.44, 100, 0.5, 16.0)
        self.assertAlmostEqual(self.car.calls[0][1] - need, 1.0, places=1)

    def test_a_far_ball_a_little_off_the_line_is_turned_to(self):
        self.go(110, x=0.46)                                          # the old maths left a far ball like this uncentred
        self.assertEqual(self.car.kinds()[0], "start_turn")
        self.assertGreater(self.car.calls[0][1], 2.5)                 # enough for the car to really move

    def test_the_same_place_across_the_picture_close_up_needs_no_turn(self):
        self.go(17, x=0.46)                                           # 17 cm away that is under 1 cm to the side
        self.assertNotIn("start_turn", self.car.kinds())
        self.assertEqual(self.player.started, [MOVE])

    def test_a_far_reading_is_not_the_distance_of_a_ball_off_to_the_side(self):
        self.go(150, x=0.2)                                           # the sensor looks past the ball: assume 40 cm
        self.assertAlmostEqual(self.car.calls[0][1], self.asked(0.2, 40), places=1)

    def test_too_close_and_off_to_one_side_backs_up_before_turning(self):
        self.go(11, x=0.3)                                            # this close the picture jumps about: no turning
        self.assertEqual(self.car.kinds()[:1], ["start_drive"])
        self.assertEqual(self.drives(), [("start_drive", -5.0, 50, 3)])

    def test_no_turn_within_three_degrees(self):
        self.go(40, x=0.52)
        self.assertNotIn("start_turn", self.car.kinds())
        self.assertEqual(len(self.drives()), 1)

    def test_hops_from_the_sensor(self):
        for r, cm, speed in ((40, 15.0, 50), (25, 6.3, 50), (19, 3.0, 50), (59, 15.0, 50)):
            self.setUp()
            self.go(r)
            self.assertEqual(self.drives(), [("start_drive", cm, speed, 3)], msg=f"reading {r}")

    def test_far_or_no_echo_without_calibration_hops_ten(self):
        for r in (None, 100):
            self.setUp()
            self.go(r)
            self.assertEqual(self.drives(), [("start_drive", 10.0, 50, 3)], msg=f"reading {r}")

    def test_far_with_calibration_hops_by_the_camera(self):
        for c, cm, speed in ((70, 25.0, 100), (30, 9.0, 50)):
            self.setUp()
            self.go(None, camera=c)
            self.assertEqual(self.drives(), [("start_drive", cm, speed, 3)], msg=f"camera {c}")

    def test_in_range_with_the_ball_in_view_starts_the_move(self):
        self.go(16.2)
        self.assertEqual(self.player.started, [MOVE])
        self.assertEqual(self.drives(), [])
        self.player.finish(True)
        self.world.ball_value = None                                  # the ball is in the gripper: not on the floor any more
        self.advance(2.0)
        self.assertIs(self.approach.ok, True)
        self.assertIn("picked", self.approach.status)
        self.assertFalse(self.approach.active)

    def test_a_ball_still_on_the_floor_after_the_move_is_a_miss_and_it_tries_again(self):
        self.go(16.2)
        self.player.finish(True)
        self.advance(2.0)                                             # the ball is still seen
        self.assertTrue(self.approach.active)
        self.assertEqual(self.drives(), [("start_drive", -10.0, 50, 3)])
        self.assertIn("2", self.approach.status)                      # attempt 2
        self.car.finish_motion()
        self.advance(2.0)                                             # backed up: the ball is at the pick distance again
        self.assertEqual(self.player.started, [MOVE, MOVE])
        self.player.finish(True)
        self.world.ball_value = None
        self.advance(2.0)
        self.assertIs(self.approach.ok, True)

    def test_gives_up_after_five_misses(self):
        self.go(16.2)
        for _ in range(10):
            if not self.approach.active:
                break
            self.player.finish(True)
            self.advance(2.0)
            self.car.finish_motion()
            self.advance(2.0)
        self.assertFalse(self.approach.active)
        self.assertIs(self.approach.ok, False)
        self.assertIn("5", self.approach.status)
        self.assertEqual(len(self.player.started), 5)

    def test_waits_before_judging_the_pick(self):
        self.go(16.2)
        self.player.finish(True)
        self.world.ball_value = None
        self.advance(0.5)
        self.assertTrue(self.approach.active)                         # not yet: the car stands still for a while first

    def test_no_new_camera_frames_while_judging_stops_it(self):
        self.go(16.2)
        self.player.finish(True)
        self.world.ball_value = None
        self.advance(0.3)
        self.world.frame = -10.0
        self.now += 2.0
        self.approach.tick(self.now)
        self.assertTrue(self.approach.active)                         # no picture since the move: it cannot say "picked" yet, so it waits
        self.assertIsNone(self.approach.ok)
        self.now += 9.0
        self.world.reading_value = Reading(16.2, self.now)
        self.approach.tick(self.now)
        self.assertFalse(self.approach.active)                        # and gives up if the camera does not come back
        self.assertIn("camera", self.approach.status)

    def test_the_move_failing_stops_it(self):
        self.go(16.2)
        self.player.finish(False)
        self.advance(0.2)
        self.assertIs(self.approach.ok, False)
        self.assertFalse(self.approach.active)

    def test_in_range_but_the_camera_disagrees_does_not_pick(self):
        self.go(16, camera=45)
        self.assertFalse(self.approach.active)
        self.assertIn("camera", self.approach.status)
        self.assertEqual(self.player.started, [])
        self.assertEqual(self.drives(), [])

    def test_in_range_without_a_ball_never_picks(self):
        self.see(16)
        self.world.ball_value = None
        self.approach.start(self.now)
        self.advance(3.5)
        self.assertFalse(self.approach.active)
        self.assertIn("lost the ball", self.approach.status)
        self.assertEqual(self.player.started, [])

    def test_something_closer_than_the_ball_stops(self):
        for r, c in ((30, 60), (10, 40)):
            self.setUp()
            self.go(r, camera=c)
            self.assertFalse(self.approach.active, msg=f"{r}/{c}")
            self.assertIn("closer", self.approach.status)

    def test_beam_beyond_the_ball_uses_the_camera(self):
        self.go(50, camera=30)
        self.assertEqual(self.drives(), [("start_drive", 9.0, 50, 3)])

    def test_too_close_backs_up(self):
        for r, cm in ((10, -6.0), (13.6, -3.0)):
            self.setUp()
            self.go(r)
            self.assertEqual(self.drives(), [("start_drive", cm, 50, 3)], msg=f"reading {r}")

    def test_never_drives_past_the_pick_distance(self):
        r = 19.0
        while r <= 60.0:
            self.setUp()
            self.go(r)
            (_, hop, _, _), = self.drives()
            self.assertLessEqual(hop, max(3.0, r - 16.0), msg=f"reading {r}")
            self.assertTrue(3.0 <= hop <= 25.0)
            r += 0.5

    # -- near the ball: slow, and short of the target (seen on the car: a 21 cm hop at 100% ended 3-7 cm from the ball) --
    def test_never_drives_fast_when_the_sensor_sees_something_near(self):
        r = 19.0
        while r <= 60.0:
            self.setUp()
            self.go(r)
            (_, _, speed, _), = self.drives()
            self.assertEqual(speed, 50, msg=f"reading {r}")
            r += 0.5

    def test_a_longer_approach_leaves_room_for_overshooting(self):
        for r in (30.0, 37.6, 45.0):
            self.setUp()
            self.go(r)
            (_, hop, _, _), = self.drives()
            self.assertLessEqual(hop, 0.75 * (r - 16.0), msg=f"reading {r}")
            self.assertGreater(hop, 0.5 * (r - 16.0), msg=f"reading {r}")

    def test_the_last_small_step_goes_all_the_way(self):
        self.go(23.0)                                                 # 7 cm to go: creep the whole of it
        self.assertEqual(self.drives(), [("start_drive", 7.0, 50, 3)])

    def test_a_far_ball_by_the_camera_is_still_approached_fast(self):
        self.go(None, camera=90)
        self.assertEqual(self.drives(), [("start_drive", 25.0, 100, 3)])

    # -- far hops are long and fast, near hops short and slow (it was the other way round: 10 cm hops far away) --
    def go_seeing(self, r, size):
        self.see(r)
        self.world.ball_value = replace(self.world.ball_value, width=float(size), height=float(size) * 0.85)
        self.approach.start(self.now)
        self.advance(1.0)

    def test_a_ball_that_looks_far_is_approached_in_long_fast_hops(self):
        for r in (150, None):
            self.setUp()
            self.go_seeing(r, 35)                                     # 35 px wide: at least 85 cm away
            self.assertEqual(self.drives(), [("start_drive", 25.0, 100, 3)], msg=f"reading {r}")

    def test_the_hop_shrinks_and_slows_as_the_ball_looks_nearer(self):
        for size, cm, speed in ((35, 25.0, 100), (60, 23.8, 50), (80, 15.0, 50), (100, 10.0, 50), (250, 10.0, 50)):
            self.setUp()
            self.go_seeing(150, size)
            self.assertEqual(self.drives(), [("start_drive", cm, speed, 3)], msg=f"{size} px")

    def test_a_near_hop_is_never_longer_or_faster_than_a_far_one(self):
        hops = []
        for r, size in ((150, 35), (100, 50), (59, 68), (45, 90), (30, 135), (22, 180)):
            self.setUp()
            self.go_seeing(r, size)
            (_, cm, speed, _), = self.drives()
            hops.append((cm, speed))
        self.assertEqual(hops, sorted(hops, reverse=True), hops)

    def test_the_log_says_why_the_far_hop_is_that_long(self):
        self.go_seeing(150, 35)
        self.assertIn("beyond the sensor: the ball is 35 px wide, so at least 86 cm away", self.text())

    # -- too close to be seen: back up and look again instead of giving up --
    def hop_then_lose_the_ball(self, reading):
        """One forward hop, after which the camera sees no ball and the sensor reads `reading`."""
        self.go(40)
        self.assertGreater(self.drives()[0][1], 0)
        self.world.ball_value = None
        self.world.reading_value = Reading(reading, self.now)
        self.car.finish_motion()

    def test_a_ball_that_vanishes_after_driving_forward_is_backed_away_from(self):
        self.hop_then_lose_the_ball(30.0)
        self.advance(4.5)
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertEqual(self.drives()[1:], [("start_drive", -12.0, 50, 3)])

    def test_something_closer_than_the_pick_range_and_no_ball_in_view_means_too_close(self):
        self.hop_then_lose_the_ball(5.0)
        self.advance(2.5)                                             # no need to wait the whole lost-ball time
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertEqual(self.drives()[1:], [("start_drive", -17.0, 50, 3)])       # back to the pick distance and a bit more

    def test_after_backing_up_the_ball_is_seen_again_and_picked(self):
        self.hop_then_lose_the_ball(5.0)
        self.advance(2.5)
        self.car.finish_motion()
        self.see(16.2)
        self.advance(1.5)
        self.assertEqual(self.player.started, [MOVE])

    def test_pressing_collect_with_the_ball_already_too_close_backs_up(self):
        self.world.ball_value = None
        self.world.reading_value = Reading(5.0, self.now)
        self.world.frame = self.now
        self.approach.start(self.now)
        self.advance(2.5)
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertEqual(self.drives(), [("start_drive", -17.0, 50, 3)])

    def test_the_ball_gets_its_full_time_to_be_found_after_the_car_stops(self):
        self.go(40)
        self.world.ball_value = None
        self.advance(4.0)                                             # a long hop: the camera sees nothing while it drives
        self.car.finish_motion()
        self.advance(1.5)                                             # standing still for only 1.5 s
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertEqual(len(self.drives()), 1)

    def test_it_gives_up_after_backing_up_three_times(self):
        self.hop_then_lose_the_ball(30.0)
        for _ in range(12):
            if not self.approach.active:
                break
            self.advance(4.5)
            self.car.finish_motion()
        self.assertFalse(self.approach.active)
        self.assertIn("lost the ball", self.approach.status)
        self.assertEqual(sum(1 for d in self.drives() if d[1] < 0), 3)

    def test_it_does_not_back_up_for_a_ball_it_has_not_driven_toward(self):
        self.see(30.0)
        self.world.ball_value = None                                  # nothing close and no drive yet: just lost
        self.approach.start(self.now)
        self.advance(4.0)
        self.assertFalse(self.approach.active)
        self.assertEqual(self.drives(), [])

    def test_waits_for_fresh_data_after_a_motion(self):
        self.see(40, at=0.0)
        self.approach.start(0.0)
        self.approach.tick(0.7)
        self.assertEqual(len(self.drives()), 1)
        self.car.finish_motion()
        self.approach.tick(5.0)                                       # the hop has ended: standing still from t = 5.0
        self.world.frame, self.world.reading_value = 5.9, Reading(25, 5.2)
        self.world.ball_value = Ball(0.5, 300.0, 5.4)
        self.approach.tick(6.0)                                       # the reading is from before it settled
        self.assertEqual(len(self.drives()), 1)
        self.world.ball_value = Ball(0.5, 300.0, 5.2)                 # a fresh reading, but the picture is from before it settled
        self.world.reading_value = Reading(25, 5.6)
        self.approach.tick(6.0)
        self.assertEqual(len(self.drives()), 1)
        self.world.ball_value = Ball(0.5, 300.0, 5.4)
        self.approach.tick(6.0)
        self.assertEqual(len(self.drives()), 2)

    def test_stops_on_stale_or_missing_data(self):
        self.see(40)
        self.approach.start(0.0)
        self.world.frame = -2.0
        self.world.reading_value = Reading(40, 10.5)
        self.approach.tick(10.5)
        self.assertIn("camera", self.approach.status)
        self.assertEqual(self.car.kinds().count("stop"), 1)
        self.setUp()
        self.see(40)
        self.approach.start(0.0)
        self.world.reading_value = Reading(40, -3.0)
        self.approach.tick(0.7)
        self.assertIn("distance", self.approach.status)
        self.assertEqual(self.car.kinds().count("stop"), 1)

    def test_a_camera_pause_while_the_car_drives_is_forgiven(self):
        # the camera link drops while the motors run: that is not a dead camera, as long as frames come back soon after it stops
        self.see(40, at=0.0)
        self.approach.start(0.0)
        self.approach.tick(0.7)
        self.assertEqual(len(self.drives()), 1)
        self.world.frame = 0.7                                        # no new camera result during the whole hop
        self.car.finish_motion()
        self.approach.tick(5.0)                                       # the hop ended at t = 5.0
        self.world.reading_value = Reading(25, 5.6)
        self.approach.tick(6.0)                                       # 5.3 s since the last frame, but only 1 s since it stopped
        self.assertTrue(self.approach.active)
        self.world.ball_value = Ball(0.5, 300.0, 6.0)
        self.world.frame = 6.0                                        # the camera is back: it carries on
        self.approach.tick(6.2)
        self.assertEqual(len(self.drives()), 2)

    def test_a_camera_that_stays_silent_after_the_car_stops_ends_the_run(self):
        self.see(40, at=0.0)
        self.approach.start(0.0)
        self.approach.tick(0.7)
        self.world.frame = 0.7
        self.car.finish_motion()
        self.approach.tick(5.0)
        self.world.reading_value = Reading(25, 15.2)
        self.approach.tick(15.4)                                      # over 10 s after the car stopped and still no picture
        self.assertFalse(self.approach.active)
        self.assertIn("camera", self.approach.status)

    # -- the Wi-Fi camera pauses now and then (seen on the car: 2.7 s with the car standing still) --
    def stand_still_then_lose_the_camera(self, seconds, ball=True):
        """A hop ends at t = 5.0, one picture arrives, then nothing for `seconds` while the distance readings carry on."""
        self.see(40, at=0.0)
        self.approach.start(0.0)
        self.approach.tick(0.7)
        self.car.finish_motion()
        self.approach.tick(5.0)
        self.world.frame = 5.4
        self.world.ball_value = Ball(0.5, 300.0, 5.4) if ball else None
        t = 5.4
        while t < 5.4 + seconds:
            t = round(t + 0.1, 1)
            self.world.reading_value = Reading(25, t)
            if t < 5.7:                                               # not yet long enough to decide on that one picture
                continue
            self.world.ball_value = None if not ball else self.world.ball_value
            self.approach.tick(t)
        self.now = t

    def test_a_camera_pause_while_the_car_stands_still_is_waited_out(self):
        self.stand_still_then_lose_the_camera(2.7, ball=False)
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertIn("Waiting for the camera", self.approach.status)
        self.assertEqual(len(self.drives()), 1)                       # it does not move without a picture
        self.see(25)
        self.advance(1.0)                                             # the camera is back
        self.assertEqual(len(self.drives()), 2)

    def test_a_pause_is_not_counted_as_a_lost_ball(self):
        self.stand_still_then_lose_the_camera(6.0, ball=False)
        self.world.reading_value = Reading(25, self.now)
        self.advance(2.0)                                             # pictures again, the ball not found in them yet
        self.assertTrue(self.approach.active, self.approach.status)
        self.assertEqual(len(self.drives()), 1)                       # nor does it back up for a ball it "lost" in the pause
        self.see(25)
        self.advance(1.0)
        self.assertEqual(len(self.drives()), 2)

    def test_the_log_says_when_it_waited_for_the_camera(self):
        self.stand_still_then_lose_the_camera(2.7, ball=False)
        self.see(25)
        self.advance(1.0)
        self.assertEqual(self.text().count("waiting: no camera picture"), 1)
        self.assertIn("waiting: the camera is back after", self.text())

    def test_a_camera_that_does_not_come_back_ends_the_run(self):
        self.stand_still_then_lose_the_camera(11.0, ball=False)
        self.assertFalse(self.approach.active)
        self.assertIn("no new camera frame", self.approach.status)
        self.assertEqual(len(self.drives()), 1)

    def test_one_picture_after_a_pause_is_not_enough_to_call_it_picked(self):
        self.go(16.2)
        self.player.finish(True)
        self.advance(0.1)
        self.world.ball_value = None
        self.now += 4.0                                               # silence
        self.world.reading_value = Reading(16.2, self.now)
        self.approach.tick(self.now)
        self.world.frame = self.now                                   # a single picture, no ball in it
        self.approach.tick(self.now)
        self.assertTrue(self.approach.active, self.approach.status)
        self.advance(1.5)
        self.assertIs(self.approach.ok, True)

    def test_limits(self):
        self.see(40)
        self.approach.start(0.0)
        for _ in range(60):                                           # every hop ends at once; the car never gets closer
            self.advance(0.7)
            self.car.finish_motion()
            if not self.approach.active:
                break
        self.assertFalse(self.approach.active)
        self.assertIn("too many steps", self.approach.status)
        self.assertEqual(len(self.drives()), 25)
        self.setUp()
        self.see(40)
        self.approach.start(0.0)
        self.approach.tick(91.0)
        self.assertFalse(self.approach.active)

    def test_the_car_refusing_or_failing(self):
        self.car.start_result = False
        self.go(40)
        self.assertIn("could not start the drive", self.approach.status)
        self.setUp()
        self.car.start_result = False
        self.go(40, x=0.2)
        self.assertIn("could not start the turn", self.approach.status)
        self.setUp()
        self.go(40)
        self.car.motion_result = False
        self.car.finish_motion()
        self.advance(0.2)
        self.assertFalse(self.approach.active)
        self.assertIs(self.approach.ok, False)

    def test_abort_stops_everything_and_is_harmless_twice(self):
        self.go(16.2)
        self.approach.abort("user")
        self.assertEqual(self.car.kinds().count("stop"), 1)
        self.assertEqual(self.player.aborted, ["user"])
        self.approach.abort("again")
        self.assertEqual(self.car.kinds().count("stop"), 1)
        self.assertEqual(self.player.aborted, ["user"])

    # -- the log says what the camera and the sensors saw, what was decided and what the car then did --
    def text(self):
        return "\n".join(self.approach.log)

    def test_each_decision_says_what_the_camera_and_sensors_saw(self):
        self.see(40, x=0.52)
        self.world.ball_value = replace(self.world.ball_value, width=40.0, height=30.0, confidence=0.82, in_view=2,
                                        bottom_ratio=0.625)
        self.world.spread = (0.51, 0.53, 9)
        self.world.sensors = Telemetry(12.0, 5.0, 6.0, "143 mm")
        self.approach.start(self.now)
        self.advance(1.0)
        for part in ("step 1", "attempt 1", "52% across", "2% right of the grab line", "box 40x30 px", "62% down", "sure 82%",
                     "2 balls in view", "9 pictures: 51-53%", "front 40.0 cm", "gripper 143 mm", "heading +12.0",
                     "wheels L 5.0 R 6.0 cm", "decided: drive forward 15.0 cm at 50%"):
            self.assertIn(part, self.text())

    def test_the_decision_to_turn_explains_the_angle(self):
        self.go(100, x=0.44)
        need = turn_to_ball(0.44, 100, 0.5, 16.0)
        for part in (f"decided: turn left {need + 1:.1f} deg", f"ball needs {need:.1f}", "stops short", "100 cm (sensor)"):
            self.assertIn(part, self.text())

    def test_the_result_of_a_turn_is_logged_from_the_gyro_and_wheels(self):
        self.world.sensors = Telemetry(10.0, 5.0, 6.0, "143 mm")
        self.go(40, x=0.8)
        self.assertNotIn("result", self.text())
        self.world.sensors = Telemetry(7.6, 6.2, 4.9, "143 mm")
        self.car.finish_motion()
        self.see(40)
        self.advance(1.0)
        asked = abs(self.car.calls[0][1])
        self.assertIn(f"result : turned right 2.4 deg (asked right {asked:.1f}); wheels L +1.2 R -1.1 cm", self.text())
        self.assertEqual(self.text().count("result :"), 1)

    def test_the_result_of_a_drive_is_logged_from_the_wheels_and_gyro(self):
        self.world.sensors = Telemetry(10.0, 100.0, 200.0, "143 mm")
        self.go(40)
        self.world.sensors = Telemetry(10.4, 117.2, 216.4, "143 mm")
        self.car.finish_motion()
        self.see(22)
        self.advance(1.0)
        self.assertIn("result : drove (asked +15.0 cm); wheels L +17.2 R +16.4 cm; heading moved left 0.4 deg", self.text())

    def test_a_stop_says_where_things_stood(self):
        self.go(40)
        self.world.sensors = Telemetry(1.0, 2.0, 3.0, "90 mm")
        self.world.ball_value = None
        self.approach.abort("stopped by you")
        for part in ("result :", "no ball in view; last seen 50% across", "front 40.0 cm", "gripper 90 mm"):
            self.assertIn(part, self.text())

    def test_the_check_after_the_pick_is_logged_with_the_gripper(self):
        self.go(16.2)
        self.world.sensors = Telemetry(0.0, 0.0, 0.0, "35 mm")
        self.player.finish(True)
        self.world.ball_value = None
        self.advance(2.0)
        self.assertIn("check  : no ball left on the floor (seen in 0 pictures after the move)", self.text())
        self.assertIn("gripper 35 mm", self.text().split("check  :")[1])

    def test_missing_sensors_do_not_break_the_log(self):
        self.go(None)
        self.car.finish_motion()
        self.advance(1.0)
        self.assertIn("front: nothing in range", self.text())
        self.assertIn("result : drove (asked +10.0 cm)", self.text())

    def test_start_refused_when_already_active_or_disconnected(self):
        self.see(40)
        self.assertTrue(self.approach.start(0.0))
        self.assertFalse(self.approach.start(0.1))
        self.setUp()
        self.car.connected_flag = False
        self.assertFalse(self.approach.start(0.0))


if __name__ == "__main__":
    unittest.main()
