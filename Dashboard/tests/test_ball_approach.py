import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from helpers.ball_approach import Approach, Ball, BallTracker, Reading  # noqa: E402
from helpers.ball_detector import Detection, DetectionResult  # noqa: E402
from helpers.ball_distance import ball_angle_deg  # noqa: E402
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

    def test_counts_the_results_that_showed_a_ball_since_a_time(self):
        tracker = BallTracker()
        tracker.update(result(1, [BOX], stamp=1.0))
        tracker.update(result(2, [], stamp=2.0))
        tracker.update(result(3, [BOX], stamp=3.0))
        tracker.update(result(4, [BOX], stamp=4.0))
        self.assertEqual(tracker.seen_since(0.0), 3)
        self.assertEqual(tracker.seen_since(2.5), 2)
        self.assertEqual(tracker.seen_since(5.0), 0)

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
                self.world.ball_value = Ball(self.world.ball_value.x_ratio, self.world.ball_value.bottom_y, self.now)
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

    def test_turns_toward_a_ball_off_the_taught_direction(self):
        self.go(40, x=0.2)
        self.assertAlmostEqual(self.car.calls[0][1], ball_angle_deg(0.2) - ball_angle_deg(0.5), places=1)
        self.assertGreater(self.car.calls[0][1], 5)                   # to the left (+)
        self.assertEqual(self.drives(), [])
        self.setUp()
        self.go(40, x=0.8)
        self.assertAlmostEqual(self.car.calls[0][1], ball_angle_deg(0.8) - ball_angle_deg(0.5), places=1)
        self.assertLess(self.car.calls[0][1], -5)

    def test_no_turn_within_three_degrees(self):
        self.go(40, x=0.52)
        self.assertNotIn("start_turn", self.car.kinds())
        self.assertEqual(len(self.drives()), 1)

    def test_hops_from_the_sensor(self):
        for r, cm, speed in ((40, 24.0, 100), (25, 9.0, 50), (19, 3.0, 50), (59, 25.0, 100)):
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
        self.now += 1.5
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
        self.approach.tick(0.7)
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
        self.world.reading_value = Reading(25, 8.0)
        self.approach.tick(8.2)                                       # 3.2 s after the car stopped and still no picture
        self.assertFalse(self.approach.active)
        self.assertIn("camera", self.approach.status)

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

    def test_start_refused_when_already_active_or_disconnected(self):
        self.see(40)
        self.assertTrue(self.approach.start(0.0))
        self.assertFalse(self.approach.start(0.1))
        self.setUp()
        self.car.connected_flag = False
        self.assertFalse(self.approach.start(0.0))


if __name__ == "__main__":
    unittest.main()
