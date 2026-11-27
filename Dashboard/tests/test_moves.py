import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from helpers.moves import LogTail, Readings, RepeatPlayer  # noqa: E402
from helpers.moves import (ARM_MARGIN_S, MoveFileError, MovePlayer, arm_duration, arm_step, describe_step,  # noqa: E402
                   drive_step, load_move, new_move, save_move, sync_speeds, turn_step, validate_move, wait_step)

LIMITS = [(0, 120), (80, 180), (40, 170), (0, 180)]
NAMES = ["Gripper", "Elbow", "Shoulder", "Servo 4"]


def move_with(*steps):
    move = new_move("t")
    move["steps"] = list(steps)
    return move


def one_of_each():
    move = new_move("all")
    move["steps"] = [arm_step({1: (150, 30), 2: (120, 30)}, wait=0.5), drive_step(20), turn_step(90), wait_step(1)]
    return move


class StepsTest(unittest.TestCase):
    def test_arm_step_sorts_servos(self):
        step = arm_step({2: (120, 30), 1: (150, 30)})
        self.assertEqual(step["servos"], [{"i": 1, "angle": 150, "speed": 30}, {"i": 2, "angle": 120, "speed": 30}])

    def test_describe_each_type(self):
        self.assertEqual(describe_step(arm_step({1: (150, 30), 2: (120, 30)}, wait=0.5), NAMES),
                         "Arm: Elbow 150°@30/s, Shoulder 120°@30/s, then wait 0.5 s")
        self.assertEqual(describe_step(drive_step(20), NAMES), "Drive: forward 20 cm at 100%")
        self.assertEqual(describe_step(drive_step(-10, 50), NAMES), "Drive: back 10 cm at 50%")
        self.assertEqual(describe_step({"type": "drive", "cm": 20}, NAMES), "Drive: forward 20 cm at 100%")   # older files: no speed
        self.assertEqual(describe_step(turn_step(90), NAMES), "Turn: left 90°")
        self.assertEqual(describe_step(turn_step(-90), NAMES), "Turn: right 90°")
        self.assertEqual(describe_step(wait_step(1), NAMES), "Wait: 1 s")

    def test_arm_duration_uses_slowest_servo(self):
        step = arm_step({1: (150, 30), 2: (120, 60)}, wait=0.5)
        self.assertAlmostEqual(arm_duration(step, {1: 120, 2: 90}), 1.0 + ARM_MARGIN_S + 0.5)

    def test_arm_duration_without_movement(self):
        step = arm_step({1: (150, 30)}, wait=0.5)
        self.assertAlmostEqual(arm_duration(step, {1: 150}), ARM_MARGIN_S + 0.5)


class SyncTest(unittest.TestCase):
    def speeds(self, step):
        return {s["i"]: s["speed"] for s in step["servos"]}

    def test_sync_makes_servos_arrive_together(self):
        step = arm_step({1: (100, 60), 2: (140, 60)})            # elbow 10 deg, shoulder 20 deg
        self.assertEqual(self.speeds(sync_speeds(step, {1: 110, 2: 120})), {1: 30, 2: 60})

    def test_sync_never_exceeds_the_stored_speeds(self):
        step = arm_step({1: (90, 20), 2: (100, 60)})             # elbow 30 deg at 20/s = 1.5 s, shoulder 10 deg
        result = sync_speeds(step, {1: 120, 2: 90})
        self.assertEqual(self.speeds(result)[1], 20)
        self.assertLessEqual(self.speeds(result)[2], 60)
        self.assertGreaterEqual(self.speeds(result)[2], 1)

    def test_sync_leaves_unmoving_servos_and_the_input_alone(self):
        step = arm_step({0: (90, 60), 1: (100, 60), 2: (140, 60)})
        before = {"servos": [dict(s) for s in step["servos"]]}
        result = sync_speeds(step, {0: 90, 1: 110, 2: 120})
        self.assertEqual(self.speeds(result)[0], 60)             # the gripper does not move: its speed is irrelevant and kept
        self.assertEqual(step["servos"], before["servos"])       # the recorded step is not changed

    def test_sync_without_movement_changes_nothing(self):
        step = arm_step({1: (110, 60)})
        self.assertEqual(sync_speeds(step, {1: 110}), step)

    def test_arm_duration_margin_can_be_zero(self):
        step = arm_step({1: (150, 30)}, wait=0.5)
        self.assertAlmostEqual(arm_duration(step, {1: 120}, margin=0), 1.0 + 0.5)


class ValidateTest(unittest.TestCase):
    def test_drive_speed_must_be_50_or_100(self):
        for good in (50, 100):
            self.assertEqual(validate_move(move_with(drive_step(20, good)), LIMITS), [])
        self.assertEqual(validate_move(move_with({"type": "drive", "cm": 20}), LIMITS), [])     # older files: no speed, 100%
        for bad in (30, 75, 0, "fast", None):
            errors = validate_move(move_with(drive_step(20, bad)), LIMITS)
            self.assertEqual(len(errors), 1, bad)
            self.assertIn("Step 1", errors[0])
            self.assertIn("speed", errors[0])

    def test_assist_options_must_be_true_or_false(self):
        move = one_of_each()
        for key in ("assist_gyro", "assist_encoders"):
            for good in (True, False):
                move[key] = good
                self.assertEqual(validate_move(move, LIMITS), [])
            move[key] = 1
            errors = validate_move(move, LIMITS)
            self.assertEqual(len(errors), 1, key)
            self.assertIn(key, errors[0])
            del move[key]

    def test_smooth_must_be_true_or_false(self):
        move = one_of_each()
        for good in (True, False):
            move["smooth"] = good
            self.assertEqual(validate_move(move, LIMITS), [])
        move["smooth"] = "yes"
        errors = validate_move(move, LIMITS)
        self.assertEqual(len(errors), 1)
        self.assertIn("smooth", errors[0])

    def test_valid_move_has_no_errors(self):
        self.assertEqual(validate_move(one_of_each(), LIMITS), [])

    def test_errors_name_step_and_field(self):
        move = new_move("bad")
        move["steps"] = [arm_step({1: (200, 30)})]
        errors = validate_move(move, LIMITS)
        self.assertEqual(len(errors), 1)
        self.assertIn("Step 1", errors[0])
        self.assertIn("angle", errors[0])
        for step in (drive_step(0), turn_step(500), wait_step(0), {"type": "arm", "servos": [], "wait": 0}, {"type": "fly"}):
            move["steps"] = [wait_step(1), step]
            errors = validate_move(move, LIMITS)
            self.assertTrue(errors, step)
            self.assertTrue(all("Step 2" in e for e in errors), (step, errors))

    def test_missing_fields_do_not_crash(self):
        for steps in ([{"type": "arm"}], [{"type": "drive"}], [{"cm": "far", "type": "drive"}], [{}], ["junk"]):
            errors = validate_move({"steps": steps}, LIMITS)
            self.assertTrue(errors, steps)
        self.assertTrue(validate_move({}, LIMITS))
        self.assertTrue(validate_move({"steps": "nope"}, LIMITS))


class FileTest(unittest.TestCase):
    def test_save_load_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "sub" / "x.json"
            move = one_of_each()
            save_move(path, move)
            self.assertEqual(load_move(path), move)

    def test_load_rejects_garbage(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, text in (("a.json", "not json"), ("b.json", "[1,2]"), ("c.json", '{"name": "x"}')):
                path = Path(tmp) / name
                path.write_text(text)
                with self.assertRaises(MoveFileError, msg=name):
                    load_move(path)
            with self.assertRaises(MoveFileError):
                load_move(Path(tmp) / "missing.json")

    def test_example_file_is_valid(self):
        move = load_move(Path(__file__).with_name("example_grab.json"))     # a sample of the file format; your own moves live in moves/
        self.assertEqual(validate_move(move, LIMITS), [])


class FakeCar:
    """Records every command the player sends; motion (drive/turn) stays busy until finish_motion()."""

    def __init__(self):
        self.calls = []
        self.connected_flag = True
        self.angles = {1: 120, 2: 90}
        self.motion_result = True
        self.start_result = True
        self.busy = False
        self.sensors, self.heading, self.wheels, self.why = "front 20.0 cm, gripper 90 mm", 0.0, (0.0, 0.0), ""

    def readings(self):
        return Readings(self.sensors, self.heading, *self.wheels)

    def failure(self):
        return self.why

    def servo(self, index, angle, speed):
        self.calls.append(("servo", index, angle, speed))
        self.angles[index] = angle

    def servo_angles(self):
        return dict(self.angles)

    def start_drive(self, cm, speed, assist):
        self.calls.append(("start_drive", cm, speed, assist))
        self.busy = self.start_result
        return self.start_result

    def start_turn(self, deg):
        self.calls.append(("start_turn", deg))
        self.busy = self.start_result
        return self.start_result

    def motion_active(self):
        return self.busy

    def motion_ok(self):
        return self.motion_result

    def stop(self):
        self.calls.append(("stop",))
        self.busy = False
        getattr(self, "on_stop", lambda: None)()

    def connected(self):
        return self.connected_flag

    def finish_motion(self):
        self.busy = False

    def kinds(self):
        return [c[0] for c in self.calls]


def player_for(car):
    return MovePlayer(car, NAMES)


def move_of(*steps):
    move = new_move("t")
    move["steps"] = list(steps)
    return move


class MoveLogTest(unittest.TestCase):
    def played(self, *steps, at=0.0):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(*steps), at)
        return car, player

    @staticmethod
    def text(player):
        return "\n".join(player.log)

    def test_it_names_the_move_and_each_step_with_what_the_sensors_say(self):
        _, player = self.played(drive_step(20))
        for part in ("move 't': 1 steps, smooth off, drives steered by gyro and encoders", "step 1/1: Drive: forward 20 cm at 100%",
                     "before : front 20.0 cm, gripper 90 mm, heading +0.0, wheels L 0.0 R 0.0 cm"):
            self.assertIn(part, self.text(player))

    def test_a_running_drive_is_sampled_twice_a_second(self):
        car, player = self.played(drive_step(20))
        for tick in range(1, 13):                                     # 1.2 s, the right wheel never counts
            car.wheels = (tick * 1.0, 0.0)
            player.tick(tick / 10)
        samples = [line for line in player.log if "moving :" in line]
        self.assertEqual(len(samples), 2)
        self.assertIn("moving : 0.5 s, wheels L +5.0 R +0.0 cm", samples[0])

    def test_the_result_of_a_drive(self):
        car, player = self.played(drive_step(20))
        car.wheels, car.heading, car.sensors = (19.5, 20.2), 0.4, "front 4.2 cm, gripper 30 mm"
        car.finish_motion()
        player.tick(1.5)
        self.assertIn("result : took 1.5 s; asked +20 cm; wheels L +19.5 R +20.2 cm; heading moved left 0.4 deg; "
                      "front 4.2 cm, gripper 30 mm", self.text(player))
        self.assertIn("done after 1.5 s", self.text(player))

    def test_the_result_of_a_turn(self):
        car, player = self.played(turn_step(90))
        car.heading, car.wheels = 88.5, (-8.0, 8.5)
        car.finish_motion()
        player.tick(2.0)
        self.assertIn("result : took 2.0 s; turned left 88.5 deg (asked left 90); wheels L -8.0 R +8.5 cm", self.text(player))

    def test_a_failed_drive_says_why_and_where_things_stood(self):
        car, player = self.played(drive_step(5, 50), wait_step(1))
        car.motion_result, car.why = False, "the car gave up"
        car.wheels = (3.0, 0.0)
        car.finish_motion()
        player.tick(1.5)
        reason = "the drive did not finish normally: the car gave up"
        self.assertEqual(player.status, f"Stopped: {reason}")
        text = self.text(player)
        for part in ("result : took 1.5 s; asked +5 cm; wheels L +3.0 R +0.0 cm", "at the stop: front 20.0 cm",
                     f"stopped after 1.5 s: {reason}"):
            self.assertIn(part, text)
        self.assertEqual(text.count("result :"), 1)
        self.assertNotIn("step 2/2", text)

    def test_stopping_it_by_hand_logs_the_motion_so_far(self):
        car, player = self.played(drive_step(20))
        player.tick(0.2)
        car.wheels = (4.0, 4.0)
        player.abort("stopped by you")
        self.assertIn("wheels L +4.0 R +4.0 cm", self.text(player))
        self.assertIn("stopped after 0.2 s: stopped by you", self.text(player))

    def test_arm_steps_are_logged_too(self):
        _, player = self.played(arm_step({1: (150, 30)}, wait=0), wait_step(1))
        player.tick(5.0)
        self.assertIn("step 1/2: Arm: Elbow 150", self.text(player))
        self.assertIn("step 2/2: Wait: 1 s", self.text(player))
        self.assertEqual(self.text(player).count("before :"), 2)

    def test_each_run_starts_a_new_log_and_a_tail_hands_out_only_new_lines(self):
        car, player = self.played(wait_step(1))
        tail = LogTail()
        self.assertEqual(tail.new(player.log), player.log)
        self.assertEqual(tail.new(player.log), [])
        player.tick(1.1)
        self.assertIn("done after", "\n".join(tail.new(player.log)))
        player.start(move_of(wait_step(1)), 2.0)
        self.assertEqual(tail.new(player.log), player.log)
        self.assertNotIn("done after", self.text(player))


class RepeatTest(unittest.TestCase):
    """Playing a move many times over and counting how many runs failed."""

    def setUp(self):
        self.car = FakeCar()
        self.player = RepeatPlayer(self.car, NAMES)
        self.now = 0.0

    def start(self, times, *steps):
        return self.player.start(move_of(*(steps or (drive_step(5, 50),))), self.now, times)

    def advance(self, seconds):
        for _ in range(int(round(seconds * 10))):
            self.now = round(self.now + 0.1, 1)
            self.player.tick(self.now)

    def finish_drive(self, ok=True, why=""):
        self.car.motion_result, self.car.why = ok, why
        self.car.finish_motion()
        self.advance(0.1)

    def drives(self):
        return self.car.kinds().count("start_drive")

    def text(self):
        return "\n".join(self.player.log)

    def test_it_plays_the_move_the_asked_number_of_times_with_a_pause_between(self):
        self.assertTrue(self.start(3))
        for run in (1, 2, 3):
            self.assertEqual(self.drives(), run)
            self.finish_drive()
            if run < 3:
                self.assertTrue(self.player.active)
                self.advance(1.5)
                self.assertEqual(self.drives(), run)                  # still pausing
                self.advance(0.7)
        self.assertFalse(self.player.active)
        self.assertIs(self.player.ok, True)
        self.assertEqual(self.player.status, "3 runs: 3 succeeded, 0 failed")
        self.assertIn("=== run 2 of 3 ===", self.text())
        self.assertIn("=== 3 runs: 3 succeeded, 0 failed ===", self.text())

    def test_a_failed_run_is_counted_and_the_next_one_still_happens(self):
        self.start(4, drive_step(5, 50), wait_step(0.1))
        for ok in (True, False, False, True):
            self.finish_drive(ok, "" if ok else "the car gave up")
            self.advance(2.5)
        self.assertFalse(self.player.active)
        self.assertIs(self.player.ok, False)
        self.assertEqual(self.player.status, "4 runs: 2 succeeded, 2 failed")
        self.assertIn("  failed 2 x at step 1/2 (Drive: forward 5 cm at 50%): the drive did not finish normally: the car gave up "
                      "(runs 2, 3)", self.text())

    def test_the_car_stopping_itself_after_a_failure_does_not_end_the_repeats(self):
        # on the dashboard, the stop a failing run sends looks like "you touched the controls" and calls abort again
        self.car.on_stop = lambda: self.player.abort("manual control")
        self.start(2)
        self.finish_drive(False, "the car gave up")
        self.assertTrue(self.player.active, self.player.status)
        self.advance(2.5)
        self.assertEqual(self.drives(), 2)

    def test_a_first_step_that_cannot_start_fails_that_run_only(self):
        self.car.on_stop = lambda: self.player.abort("manual control")
        self.car.start_result = False
        self.assertTrue(self.start(2))
        self.advance(2.5)
        self.assertFalse(self.player.active)
        self.assertEqual(self.player.status, "2 runs: 0 succeeded, 2 failed")

    def test_stop_ends_all_the_runs_also_during_the_pause(self):
        for during_pause in (False, True):
            self.setUp()
            self.start(5)
            self.finish_drive()
            self.advance(0.5 if during_pause else 2.2)
            self.player.abort("stopped by you")
            self.assertFalse(self.player.active)
            self.assertIn("stop", self.car.kinds())
            before = self.drives()
            self.advance(5.0)
            self.assertEqual(self.drives(), before)
            self.assertIn("stopped by you", self.player.status)
            self.assertIn("1 succeeded, 0 failed", self.player.status)
            self.assertIn("of 5 runs", self.text())

    def test_a_disconnected_car_ends_the_runs(self):
        self.start(5)
        self.car.connected_flag = False
        self.advance(0.2)
        self.assertFalse(self.player.active)
        self.assertIn("car disconnected", self.player.status)

    def test_one_run_behaves_as_before(self):
        self.start(1)
        self.finish_drive()
        self.assertEqual((self.player.active, self.player.ok, self.player.status), (False, True, "Done"))
        self.assertNotIn("===", self.text())
        self.setUp()
        self.start(1)
        self.finish_drive(False, "the car gave up")
        self.assertEqual(self.player.status, "Stopped: the drive did not finish normally: the car gave up")

    def test_while_it_runs_the_status_shows_the_count(self):
        self.start(10)
        self.assertEqual(self.player.status, "Run 1/10 (0 ok, 0 failed) - Step 1/1: Drive: forward 5 cm at 50%")
        self.finish_drive()
        self.assertEqual(self.player.status, "Run 1/10 done (1 ok, 0 failed) - next run in 2 s")

    def test_refused_when_there_is_nothing_to_play_or_no_car(self):
        self.assertFalse(self.player.start(move_of(), 0.0, 3))
        self.assertEqual(self.player.status, "Nothing to play")
        self.car.connected_flag = False
        self.assertFalse(self.start(3))
        self.assertEqual(self.player.status, "Car not connected")

    def test_refused_while_already_running(self):
        self.start(3)
        self.assertFalse(self.start(3))
        self.assertEqual(self.drives(), 1)


class PlayerTest(unittest.TestCase):
    def test_plays_steps_in_order(self):
        car = FakeCar()
        player = player_for(car)
        self.assertTrue(player.start(move_of(arm_step({1: (150, 30)}, wait=0), drive_step(20), turn_step(90), wait_step(1)), 0.0))
        self.assertEqual(car.kinds(), ["servo"])
        player.tick(1.0)
        self.assertEqual(car.kinds(), ["servo"])            # the elbow is still on its way (1.3 s)
        player.tick(1.31)
        self.assertEqual(car.kinds(), ["servo", "start_drive"])
        player.tick(1.5)
        car.finish_motion()
        player.tick(1.6)
        self.assertEqual(car.kinds()[-1], "start_turn")
        car.finish_motion()
        player.tick(1.7)                                     # the wait step begins, 1 s long
        player.tick(2.69)
        self.assertTrue(player.active)
        player.tick(2.71)
        self.assertFalse(player.active)
        self.assertTrue(player.ok)
        self.assertEqual(player.status, "Done")
        self.assertLess(car.kinds().index("servo"), car.kinds().index("start_drive"))
        self.assertLess(car.kinds().index("start_drive"), car.kinds().index("start_turn"))

    def test_arm_waits_for_travel(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(arm_step({1: (150, 30)}, wait=0), drive_step(20)), 10.0)
        player.tick(11.0)
        self.assertNotIn("start_drive", car.kinds())
        player.tick(11.0 + ARM_MARGIN_S + 0.01)
        self.assertIn("start_drive", car.kinds())

    def test_drive_passes_its_own_speed_and_defaults_to_100(self):
        for step, expected in ((drive_step(20, 50), 50), (drive_step(20, 100), 100), ({"type": "drive", "cm": 20}, 100)):
            car = FakeCar()
            player = player_for(car)
            player.start(move_of(step), 0.0)
            self.assertIn(("start_drive", 20, expected, 3), car.calls)
            car.finish_motion()
            player.tick(0.1)
            self.assertTrue(player.ok)

    def test_drive_straight_assist_follows_the_moves_ticks(self):
        # bit 1 = gyro heading, bit 2 = encoders (the car's G command uses the same numbers); both on unless the move says otherwise
        for settings, expected in (({}, 3), ({"assist_gyro": False}, 2), ({"assist_encoders": False}, 1),
                                   ({"assist_gyro": False, "assist_encoders": False}, 0), ({"assist_gyro": True}, 3)):
            car = FakeCar()
            player = player_for(car)
            move = move_of(drive_step(20, 100))
            move.update(settings)
            player.start(move, 0.0)
            self.assertIn(("start_drive", 20, 100, expected), car.calls, settings)

    def test_abort_stops_the_car(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(drive_step(20), arm_step({1: (150, 30)})), 0.0)
        player.abort("test")
        self.assertEqual(car.kinds().count("stop"), 1)
        self.assertFalse(player.active)
        self.assertFalse(player.ok)
        self.assertIn("test", player.status)
        before = list(car.calls)
        player.tick(5.0)
        self.assertEqual(car.calls, before)

    def test_abort_twice_is_harmless(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(wait_step(5)), 0.0)
        player.abort("one")
        before = list(car.calls)
        player.abort("two")
        self.assertEqual(car.calls, before)
        self.assertIn("one", player.status)

    def test_disconnect_aborts(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(wait_step(5)), 0.0)
        car.connected_flag = False
        player.tick(1.0)
        self.assertFalse(player.active)
        self.assertIn("disconnected", player.status)
        self.assertIn("stop", car.kinds())

    def test_failed_motion_aborts_and_skips_arm_steps(self):
        car = FakeCar()
        car.motion_result = False
        player = player_for(car)
        player.start(move_of(drive_step(20), arm_step({1: (150, 30)})), 0.0)
        car.finish_motion()
        player.tick(0.5)
        self.assertFalse(player.active)
        self.assertFalse(player.ok)
        self.assertNotIn("servo", car.kinds())

    def test_refuses_empty_and_double_start(self):
        car = FakeCar()
        player = player_for(car)
        self.assertFalse(player.start({"steps": []}, 0.0))
        self.assertEqual(player.status, "Nothing to play")
        self.assertEqual(car.calls, [])
        self.assertTrue(player.start(move_of(wait_step(5)), 0.0))
        before = list(car.calls)
        self.assertFalse(player.start(move_of(wait_step(5)), 0.1))
        self.assertEqual(car.calls, before)
        self.assertTrue(player.active)

    def test_smooth_skips_the_margin_between_arm_steps(self):
        car = FakeCar()
        player = player_for(car)
        move = move_of(arm_step({1: (150, 30)}, wait=0), arm_step({1: (120, 30)}, wait=0))
        move["smooth"] = True
        player.start(move, 10.0)
        player.tick(10.5)
        self.assertEqual(car.kinds().count("servo"), 1)
        player.tick(11.01)                                   # the first step's travel (1.0 s) is over: no 0.3 s margin
        self.assertEqual(car.kinds().count("servo"), 2)

    def test_not_smooth_keeps_the_margin_between_arm_steps(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(arm_step({1: (150, 30)}, wait=0), arm_step({1: (120, 30)}, wait=0)), 10.0)
        player.tick(11.01)
        self.assertEqual(car.kinds().count("servo"), 1)
        player.tick(11.0 + ARM_MARGIN_S + 0.01)
        self.assertEqual(car.kinds().count("servo"), 2)

    def test_smooth_keeps_the_margin_before_a_drive(self):
        car = FakeCar()
        player = player_for(car)
        move = move_of(arm_step({1: (150, 30)}, wait=0), drive_step(20))
        move["smooth"] = True
        player.start(move, 10.0)
        player.tick(11.01)
        self.assertNotIn("start_drive", car.kinds())
        player.tick(11.0 + ARM_MARGIN_S + 0.01)
        self.assertIn("start_drive", car.kinds())

    def test_smooth_sends_synchronised_speeds(self):
        car = FakeCar()                                      # the arm starts at elbow 120, shoulder 90
        player = player_for(car)
        move = move_of(arm_step({1: (110, 60), 2: (120, 60)}, wait=0))   # elbow 10 deg, shoulder 30 deg
        move["smooth"] = True
        player.start(move, 0.0)
        self.assertIn(("servo", 1, 110, 20), car.calls)
        self.assertIn(("servo", 2, 120, 60), car.calls)

    def test_not_smooth_sends_the_stored_speeds(self):
        car = FakeCar()
        player = player_for(car)
        player.start(move_of(arm_step({1: (110, 60), 2: (120, 60)}, wait=0)), 0.0)
        self.assertIn(("servo", 1, 110, 60), car.calls)
        self.assertIn(("servo", 2, 120, 60), car.calls)

    def test_editing_the_move_while_playing_does_not_change_the_playback(self):
        car = FakeCar()
        player = player_for(car)
        move = move_of(wait_step(1), arm_step({1: (150, 30)}, wait=0))
        player.start(move, 0.0)
        move["steps"].insert(0, drive_step(50))              # the window's Up/Down/Delete/Add edit this same list
        player.tick(1.01)
        self.assertEqual(car.kinds(), ["servo"])             # the second step of the move as it was when playing began
        self.assertNotIn("start_drive", car.kinds())

    def test_refuses_when_not_connected(self):
        car = FakeCar()
        car.connected_flag = False
        player = player_for(car)
        self.assertFalse(player.start(move_of(wait_step(1)), 0.0))
        self.assertEqual(player.status, "Car not connected")
        self.assertEqual(car.calls, [])

    def test_drive_start_refused_aborts(self):
        car = FakeCar()
        car.start_result = False
        player = player_for(car)
        self.assertFalse(player.start(move_of(drive_step(20)), 0.0))
        self.assertFalse(player.active)
        self.assertFalse(player.ok)
        self.assertIn("could not start", player.status)


if __name__ == "__main__":
    unittest.main()
