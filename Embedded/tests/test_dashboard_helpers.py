import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402


class ServoCommandsTest(unittest.TestCase):
    def test_servo_commands_plain(self):
        self.assertEqual(dashboard.servo_commands(1, 150, 30), ["V1 150 30"])

    def test_servo_commands_first_command_sets_held_angle(self):
        self.assertEqual(dashboard.servo_commands(1, 150, 30, 120), ["H1 120", "V1 150 30"])


class FakeScale:
    """A Tk Scale: set() calls the command only when the value changes."""

    def __init__(self, value, command=None):
        self.value, self.command = value, command

    def get(self):
        return self.value

    def set(self, value):
        if value != self.value:
            self.value = value
            if self.command:
                self.command(value)


class SendServoCommandTest(unittest.TestCase):
    def fake_app(self, commanded=()):
        D = dashboard.Dashboard
        app = types.SimpleNamespace(_applying_servo=False, servo_commanded=set(commanded), last_angles={"1": 120}, sent=[],
                                    connected=types.SimpleNamespace(is_set=lambda: True), _schedule_servo_memory_save=lambda: None)
        app._send = app.sent.append
        app.send_servo_command = lambda i, a, s: D.send_servo_command(app, i, a, s)
        app._send_servo = lambda i: D._send_servo(app, i)
        app._send_servo_throttled = app._send_servo            # no throttling in the test
        var = types.SimpleNamespace(set=lambda text: None)
        app.servo_scales = {1: FakeScale(120)}
        app.speed_scales = {1: FakeScale(60, lambda v: D._servo_speed_changed(app, 1, v, var))}
        return app

    def test_first_command_sets_held_angle_then_moves_and_sliders_follow(self):
        app = self.fake_app()
        dashboard.Dashboard.send_servo_command(app, 1, 150, 30)
        self.assertEqual(app.sent, ["H1 120", "V1 150 30"])
        self.assertEqual((app.servo_scales[1].value, app.speed_scales[1].value), (150, 30))
        self.assertEqual(app.last_angles["1"], 150)

    def test_later_command_is_sent_once_even_though_the_speed_slider_changes(self):
        app = self.fake_app(commanded={1})
        dashboard.Dashboard.send_servo_command(app, 1, 90, 45)
        self.assertEqual(app.sent, ["V1 90 45"])


class DashboardCarTest(unittest.TestCase):
    def test_turn_does_not_count_as_manual_control(self):
        seen = []
        app = types.SimpleNamespace(_runner_sending=False)
        app.turn_by = lambda deg: seen.append(("turn", app._runner_sending)) or True
        car = dashboard.DashboardCar(app)
        self.assertTrue(car.start_turn(90))
        self.assertEqual(seen, [("turn", True)])                      # turn_by sends S itself
        self.assertFalse(app._runner_sending)

    def test_drive_goes_to_the_cars_encoder_drive_with_its_speed(self):
        seen = []
        app = types.SimpleNamespace(drive_encoder=lambda cm, pct, assist: seen.append((cm, pct, assist)) or True)
        self.assertTrue(dashboard.DashboardCar(app).start_drive(20, 50, 2))
        self.assertEqual(seen, [(20, 50, 2)])

    def test_activity_and_result_follow_the_kind_of_motion(self):
        app = types.SimpleNamespace(drive_encoder=lambda cm, pct, assist: True, turn_by=lambda deg: True, _runner_sending=False,
                                    runner=types.SimpleNamespace(active=False), runner_ok=False,
                                    drive_failed=False, drive_busy=lambda: True)
        car = dashboard.DashboardCar(app)
        car.start_drive(20, 100, 3)
        self.assertTrue(car.motion_active())                          # the drive, not the (idle) path runner
        self.assertTrue(car.motion_ok())                              # not failed, whatever the runner last did
        app.drive_failed = True
        self.assertFalse(car.motion_ok())
        car.start_turn(90)
        self.assertFalse(car.motion_active())                         # now it is the path runner that counts
        self.assertFalse(car.motion_ok())                             # runner_ok is False


class EncoderDriveTest(unittest.TestCase):
    """Dashboard side of the car's M<cm> <speed> drive: the car counts its own encoders and reports a drive state
    (0 idle, 1 driving, 2 stopping, 3 failed) as a 7th number of every encoder reading."""

    def app(self, connected=True, runner_active=False):
        sent = []
        return types.SimpleNamespace(
            sent=sent, _send=sent.append, connected=types.SimpleNamespace(is_set=lambda: connected),
            runner=types.SimpleNamespace(active=runner_active, abort=lambda reason: None), encoder_seq=5, drive_state=0,
            drive_failed=True, drive_wait_seq=None, drive_deadline=0.0)

    def note(self, app, state):
        dashboard.Dashboard._note_drive_state(app, state)

    def test_drive_sends_the_command_and_waits_for_the_car(self):
        app = self.app()
        self.assertTrue(dashboard.Dashboard.drive_encoder(app, 20, 100, 3))
        self.assertEqual(app.sent, ["M20 100 3"])
        self.assertFalse(app.drive_failed)
        self.assertTrue(dashboard.Dashboard.drive_busy(app))
        dashboard.Dashboard.drive_encoder(app, -10, 50, 1)                # gyro only
        self.assertEqual(app.sent[-1], "M-10 50 1")

    def test_drive_refused_when_not_connected_or_runner_busy(self):
        for app in (self.app(connected=False), self.app(runner_active=True)):
            self.assertFalse(dashboard.Dashboard.drive_encoder(app, 20, 100))
            self.assertEqual(app.sent, [])

    def test_a_short_drive_that_is_already_over_is_not_missed(self):
        app = self.app()
        dashboard.Dashboard.drive_encoder(app, 5, 100)
        self.note(app, 0)                                             # the first reading after the command: not enough yet
        self.assertTrue(dashboard.Dashboard.drive_busy(app))
        self.note(app, 0)                                             # the second, still idle: it really is over
        self.assertFalse(dashboard.Dashboard.drive_busy(app))
        self.assertFalse(app.drive_failed)

    def test_busy_while_the_car_reports_driving_or_stopping(self):
        app = self.app()
        dashboard.Dashboard.drive_encoder(app, 30, 100)
        for state in (1, 1, 2):
            self.note(app, state)
            self.assertTrue(dashboard.Dashboard.drive_busy(app))
        self.note(app, 0)
        self.assertFalse(dashboard.Dashboard.drive_busy(app))

    def test_a_failed_drive_is_reported(self):
        app = self.app()
        dashboard.Dashboard.drive_encoder(app, 30, 100)
        self.note(app, 3)
        self.note(app, 0)
        self.note(app, 0)
        self.assertTrue(app.drive_failed)
        self.assertFalse(dashboard.Dashboard.drive_busy(app))

    def test_abort_ends_the_wait(self):
        app = self.app()
        app._runner_sending = False
        app.drive_wait_seq = 5
        dashboard.Dashboard.path_abort(app)
        self.assertIsNone(app.drive_wait_seq)
        self.assertEqual(app.sent, ["S"])

    def test_encoder_reading_with_the_drive_state(self):
        app = types.SimpleNamespace(posted=[])
        app._post = app.posted.append
        dashboard.Dashboard._on_encoder(app, None, b"10,12,3.0,4.0,5.0,6.0,1")
        self.assertEqual(len(app.posted), 1)
        shown = []
        app._note_drive_state = lambda state: shown.append(("state", state))
        app._show_encoder = lambda *args: shown.append(("show", args[:4]))
        app.posted[0]()
        self.assertEqual(shown, [("state", 1), ("show", (3.0, 4.0, 5.0, 6.0))])

    def test_encoder_reading_without_the_drive_state_still_works(self):
        app = types.SimpleNamespace(posted=[])
        app._post = app.posted.append
        dashboard.Dashboard._on_encoder(app, None, b"10,12,3.0,4.0,5.0,6.0")        # an older car: six numbers
        shown = []
        app._note_drive_state = lambda state: shown.append(state)
        app._show_encoder = lambda *args: None
        app.posted[0]()
        self.assertEqual(shown, [0])


class ManualControlHookTest(unittest.TestCase):
    def fake_app(self, sending):
        aborted = []

        async def write(message):
            pass

        app = types.SimpleNamespace(
            connected=types.SimpleNamespace(is_set=lambda: True), loop=types.SimpleNamespace(is_running=lambda: True),
            runner=types.SimpleNamespace(active=False), _runner_sending=sending, _yaw_resync=False, _write=write,
            tracker=types.SimpleNamespace(set_command=lambda m: None),
            moves_window=types.SimpleNamespace(player=types.SimpleNamespace(active=True, abort=aborted.append)))
        return app, aborted

    def send(self, app, message):
        original = dashboard.asyncio.run_coroutine_threadsafe
        dashboard.asyncio.run_coroutine_threadsafe = lambda coro, loop: coro.close()
        try:
            dashboard.Dashboard._send(app, message)
        finally:
            dashboard.asyncio.run_coroutine_threadsafe = original

    def test_manual_stop_aborts_a_playing_move(self):
        app, aborted = self.fake_app(sending=False)
        self.send(app, "S")
        self.assertEqual(aborted, ["manual control"])

    def test_arrow_key_aborts_a_playing_move(self):
        app, aborted = self.fake_app(sending=False)
        self.send(app, "U")
        self.assertEqual(aborted, ["manual control"])

    def test_commands_that_are_not_driving_do_not_abort(self):
        app, aborted = self.fake_app(sending=False)
        self.send(app, "P100")
        self.assertEqual(aborted, [])

    def test_the_runners_own_commands_do_not_abort_the_move(self):
        app, aborted = self.fake_app(sending=True)
        self.send(app, "S")
        self.assertEqual(aborted, [])


class RemainingAheadTest(unittest.TestCase):
    def test_remaining_ahead(self):
        self.assertAlmostEqual(dashboard.remaining_along_heading(0, 0, 0, 0, 10), 10.0)
        self.assertAlmostEqual(dashboard.remaining_along_heading(0, 0, 90, -10, 0), 10.0)   # facing left: forward is -x
        self.assertAlmostEqual(dashboard.remaining_along_heading(0, 0, 0, 0, -2), -2.0)     # already passed


class RunnerMoveArrivalTest(unittest.TestCase):
    def runner_with(self, step, y, cmd):
        stub = types.SimpleNamespace(x=0.0, y=y, cmd=cmd, cm_per_s_100=30.0, corners=[], heading_now=lambda now: 0.0)
        sent = []
        runner = dashboard.PathRunner(stub, sent.append, lambda: 100, lambda: None)
        runner.log_file = Path(tempfile.gettempdir()) / "moves_test_runner_log.txt"
        runner.active, runner.steps, runner.idx = True, [step], 0
        runner.state, runner.until, runner.move_from, runner.coast_cm = "moving", 1e9, (0.0, 0.0), 10.0
        return runner, stub, sent

    def test_runner_stops_backward_move_at_target(self):
        runner, stub, sent = self.runner_with(["move", -20.0, 0.0, -20.0], -5.0, "D")
        runner.tick(1.0, 0.0)
        self.assertEqual(sent, [])                      # still 15 cm from the target: keep reversing
        stub.y = -15.0                                  # within the coast distance of the target
        runner.tick(2.0, 0.0)
        self.assertEqual(sent, ["S"])

    def test_runner_still_stops_forward_move_at_target(self):
        runner, stub, sent = self.runner_with(["move", 20.0, 0.0, 20.0], 5.0, "U")
        runner.tick(1.0, 0.0)
        self.assertEqual(sent, [])
        stub.y = 15.0
        runner.tick(2.0, 0.0)
        self.assertEqual(sent, ["S"])


if __name__ == "__main__":
    unittest.main()
