import sys
import tempfile
import time
import types
import unittest
from unittest import mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402


class CtrlCTest(unittest.TestCase):
    def pump(self, interrupted):
        import queue
        calls = []
        app = types.SimpleNamespace(interrupted=interrupted, closing=False, ui_queue=queue.Queue(),
                                    _refresh_camera=lambda: calls.append("camera"), _refresh_badges=lambda: None,
                                    _path_tick=lambda: None, _tof_tick=lambda now: None, _record_tick=lambda: None, _close=lambda: calls.append("close"), _pump=lambda: None,
                                    root=types.SimpleNamespace(after=lambda ms, fn: calls.append("again")))
        dashboard.Dashboard._pump(app)
        return calls

    def test_ctrl_c_closes_the_dashboard_the_same_way_as_the_window_button(self):
        self.assertEqual(self.pump(True), ["close"])               # closes, does not draw another picture, does not run again

    def test_without_ctrl_c_the_pump_runs_as_usual(self):
        self.assertEqual(self.pump(False), ["camera", "again"])


class DistanceStampTest(unittest.TestCase):
    def make_app(self):
        class Bar:
            def delete(self, *a): pass
            def winfo_width(self): return 100
            def create_rectangle(self, *a, **k): pass

        class Var:
            def set(self, v): self.v = v

        class Label:
            def configure(self, **k): pass

        return types.SimpleNamespace(dist_bar=Bar(), dist_var=Var(), dist_note=Var(), dist_label=Label(), dist_cm=None, dist_stamp=None)

    def test_a_reading_is_stamped_with_the_time(self):
        app = self.make_app()
        before = time.monotonic()
        dashboard.Dashboard._show_distance(app, 25.0)
        self.assertEqual(app.dist_cm, 25.0)
        self.assertTrue(before <= app.dist_stamp <= time.monotonic())

    def test_no_echo_is_still_a_new_reading(self):
        app = self.make_app()
        dashboard.Dashboard._show_distance(app, 25.0)
        first = app.dist_stamp
        time.sleep(0.05)
        dashboard.Dashboard._show_distance(app, -1)
        self.assertIsNone(app.dist_cm)
        self.assertGreater(app.dist_stamp, first)


class GripperSensorTest(unittest.TestCase):
    """The gripper's VL53L0X: millimetres as text, "-1" = nothing in range, "NA" = the car cannot find the sensor.
    It is one line of text in the Distance card's title row (a block of its own made the window 91 px taller and cut the Drive card)."""

    def make_app(self):
        class Var:
            v = None
            def set(self, v): self.v = v

        class Label:
            fg = None
            def configure(self, **k): self.fg = k.get("fg", self.fg)

        posted = []
        return types.SimpleNamespace(tof_var=Var(), tof_label=Label(), tof_mm=None, tof_stamp=None, posted=posted, _post=lambda fn: fn(),
                                     _show_tof=lambda mm: posted.append(("tof", mm)), _show_tof_missing=lambda: posted.append(("missing",)))

    def test_a_reading_is_shown_and_stamped(self):
        app = self.make_app()
        before = time.monotonic()
        dashboard.Dashboard._show_tof(app, 123)
        self.assertEqual((app.tof_mm, app.tof_var.v), (123, "Gripper 123 mm"))
        self.assertTrue(before <= app.tof_stamp <= time.monotonic())

    def test_close_means_something_is_in_the_jaws(self):
        app = self.make_app()
        dashboard.Dashboard._show_tof(app, dashboard.TOF_NEAR_MM - 1)
        self.assertIn("in jaws", app.tof_var.v)
        self.assertEqual(app.tof_label.fg, dashboard.GOOD)
        dashboard.Dashboard._show_tof(app, dashboard.TOF_NEAR_MM + 1)
        self.assertNotIn("in jaws", app.tof_var.v)
        self.assertNotEqual(app.tof_label.fg, dashboard.GOOD)

    def test_nothing_in_range_clears_the_value(self):
        app = self.make_app()
        dashboard.Dashboard._show_tof(app, 80)
        dashboard.Dashboard._show_tof(app, -1)
        self.assertIsNone(app.tof_mm)
        self.assertEqual(app.tof_var.v, "Gripper: out of range")

    def test_sensor_not_found_says_so_and_drops_the_old_reading(self):
        app = self.make_app()
        dashboard.Dashboard._show_tof(app, 80)
        dashboard.Dashboard._show_tof_missing(app)
        self.assertIsNone(app.tof_mm)
        self.assertEqual(app.tof_var.v, "Gripper: not found")

    def test_the_notification_text_is_understood(self):
        for data, expected in ((b"42", ("tof", 42)), (b"-1", ("tof", -1)), (b"NA", ("missing",))):
            app = self.make_app()
            dashboard.Dashboard._on_tof(app, None, bytearray(data))
            self.assertEqual(app.posted, [expected])

    def test_garbage_is_ignored(self):
        app = self.make_app()
        dashboard.Dashboard._on_tof(app, None, bytearray(b"x?"))
        self.assertEqual(app.posted, [])

    def test_a_value_that_stops_updating_is_not_left_on_screen_as_if_live(self):
        app = self.make_app()
        dashboard.Dashboard._show_tof(app, 29)
        now = app.tof_stamp
        dashboard.Dashboard._tof_tick(app, now + dashboard.TOF_STALE_S - 0.1)      # still fresh: untouched
        self.assertIn("29", app.tof_var.v)
        dashboard.Dashboard._tof_tick(app, now + dashboard.TOF_STALE_S + 0.1)      # stale: shown as no data
        self.assertEqual(app.tof_var.v, "Gripper: no readings")
        self.assertIsNone(app.tof_mm)

    def test_no_reading_yet_is_not_called_stale(self):
        app = self.make_app()
        app.tof_var.set("Gripper: waiting")
        dashboard.Dashboard._tof_tick(app, 1e9)
        self.assertEqual(app.tof_var.v, "Gripper: waiting")

    def test_the_sensor_is_subscribed_to_with_the_car_s_uuid(self):
        self.assertEqual(dashboard.TOF_CHAR_UUID, "55667788-99AA-BBCC-DDEE-FF0011223344")

    def test_a_channel_the_car_does_not_have_is_reported_not_swallowed(self):
        # "Gripper: waiting" for ever: the subscription failed (old firmware, or Windows' saved copy of the car's services) and the
        # error was thrown away
        import asyncio

        class Client:
            def __init__(self): self.started = []
            async def start_notify(self, uuid, handler):
                if uuid == "bad":
                    raise RuntimeError("Characteristic bad was not found!")
                self.started.append(uuid)

        client = Client()
        failed = asyncio.run(dashboard.subscribe_all(client, (("a", None), ("bad", None), ("b", None))))
        self.assertEqual(client.started, ["a", "b"])                 # one bad channel does not stop the others
        self.assertEqual(list(failed), ["bad"])
        self.assertIn("not found", failed["bad"])

    def test_the_car_is_never_connected_with_windows_saved_services(self):
        # Windows kept the car's old service list (from before the gripper channel existed) and could not be made to forget it
        seen = []
        with mock.patch.object(dashboard, "BleakClient", lambda target, **kw: seen.append((target, kw))):
            dashboard.make_client("AA:BB", 8)
        self.assertEqual(seen, [("AA:BB", {"timeout": 8, "winrt": {"use_cached_services": False}})])

    def test_a_missing_gripper_channel_is_shown_on_the_card(self):
        app = self.make_app()
        dashboard.Dashboard._show_tof_unavailable(app)
        self.assertIn("firmware", app.tof_var.v)
        self.assertNotEqual(app.tof_var.v, dashboard.TOF_WAITING)


class RecordingTest(unittest.TestCase):
    """The Record button: the camera thread hands every frame to a recorder that saves the camera's JPEGs into a folder."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        patch = mock.patch.object(dashboard, "RECORD_DIR", Path(self.tmp.name))
        patch.start()
        self.addCleanup(patch.stop)

    def make_app(self, rate="every frame", rotate="cw"):
        class Var:
            def __init__(self, v=""): self.v = v
            def get(self): return self.v
            def set(self, v): self.v = v

        class Button:
            text = ""
            def configure(self, **k): self.text = k.get("text", self.text)

        camera = dashboard.CameraWorker("host", 1)
        return types.SimpleNamespace(camera=camera, record_rate_var=Var(rate), rotate_var=Var(rotate), record_button=Button(),
                                     detect_status_var=Var())

    def test_the_camera_thread_hands_each_frame_to_the_recorder(self):
        app = self.make_app()
        dashboard.Dashboard.start_recording(app)
        app.camera._publish(b"jpeg-1", "rgb-1")
        app.camera._publish(b"jpeg-2", "rgb-2")
        self.assertEqual(app.camera.last_jpeg, b"jpeg-2")                       # the picture shown is unchanged by recording
        self.assertEqual(app.camera.frame_id, 2)
        self.assertEqual(sorted(p.read_bytes() for p in app.camera.recorder.folder.glob("*.jpg")), [b"jpeg-1", b"jpeg-2"])

    def test_without_a_recording_frames_are_only_shown(self):
        camera = dashboard.CameraWorker("host", 1)
        camera._publish(b"jpeg", "rgb")
        self.assertEqual((camera.frame_id, camera.recorder), (1, None))

    def test_start_uses_the_menu_rate_and_the_rotation_setting(self):
        app = self.make_app(rate="1 per s", rotate="ccw")
        dashboard.Dashboard.start_recording(app)
        rec = app.camera.recorder
        self.assertEqual((rec.min_gap, rec.rotate), (1.0, "ccw"))
        self.assertTrue(str(rec.folder).startswith(self.tmp.name))
        self.assertIn("Stop", app.record_button.text)

    def test_stop_ends_the_recording_and_the_button_offers_to_record_again(self):
        app = self.make_app()
        dashboard.Dashboard.start_recording(app)
        rec = app.camera.recorder
        dashboard.Dashboard.stop_recording(app)
        self.assertIsNone(app.camera.recorder)
        self.assertFalse(rec.active)
        self.assertIn("Record", app.record_button.text)
        self.assertIn("recording.json", [p.name for p in rec.folder.iterdir()])

    def test_the_button_toggles(self):
        app = self.make_app()
        app.toggle = lambda: dashboard.Dashboard.toggle_recording(app)
        app.start_recording = lambda: dashboard.Dashboard.start_recording(app)
        app.stop_recording = lambda: dashboard.Dashboard.stop_recording(app)
        app.toggle()
        self.assertIsNotNone(app.camera.recorder)
        app.toggle()
        self.assertIsNone(app.camera.recorder)

    def test_the_tick_shows_the_number_of_frames_saved(self):
        app = self.make_app()
        dashboard.Dashboard.start_recording(app)
        app.camera._publish(b"a", "x")
        app.camera._publish(b"b", "x")
        dashboard.Dashboard._record_tick(app)
        self.assertIn("2", app.record_button.text)

    def test_a_disk_error_stops_the_recording_and_is_shown(self):
        app = self.make_app()
        app.stop_recording = lambda: dashboard.Dashboard.stop_recording(app)
        dashboard.Dashboard.start_recording(app)
        with mock.patch.object(Path, "write_bytes", side_effect=OSError("disk full")):
            app.camera._publish(b"a", "x")
        dashboard.Dashboard._record_tick(app)
        self.assertIsNone(app.camera.recorder)
        self.assertIn("disk full", app.detect_status_var.v)


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


class MoveReadingsTest(unittest.TestCase):
    """What the car tells the move log: its sensors, and why a drive or turn did not finish."""

    def app(self, **changes):
        values = dict(dist_cm=12.5, dist_stamp=1.0, tof_mm=38, tof_stamp=1.0, wheel_cm=(10.0, 11.0),
                      tracker=types.SimpleNamespace(heading=-3.5), rail_watch=None,
                      runner=types.SimpleNamespace(status="Turn +9: turn timed out"), drive_fail_reason="")
        values.update(changes)
        return types.SimpleNamespace(**values)

    def test_readings(self):
        readings = dashboard.DashboardCar(self.app()).readings()
        self.assertEqual(readings.sensors, "front 12.5 cm, gripper 38 mm")
        self.assertEqual((readings.heading, readings.left_cm, readings.right_cm), (-3.5, 10.0, 11.0))

    def test_readings_without_any_sensor(self):
        app = self.app(dist_stamp=None, tof_stamp=None, wheel_cm=None)
        readings = dashboard.DashboardCar(app).readings()
        self.assertEqual(readings.sensors, "front: no reading, gripper no readings")
        self.assertIsNone(readings.left_cm)

    def test_the_servo_supply_shows_the_worst_since_the_last_look(self):
        watch = dashboard.watch_rails(None, 5.80, 5.90, 30.0, 40.0, 0, 0)
        watch = dashboard.watch_rails(watch, 5.78, 5.50, 20.0, 420.0, 0, 2)
        watch = dashboard.watch_rails(watch, 5.79, 5.88, 10.0, 15.0, 0, 0)
        app = self.app(rail_watch=watch)
        car = dashboard.DashboardCar(app)
        self.assertIn("supply arm 5.79 V dip 30 mV, gripper 5.88 V dip 420 mV STOPPED by the guard", car.readings().sensors)
        self.assertNotIn("supply", car.readings().sensors)                # told once: the next line starts afresh

    def test_a_supply_without_a_voltage_wire(self):
        app = self.app(rail_watch=dashboard.watch_rails(None, -1.0, 5.0, 0.0, 300.0, 0, 1))
        self.assertIn("supply arm no sensor, gripper 5.00 V dip 300 mV slowed by the guard",
                      dashboard.DashboardCar(app).readings().sensors)

    def test_why_a_motion_failed(self):
        app = self.app(drive_fail_reason="the car gave up", drive_encoder=lambda cm, pct, assist: True,
                       turn_by=lambda deg: True, _runner_sending=False)
        car = dashboard.DashboardCar(app)
        car.start_drive(5, 50, 3)
        self.assertEqual(car.failure(), "the car gave up")
        car.start_turn(9)
        self.assertEqual(car.failure(), "Turn +9: turn timed out")


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
        self.assertIn("the car gave up", app.drive_fail_reason)

    def test_a_car_that_never_reports_finishing_is_explained(self):
        app = self.app()
        dashboard.Dashboard.drive_encoder(app, 30, 100)
        self.note(app, 1)
        app.drive_deadline = 0.0
        self.assertFalse(dashboard.Dashboard.drive_busy(app))
        self.assertTrue(app.drive_failed)
        self.assertIn("did not report the drive finished in time (last state 1, 1 readings", app.drive_fail_reason)

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
            runner=types.SimpleNamespace(active=False), _runner_sending=sending, collect_window=None, _yaw_resync=False, _write=write,
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

    def collect_app(self, sending, active=True):
        app, _ = self.fake_app(sending=sending)
        stops = []
        app.collect_window = types.SimpleNamespace(approach=types.SimpleNamespace(active=active), stop=lambda: stops.append(1))
        return app, stops

    def test_manual_control_stops_collect(self):
        for message in ("S", "U"):
            app, stops = self.collect_app(sending=False)
            self.send(app, message)
            self.assertEqual(stops, [1], msg=message)

    def test_collect_is_not_stopped_by_other_commands_the_runner_or_when_idle(self):
        app, stops = self.collect_app(sending=False)
        self.send(app, "P100")
        self.assertEqual(stops, [])
        app, stops = self.collect_app(sending=True)                  # the car's own turn commands come through the runner
        self.send(app, "S")
        self.assertEqual(stops, [])
        app, stops = self.collect_app(sending=False, active=False)
        self.send(app, "S")
        self.assertEqual(stops, [])

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
