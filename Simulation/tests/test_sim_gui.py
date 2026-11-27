import sys
import tkinter as tk
import unittest
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import gui  # noqa: E402
from sim.car import Noise  # noqa: E402
from sim.run import run_episode  # noqa: E402

ROOT = None


def setUpModule():
    global ROOT
    try:
        ROOT = tk.Tk()
        ROOT.withdraw()
    except tk.TclError as e:
        raise unittest.SkipTest(f"no display for Tk: {e}")


def tearDownModule():
    if ROOT is not None:
        ROOT.destroy()


class NoiseFromSlidersTest(unittest.TestCase):
    def test_the_slider_defaults_are_the_default_noise(self):
        built = gui.build_noise({s.key: s.default for s in gui.SLIDERS}, perfect=False)
        for name, value in asdict(Noise()).items():
            self.assertAlmostEqual(asdict(built)[name], value, places=6, msg=name)

    def test_perfect_ignores_the_sliders(self):
        built = gui.build_noise({s.key: s.hi for s in gui.SLIDERS}, perfect=True)
        self.assertEqual(built, Noise.perfect())

    def test_a_slider_moves_its_error(self):
        values = {s.key: s.default for s in gui.SLIDERS}
        values["thin"] = 100                                   # the ultrasonic misses every chair leg
        self.assertEqual(gui.build_noise(values, perfect=False).thin_hit, 0.0)
        values["gyro"] = 0.2
        self.assertEqual(gui.build_noise(values, perfect=False).gyro_bias_dps, 0.2)


class WindowTest(unittest.TestCase):
    def setUp(self):
        self.win = tk.Toplevel(ROOT)
        self.app = gui.SimWindow(self.win)
        self.addCleanup(self.win.destroy)

    def wait(self, seconds=30):
        import time
        end = time.monotonic() + seconds
        while self.app.busy and time.monotonic() < end:
            self.win.update()
            time.sleep(0.02)
        self.win.update()

    def test_it_opens_ready_to_run(self):
        self.win.update()
        self.assertFalse(self.app.busy)
        self.assertEqual(str(self.app.run_button["state"]), "normal")

    def test_a_run_shows_the_summary_and_the_picture(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(2)
        self.app.start_run(batch=False)
        self.assertTrue(self.app.busy)
        self.assertEqual(str(self.app.run_button["state"]), "disabled")      # no second run on top of the first
        self.wait()
        self.assertFalse(self.app.busy)
        self.assertIn("Collected", self.app.summary())
        self.assertIsNotNone(self.app.shown_image)
        self.assertEqual(str(self.app.run_button["state"]), "normal")

    def test_a_batch_lists_every_run_and_the_total(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.batch_var.set(3)
        self.app.start_run(batch=True)
        self.wait(60)
        text = self.app.summary()
        self.assertIn("3 layouts", text)
        self.assertEqual(len(self.app.run_choices()), 3)                  # one entry per layout to look at

    def test_the_picture_can_be_saved(self):
        import tempfile
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.start_run(batch=False)
        self.wait()
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "run.png"
            self.app.save_picture(str(out))
            self.assertGreater(out.stat().st_size, 1000)

    def run_once(self, minutes=6):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(minutes)
        self.app.start_run(batch=False)
        self.wait()
        self.app.pause()                                       # tests drive the playback by hand

    def test_a_finished_run_starts_playing_from_the_beginning(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(2)
        self.app.start_run(batch=False)
        self.wait()
        self.assertTrue(self.app.playing)
        self.assertLess(self.app.t, 60.0)                       # started from the beginning (it has only just begun to play)
        self.app.pause()

    def test_seeking_shows_that_moment(self):
        self.run_once()
        before = self.app.shown_image
        self.app.seek(60.0)
        self.assertAlmostEqual(self.app.t, 60.0)
        self.assertIn("1:00", self.app.time_var.get())
        self.assertIsNot(self.app.shown_image, before)

    def test_playing_moves_the_time_forward(self):
        import time
        self.run_once()
        self.app.seek(0.0)
        self.app.speed_var.set("60x")
        self.app.play()
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self.win.update()
            time.sleep(0.02)
        self.app.pause()
        self.assertGreater(self.app.t, 5.0)

    def test_playing_at_the_end_stops_and_replays_from_the_start(self):
        self.run_once()
        self.app.seek(self.app.results[0].seconds)
        self.app.play()
        self.assertLess(self.app.t, 5.0)                       # pressing Play on a finished run starts it over
        self.app.pause()

    def test_the_whole_run_view_can_be_switched_on(self):
        self.run_once()
        self.app.overview_var.set(True)
        self.app.refresh()
        self.assertTrue(self.app.overview)
        self.assertIn("whole run", self.app.time_var.get().lower())

    def test_the_timeline_lists_what_happened_with_clickable_times(self):
        self.run_once()
        text = self.app.summary()
        self.assertIn("Picking up ball 1", text)
        self.assertGreaterEqual(len(self.app.event_times), 2)

    def test_an_event_can_be_jumped_to(self):
        self.run_once()
        t = self.app.event_times[0]
        self.app.jump_to_event(0)
        self.assertAlmostEqual(self.app.t, max(0.0, t - 3.0))   # a few seconds before it, so the build-up is visible

    def test_another_layout_of_a_batch_can_be_looked_at(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.batch_var.set(2)
        self.app.start_run(batch=True)
        self.wait(60)
        self.app.pause()
        self.app.choose_run(1)
        self.assertIs(self.app.current, self.app.results[1])

    def test_the_legend_explains_the_colours(self):
        names = " ".join(self.app.legend_names())
        for word in ("Car", "Ball", "Bin", "Chair leg"):
            self.assertIn(word, names)

    def test_playback_slows_down_while_the_arm_moves(self):
        self.run_once()
        self.app.speed_var.set("200x")
        g = [g for g in self.app.results[0].car.grips if g.kind == "pick"][0]
        self.app.slow_arm_var.set(True)
        self.app.seek(g.start - 5.0)
        self.assertEqual(self.app.effective_speed(), 200.0)                 # driving: as fast as asked
        self.app.seek((g.start + g.end) / 2)
        self.assertLessEqual(self.app.effective_speed(), 3.0)               # the arm is moving: slow enough to watch
        self.app.seek(g.end + 5.0)
        self.assertEqual(self.app.effective_speed(), 200.0)                 # and fast again afterwards

    def test_the_slow_down_can_be_switched_off(self):
        self.run_once()
        self.app.speed_var.set("60x")
        g = [g for g in self.app.results[0].car.grips if g.kind == "pick"][0]
        self.app.slow_arm_var.set(False)
        self.app.seek((g.start + g.end) / 2)
        self.assertEqual(self.app.effective_speed(), 60.0)

    def test_it_is_on_by_default(self):
        self.assertTrue(self.app.slow_arm_var.get())

    def test_the_event_for_an_arm_move_jumps_to_just_before_it_starts(self):
        self.run_once()
        i = next(n for n, (t, line) in enumerate(self.app.events) if line.startswith("Picking up"))
        self.app.jump_to_event(i)
        self.assertAlmostEqual(self.app.t, max(0.0, self.app.event_times[i] - 3.0))
        self.assertIsNone(__import__("sim.timeline", fromlist=["x"]).state_at(self.app.current, self.app.t).arm)       # before the arm moves

    def test_the_camera_view_can_be_shown_and_is_on_by_default(self):
        self.assertTrue(self.app.camera_var.get())
        self.run_once()
        from unittest import mock
        seen = []
        real = gui.R.render_frame
        with mock.patch.object(gui.R, "render_frame", lambda *a, **k: seen.append(k.get("show_camera")) or real(*a, **k)):
            self.app.refresh()
            self.app.camera_var.set(False)
            self.app.refresh()
        self.assertEqual(seen, [True, False])

    def test_the_legend_names_both_beams(self):
        names = " ".join(self.app.legend_names())
        self.assertIn("Camera view", names)
        self.assertIn("Distance sensor", names)

    def test_there_is_a_search_strategy_dropdown_with_both_searches(self):
        self.assertEqual(gui.STRATEGY_LABELS["map"], self.app.strategy_var.get())          # the map-guided search is the default
        self.assertEqual(list(gui.STRATEGY_LABELS), ["map", "lanes"])
        self.assertTrue(all(isinstance(v, str) and v for v in gui.STRATEGY_LABELS.values()))
        self.assertEqual(list(self.app.strategy_combo["values"]), list(gui.STRATEGY_LABELS.values()))

    def test_choosing_a_search_explains_it(self):
        self.app.strategy_var.set(gui.STRATEGY_LABELS["lanes"])
        self.app.strategy_changed()
        self.assertIn("lanes", self.app.strategy_hint.cget("text").lower())

    def test_the_chosen_search_is_the_one_that_runs(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(3)
        self.app.strategy_var.set(gui.STRATEGY_LABELS["lanes"])
        self.app.start_run(batch=False)
        self.wait()
        self.app.pause()
        self.assertEqual(self.app.current.strategy, "lanes")
        self.assertIn("lawnmower", self.app.summary().lower())

    def test_a_batch_uses_the_chosen_search_too(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.batch_var.set(2)
        self.app.strategy_var.set(gui.STRATEGY_LABELS["lanes"])
        self.app.start_run(batch=True)
        self.wait(60)
        self.app.pause()
        self.assertTrue(all(r.strategy == "lanes" for r in self.app.results))

    def test_the_default_search_is_used_when_nothing_is_changed(self):
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.start_run(batch=False)
        self.wait()
        self.app.pause()
        self.assertEqual(self.app.current.strategy, "map")

    def test_the_lane_route_can_be_shown_and_is_on_by_default(self):
        self.assertTrue(self.app.route_var.get())
        self.assertIn("route", " ".join(self.app.legend_names()).lower())

    def test_what_has_been_looked_at_can_be_shown_and_is_on_by_default(self):
        self.assertTrue(self.app.seen_var.get())
        self.assertIn("looked at", " ".join(self.app.legend_names()).lower())
        self.run_once()
        from unittest import mock
        seen = []
        real = gui.R.render_frame
        with mock.patch.object(gui.R, "render_frame", lambda *a, **k: seen.append(k.get("show_seen")) or real(*a, **k)):
            self.app.refresh()
            self.app.seen_var.set(False)
            self.app.refresh()
        self.assertEqual(seen, [True, False])

    def test_the_cars_own_map_shows_all_three_colours(self):
        names = " ".join(self.app.legend_names()).lower()
        for word in ("looked at", "obstacle", "come back"):
            self.assertIn(word, names)
        self.run_once()
        from unittest import mock
        seen = []
        real = gui.R.render_frame
        with mock.patch.object(gui.R, "render_frame", lambda *a, **k: seen.append((k.get("show_seen"), k.get("show_blocked"), k.get("show_pending"))) or real(*a, **k)):
            self.app.refresh()
            self.app.seen_var.set(False)
            self.app.refresh()
        self.assertEqual(seen, [(True, True, True), (False, False, False)])                 # one tick box for the car's whole map

    def test_extra_check_rounds_default_to_none_and_reach_the_run(self):
        self.assertEqual(self.app.rechecks_var.get(), 1)                       # one extra round is the default
        self.app.arena_var.set("empty")
        self.app.perfect_var.set(True)
        self.app.minutes_var.set(1)
        self.app.rechecks_var.set(2)
        self.app.start_run(batch=False)
        self.wait()
        self.app.pause()
        self.assertEqual(self.app.current.brain.rechecks, 2)

    def test_a_failure_is_shown_not_swallowed(self):
        self.app.finish_error("boom")
        self.assertIn("boom", self.app.summary())
        self.assertFalse(self.app.busy)


if __name__ == "__main__":
    unittest.main()
