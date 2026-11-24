# Collect: drive to the ball and pick it up (Stage 2b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A "Collect" window in the dashboard: with a paper ball in view the car turns to face it, drives to it in short hops using the ultrasonic sensor, the camera and an optional camera distance model, stops at the taught grab spot, then plays the chosen move.

**Architecture:** Three pure modules, unit-tested with fakes and with time passed in (like `helpers/moves.py`): `ball_distance.py` (camera distance model, distance lines), `grab_spot.py` (the taught spot), `ball_approach.py` (steady-ball tracker and the approach state machine). The dashboard gets a `CollectWindow`, the distance-lines checkbox, and thin adapters over what it already has (the detector worker, the ultrasonic reading, `DashboardCar`, `MovePlayer`).

**Tech Stack:** Python 3.12 (`C:\Users\HP\AppData\Local\Programs\Python\Python312\python.exe`), numpy/OpenCV (already used), tkinter, stdlib `unittest`. No new packages.

**Spec:** `docs/superpowers/specs/2026-11-23-paper-ball-pickup-design.md` (Stage 2b)

## Global Constraints

- Drives go through `car.start_drive(cm, speed, 3)` (the car's encoder drive; assist 3 = gyro + encoders): speed 50 under 15 cm and 100 otherwise; a hop is at least 3 cm and at most 25 cm. Turns go through `car.start_turn(deg)` (+ = left). `car` is the `moves.PlayerCar` interface.
- Field of view 55 degrees (constant `FOV_DEG`). Pick range = the taught ultrasonic reading plus and minus 2 cm.
- Time is passed in (`tick(now)`, `start(now)`), never read inside the logic.
- Anything that stops Collect stops the car and the move and says why (spec list). STOP, the dashboard STOP and arrow keys go through the existing manual-control hook in `Dashboard._send`.
- The camera card must stay within 520 px wide (`tests/test_camera_layout.py`).
- The new modules live in `Dashboard/helpers/` and are imported as `from helpers.<name> import ...` (the folder is a package; `dashboard.py` stays in `Dashboard/`). Files the program writes go through `helpers.paths.data_file(name)` into `Dashboard/data/` (not kept in git); recorded moves stay in `Dashboard/moves/`.
- Run Python and tests with the 3.12 interpreter above. Commit only when the owner asks, only the files a task names, and with no attribution lines.

## Review Focus

1. A sensor reading in the pick range while the camera cannot confirm a ball (no ball seen) or disagrees by more than 8 cm / 25%: the move never starts. (Task 4)
2. No new camera frame, no new distance reading, or no ball for too long, in any state: Collect stops with the reason and the car stops; it never keeps driving on old data. (Task 4)
3. A hop never drives past the pick distance (never below 3 cm or above 25 cm); too close backs up instead; something closer than the ball stops it. (Task 4)
4. STOP, an arrow key, closing the window or the dashboard during a turn, a hop, the settling pause or the pick move: the car and the move stop and Collect ends. (Task 6)
5. No calibration file works (ultrasonic only); a broken calibration or grab-spot file gives a message, not a crash; Collect refuses to start without a grab spot. (Tasks 1, 2, 4, 6)

## File Structure

- Create `Dashboard/helpers/ball_distance.py`, `Dashboard/helpers/grab_spot.py`, `Dashboard/helpers/ball_approach.py` (pure; no tkinter).
- Modify `Dashboard/helpers/ball_detector.py` (`DetectorWorker.latest_frame()`), `Dashboard/dashboard.py` (camera card checkbox, `dist_stamp`, `CollectWindow`, adapters, hooks).
- Create tests `Dashboard/tests/test_ball_distance.py`, `test_grab_spot.py`, `test_ball_approach.py`, `test_collect_window.py`; extend `test_ball_detector.py`, `test_camera_layout.py`, `test_dashboard_helpers.py`.
- Saved by the app (not committed unless the owner wants): `Dashboard/data/ball_calibration.json`, `Dashboard/data/grab_spot.json`.

Run all tests from the repo root:
`& "C:\Users\HP\AppData\Local\Programs\Python\Python312\python.exe" -m unittest discover -s Dashboard/tests -p "test_*.py" -v`

---

### Task 1: Camera distance model, bearing, drawing (`ball_distance.py`)

**Files:** Create `Dashboard/helpers/ball_distance.py`; Test `Dashboard/tests/test_ball_distance.py`

**Interfaces — Produces:**
- `FOV_DEG = 55.0`, `CALIB_FILE = data_file("ball_calibration.json")`, `GUIDE_DISTANCES_CM = (20, 30, 40, 60, 80)`
- `class CalibrationError(ValueError)`
- `class DistanceModel`: `__init__(self, a: float, b: float, points: list | None = None)` (1/distance = a·y + b, y = the bottom row of the ball's box in picture pixels); `@classmethod fit(cls, samples: list[tuple[float, float]]) -> DistanceModel` (samples are `(distance_cm, bottom_y)`; at least 3, and `a` must come out > 0, else `CalibrationError`); `distance_cm(self, bottom_y: float) -> float | None` (None when 1/distance is not positive); `y_for(self, distance_cm: float) -> float`; `errors(self) -> list[tuple[float, float, float]]` ((real, estimated, estimated − real) per calibration point); `to_dict(self) -> dict`; `@classmethod from_dict(cls, d: dict) -> DistanceModel` (raises `CalibrationError` for a missing/odd field)
- `save_model(path: Path, model: DistanceModel) -> None`; `load_model(path: Path) -> DistanceModel | None` (None if the file does not exist; `CalibrationError` if it cannot be read)
- `ball_angle_deg(x_ratio: float, fov_deg: float = FOV_DEG) -> float` (x_ratio = box centre x / picture width; + = left of the centre line, the same sign as the car's turns)
- `draw_guides(rgb, model: DistanceModel | None, distances=GUIDE_DISTANCES_CM)` -> a copy with a horizontal line and an "N cm" label at each distance's row (red under 30 cm, yellow under 50, green beyond; skipped when the row is outside the picture); the same array when `model` is None
- `draw_spot_marker(rgb, x_ratio: float)` -> a copy with a dashed vertical line at that x and the label "grab direction"

- [ ] **Step 1: Write the failing tests** (`ModelTest`, `AngleTest`, `DrawTest`). The line used is 1/d = 0.0005·y − 0.1, so `y_for(20) = 300`, `y_for(40) = 250`, `y_for(80) = 225`:
  - `test_fit_recovers_the_line`: `DistanceModel.fit([(20, 300), (40, 250), (80, 225)])` gives `a` ≈ 0.0005 and `b` ≈ −0.1 (`places=6`); `distance_cm(300)` ≈ 20; `y_for(40)` ≈ 250; every error in `errors()` ≈ 0
  - `test_a_noisy_fit_reports_each_points_error`: samples `[(20, 300), (40, 252), (80, 225)]` give 3 error tuples, each `(real, estimated, estimated - real)`, and at least one with `abs(error) > 0.1` (the fit cannot pass through all three)
  - `test_fit_needs_three_samples_and_a_rising_curve`: two samples raise `CalibrationError` mentioning "3"; `[(20, 200), (40, 250), (80, 300)]` raises `CalibrationError`
  - `test_no_distance_above_the_horizon`: `distance_cm(150)` is None for the line above (1/d would be negative)
  - `test_save_and_load_roundtrip`; `test_load_missing_file_gives_none`; `test_load_broken_file_raises` (text `not json`, and `{"a": 1}`)
  - `test_angle`: `ball_angle_deg(0.5) == 0`; `ball_angle_deg(0.0)` ≈ 27.5 and `ball_angle_deg(1.0)` ≈ −27.5 (`places=3`); the angle grows as x_ratio shrinks
  - `test_guides_are_drawn_at_the_calibrated_rows`: a 640×480 black picture; with the model above the row 250 (40 cm) has non-black pixels, the row 300 (20 cm) too, and the row 225 (80 cm) too; the input array is unchanged; a model whose line for 80 cm falls below the picture is skipped without error
  - `test_no_model_returns_the_same_picture`; `test_spot_marker_marks_the_column` (non-black pixels in the column near `0.25 * width`)

- [ ] **Step 2: Run** `... -m unittest discover -s Dashboard/tests -p "test_ball_distance.py" -v`. Expected: ImportError.

- [ ] **Step 3: Implement** the interface. Fit by `numpy.polyfit(y, 1/d, 1)`, as `camera_distance.py` does. `y_for` inverts the line.

- [ ] **Step 4: Run.** Expected: all pass.

- [ ] **Step 5: Commit (only when the owner asks):** `git add Dashboard/helpers/ball_distance.py Dashboard/tests/test_ball_distance.py`; `feat(distance): the camera's distance model for the ball, the bearing and the distance lines`.

---

### Task 2: The grab spot, and a time-stamped detector result

**Files:** Create `Dashboard/helpers/grab_spot.py`; Modify `Dashboard/helpers/ball_detector.py`; Test `Dashboard/tests/test_grab_spot.py`, `Dashboard/tests/test_ball_detector.py`

**Interfaces — Produces:**
- `grab_spot.py`: `SPOT_FILE = data_file("grab_spot.json")`, `RANGE_MARGIN_CM = 2.0`, `class GrabSpotError(ValueError)`, frozen dataclass `GrabSpot(x_ratio: float, ultrasonic_cm: float, camera_cm: float | None = None)` with properties `pick_min` (= `ultrasonic_cm - 2`) and `pick_max` (= `+ 2`); `save_spot(path, spot) -> None`; `load_spot(path) -> GrabSpot | None` (None if missing; `GrabSpotError` if unreadable or invalid: `0 < x_ratio < 1`, `5 <= ultrasonic_cm <= 60`)
- `ball_detector.py`: `DetectionResult = namedtuple("DetectionResult", "frame_id detections ms stamp width height")` and `DetectorWorker.latest_frame(self) -> DetectionResult | None` (stamp = `time.monotonic()` when the model finished; width/height of the frame it ran on). `latest()` and its 3-tuple stay as they are.

- [ ] **Step 1: Write the failing tests:**
  - `test_grab_spot.py`: `GrabSpot(0.5, 16.0)` has `pick_min == 14.0` and `pick_max == 18.0`; save/load roundtrip (with and without `camera_cm`); `load_spot` of a missing file is None; garbage text, `{"x_ratio": 1.5, "ultrasonic_cm": 16}` and `{"x_ratio": 0.5, "ultrasonic_cm": 2}` each raise `GrabSpotError`
  - `test_ball_detector.py::WorkerTest.test_latest_frame_carries_the_frame_size_and_a_time`: fake camera frame `np.zeros((10, 20, 3))` → `latest_frame()` has `width == 20`, `height == 10`, `frame_id`, `ms >= 0`, and `before <= stamp <= after` (`time.monotonic()` taken around `step()`); `latest_frame()` is None before any frame

- [ ] **Step 2: Run both files.** Expected: fail (missing module / attribute).
- [ ] **Step 3: Implement** both.
- [ ] **Step 4: Run all tests.** Expected: pass.
- [ ] **Step 5: Commit (only when the owner asks):** `git add Dashboard/helpers/grab_spot.py Dashboard/helpers/ball_detector.py Dashboard/tests/test_grab_spot.py Dashboard/tests/test_ball_detector.py`; `feat(collect): the taught grab spot, and detector results with a time and the frame size`.

---

### Task 3: The steady-ball tracker (`ball_approach.py`, part 1)

**Files:** Create `Dashboard/helpers/ball_approach.py`; Test `Dashboard/tests/test_ball_approach.py` (class `TrackerTest`)

**Interfaces — Consumes:** `ball_detector.Detection`, `DetectionResult`. **Produces:** frozen dataclass `Ball(x_ratio: float, bottom_y: float, stamp: float)`; `class BallTracker(need_frames: int = 3, max_jump_px: float = 15.0)` with `update(result: DetectionResult) -> None`, `ball(self) -> Ball | None`, `reset(self) -> None`.

Behaviour: from each result the nearest ball (largest bottom edge `y2`) is taken; `x_ratio` = its centre x / `result.width`, `bottom_y` = `y2`, `stamp` = `result.stamp`. A ball is *steady* after `need_frames` consecutive results whose nearest ball's centre moved at most `max_jump_px` pixels from the previous result's; a result with no ball, or a jump, restarts the count. A result whose `frame_id` was already seen is ignored. `ball()` returns the newest steady ball, else None.

- [ ] **Step 1: Write the failing tests** (a helper `result(frame_id, boxes, stamp=...)` builds a `DetectionResult` with width 640, height 480 from `(x1, y1, x2, y2)` boxes):
  - `test_steady_after_three_frames`: boxes at the same place in 3 results → `ball()` has `x_ratio == 0.5` for centre 320, `bottom_y` = `y2`, `stamp` of the third; after 2 results it is None
  - `test_a_jump_restarts_the_count`: 2 results, then one 40 px away, then 2 more near it → None until 3 near each other
  - `test_an_empty_result_restarts_the_count`
  - `test_takes_the_lowest_ball`: two boxes, one with larger `y2` → that one
  - `test_the_same_frame_counts_once`: the same `frame_id` three times → None
  - `test_reset`
- [ ] **Step 2: Run.** Expected: ImportError.
- [ ] **Step 3: Implement** `Ball` and `BallTracker`.
- [ ] **Step 4: Run.** Expected: pass.
- [ ] **Step 5: Commit (only when the owner asks):** `git add Dashboard/helpers/ball_approach.py Dashboard/tests/test_ball_approach.py`; `feat(collect): a tracker that finds the nearest steady ball`.

---

### Task 4: The approach state machine (`ball_approach.py`, part 2)

**Files:** Modify `Dashboard/helpers/ball_approach.py`; Test `Dashboard/tests/test_ball_approach.py` (class `ApproachTest`)

**Interfaces — Consumes:** `Ball` (Task 3), `GrabSpot` (Task 2), `ball_angle_deg` (Task 1), `moves.MovePlayer` (its `start(move, now)`, `tick(now)`, `.active`, `.ok`, `.status`), `moves.PlayerCar`.
**Produces:**
- `class Reading(NamedTuple)`: `cm: float | None` (None = no echo), `stamp: float`
- `class World(Protocol)`: `ball() -> Ball | None`; `frame_stamp() -> float | None` (time of the newest camera frame); `reading() -> Reading | None`; `camera_cm(ball: Ball) -> float | None` (None when not calibrated)
- `class Approach`: `__init__(self, world, car, player, move: dict, spot: GrabSpot)`; `start(self, now: float) -> bool` (False if already active or `car.connected()` is False); `tick(self, now: float) -> None`; `abort(self, reason: str) -> None` (no-op when not active; sets `active` False first, then `car.stop()` and `player.abort(...)` if the player is active); attributes `active: bool`, `ok: bool | None`, `status: str`, `log: list[str]` (one line per decision)
- Class constants: `TURN_TOL_DEG = 3.0`, `SETTLE_S = 0.6`, `FRAME_AFTER_S = 0.3`, `READING_AFTER_S = 0.5`, `NO_BALL_S = 3.0`, `FRAME_STALE_S = 1.5`, `READING_STALE_S = 2.0`, `MAX_DECISIONS = 25`, `MAX_TIME_S = 90.0`, `FAR_CM = 60.0`, `HOP_MIN_CM = 3.0`, `HOP_MAX_CM = 25.0`, `BLIND_HOP_CM = 10.0`, `SLOW_BELOW_CM = 15.0`, `AGREE_CM = 8.0`, `AGREE_FRACTION = 0.25`, `CAMERA_MARGIN_CM = 5.0`

Behaviour. States: waiting-for-data/deciding, moving (a turn or hop), settling (`SETTLE_S` after a motion ends), picking. `tick`: abort on `car.connected()` False, on `MAX_TIME_S`, on more than `MAX_DECISIONS` decisions ("too many steps"). While moving: wait for `car.motion_active()` False, abort if `car.motion_ok()` is False, else settle; the end time of the motion is `ready_since`. A decision needs fresh data: the ball's `stamp >= ready_since + FRAME_AFTER_S` and the reading's `stamp >= ready_since + READING_AFTER_S` (before the first motion `ready_since` is minus infinity); otherwise wait. Abort if `now - world.frame_stamp() > FRAME_STALE_S` ("no new camera frame"), if `now - reading.stamp > READING_STALE_S` ("no distance readings"), or if no ball has been seen for `NO_BALL_S` ("lost the ball"). Decision: let `error = ball_angle_deg(ball.x_ratio) - ball_angle_deg(spot.x_ratio)`; if `abs(error) > TURN_TOL_DEG`, `car.start_turn(error)` (abort "could not start the turn" on False). Otherwise, with `r = reading.cm`, `c = world.camera_cm(ball)`, `pick = spot.ultrasonic_cm`, `agree = c is None or r is None or abs(r - c) <= max(AGREE_CM, AGREE_FRACTION * c)`, and `hop(d) = clamp(d, HOP_MIN_CM, HOP_MAX_CM)`, speed `50` if the hop is under `SLOW_BELOW_CM` else `100`:
- `spot.pick_min <= r <= spot.pick_max`: abort ("the sensor says r cm but the camera says c cm") if not `agree`; else `player.start(move, now)` and go to picking
- `r < spot.pick_min`: abort ("something is closer than the ball") if `c` is not None and `r < c - max(AGREE_CM, AGREE_FRACTION * c)`; else drive `-max(HOP_MIN_CM, pick - r)` at 50
- `pick_max < r <= FAR_CM`: if `agree` (or no `c`) drive `hop(r - pick)`; elif `r < c` abort ("something is closer than the ball"); else drive `hop(c - pick - CAMERA_MARGIN_CM)`
- `r` is None or `r > FAR_CM`: drive `hop(c - pick - CAMERA_MARGIN_CM)` if `c` is not None, else `BLIND_HOP_CM`
Every drive is `car.start_drive(cm, speed, 3)`, aborting with "could not start the drive" on False; each decision appends a line to `log` (bearing error, `r`, `c`, action). Picking: `player.tick(now)`; when the player is no longer active, `ok = True`, status "Done: picked" if `player.ok`, else abort with the player's status. `abort` sets `ok = False` and `status = "Stopped: <reason>"`.

- [ ] **Step 1: Write the failing tests.** Fakes: `FakeWorld` (settable `ball_value`, `frame`, `reading_value`, `camera` function), `FakeCar` imported from `test_moves.py` (its `start_drive(cm, speed, assist)` records `("start_drive", cm, speed, assist)`; `start_turn`; `finish_motion()`), `FakePlayer` (`start` records and sets `active`; `tick`; `finish(ok)`), spot `GrabSpot(0.5, 16.0)`, a ball `Ball(0.5, 300, stamp)`:
  - `test_turns_toward_a_ball_off_the_taught_direction`: ball `x_ratio` 0.2 → `start_turn` called with ≈ +17.35 (`places=1`); 0.8 → ≈ −17.35; no drive yet
  - `test_no_turn_within_three_degrees`: ball 0.52 → no turn, a drive or a decision follows
  - `test_hops_from_the_sensor` (centred ball, no calibration): reading 40 → `("start_drive", 24.0, 100, 3)`; 25 → `(9.0, 50)`; 19 → `(3.0, 50)`; 59 → `(25.0, 100)`
  - `test_far_or_no_echo_without_calibration_hops_ten`: reading None → `(10.0, 50)`; reading 100 → `(10.0, 50)`
  - `test_far_with_calibration_hops_by_the_camera`: reading None, camera 70 → `(25.0, 100)`; camera 30 → `(9.0, 50)`
  - `test_in_range_with_the_ball_in_view_starts_the_move`: reading 16.2 → `player.start` called once with the move; `player.finish(True)` then a tick → `ok is True`, status contains "picked"
  - `test_the_move_failing_stops_it`: `player.finish(False)` → `ok is False`
  - `test_in_range_but_the_camera_disagrees_does_not_pick`: reading 16, camera 45 → aborted, status contains "camera", `player.start` never called, no drive
  - `test_in_range_without_a_ball_never_picks`: ball None for more than 3 s → aborted "lost the ball" and `player.start` never called (also with a reading of 16 throughout)
  - `test_something_closer_than_the_ball_stops`: reading 30, camera 60 → aborted, status contains "closer"; reading 10, camera 40 → aborted
  - `test_beam_beyond_the_ball_uses_the_camera`: reading 50, camera 30 → `(9.0, 50)`
  - `test_too_close_backs_up`: reading 10 → `("start_drive", -6.0, 50, 3)`; 13.6 → `-3.0`
  - `test_never_drives_past_the_pick_distance`: for readings 19.0 to 60.0 step 0.5 (no calibration) the hop satisfies `hop <= max(3.0, r - 16.0)` and `3.0 <= hop <= 25.0`
  - `test_waits_for_fresh_data_after_a_motion`: after a hop ends at t = 5.0, a reading stamped 5.2 does not start another action at t = 6.0 (no new call); with a reading stamped 5.6 and a ball stamped 5.4 it does
  - `test_stops_on_stale_or_missing_data`: `frame_stamp()` older than 1.5 s → aborted "camera"; reading older than 2 s → aborted "distance"; the car's `stop` was called once each time
  - `test_limits`: 26 decisions → aborted "too many steps"; `tick(now = start + 91)` → aborted
  - `test_the_car_refusing_or_failing`: `start_drive` False → aborted "could not start the drive"; `start_turn` False → "could not start the turn"; `motion_result = False` after a hop → aborted
  - `test_abort_stops_everything_and_is_harmless_twice`: abort during picking → `car.stop` once and the player aborted; a second abort adds no calls
  - `test_start_refused_when_already_active_or_disconnected`
- [ ] **Step 2: Run.** Expected: fail (`Approach` missing).
- [ ] **Step 3: Implement** `Reading`, `World`, `Approach` as above (one decision per `tick` at most; keep the decision a small method returning the action so it reads like the table).
- [ ] **Step 4: Run.** Expected: all pass.
- [ ] **Step 5: Commit (only when the owner asks):** `git add Dashboard/helpers/ball_approach.py Dashboard/tests/test_ball_approach.py`; `feat(collect): the approach that turns, hops and picks, using the sensor and the camera`.

---

### Task 5: Distance lines in the camera picture

**Files:** Modify `Dashboard/dashboard.py` (`__init__`, `_build_camera_card`, `_refresh_camera`, `_show_distance`, the disconnect reset at the existing `self.dist_cm = None` in the disconnect path); Test `Dashboard/tests/test_camera_layout.py`, `Dashboard/tests/test_dashboard_helpers.py`

**Interfaces — Consumes:** `ball_distance.load_model`, `draw_guides`, `draw_spot_marker`, `CALIB_FILE`, `CalibrationError` (Task 1), `grab_spot.GrabSpot` (Task 2). **Produces:** `Dashboard.distance_model: DistanceModel | None` (loaded at start; a broken file gives None and a status message), `Dashboard.grab_spot: GrabSpot | None` (loaded at start the same way), `Dashboard.lines_var: tk.BooleanVar` (checkbox "Distance lines", off by default, in the camera card's first detector row), `Dashboard.dist_stamp: float | None` (`time.monotonic()` of the newest ultrasonic notification, set in `_show_distance`, None after a disconnect), and in `_refresh_camera` the live picture goes through `draw_guides` when `lines_var` is on and a model exists, and through `draw_spot_marker` when the Collect window is open and a grab spot exists.

- [ ] **Step 1: Write the failing tests:**
  - `test_camera_layout.py`: extend `build_card`'s stand-in with `lines_var` and re-run the three layout tests; they must still pass with the new checkbox (the widest row stays ≤ 520 px, a longer status does not widen the card, a short window cuts only the picture)
  - `test_dashboard_helpers.py::DistanceStampTest`: `Dashboard._show_distance` stand-in call is awkward (it draws on a canvas), so test through a stub app with a minimal `dist_bar` canvas stub (`delete`, `winfo_width` returning 100, `create_rectangle`) and `dist_var`/`dist_note` fakes: after `_show_distance(app, 25.0)` `app.dist_cm == 25.0` and `app.dist_stamp` is within a second of `time.monotonic()`; after `_show_distance(app, -1)` `dist_cm is None` but `dist_stamp` is new (a "no echo" is still a reading)
- [ ] **Step 2: Run.** Expected: the layout tests fail on the missing `lines_var` use, the stamp test fails.
- [ ] **Step 3: Implement** the attributes, the checkbox in the first detector row (text "Distance lines", same style as "Detect paper balls"), the drawing calls in the live branch of `_refresh_camera`, and `dist_stamp`.
- [ ] **Step 4: Run all tests.** Expected: pass. Then run the dashboard with a saved calibration, tick "Distance lines" and check by eye that a ball on a line is at that distance (the owner does this in Task 6's checklist).
- [ ] **Step 5: Commit (only when the owner asks):** `git add Dashboard/dashboard.py Dashboard/tests/test_camera_layout.py Dashboard/tests/test_dashboard_helpers.py`; `feat(dashboard): distance lines on the camera picture, and the time of the newest sensor reading`.

---

### Task 6: The Collect window

**Files:** Modify `Dashboard/dashboard.py` (class `CollectWindow`, adapters `DashboardWorld`, the header button, `open_collect_window`, the manual-control hook in `_send`, `_close`); Test `Dashboard/tests/test_collect_window.py`, `Dashboard/tests/test_dashboard_helpers.py` (hook test), README

**Interfaces — Consumes:** Tasks 1–5, `DashboardCar`, `MovePlayer`, `moves.load_move`, `validate_move`, `MOVES_DIR`. **Produces:**
- `class DashboardWorld(app, tracker: BallTracker)` implementing `World`: `ball()` → `tracker.ball()`; `frame_stamp()` → the newest `DetectionResult.stamp` seen (None before any); `reading()` → `Reading(app.dist_cm, app.dist_stamp)` or None when `dist_stamp` is None; `camera_cm(ball)` → `app.distance_model.distance_cm(ball.bottom_y)` or None without a model
- `class CollectWindow(app, car=None, moves_dir=MOVES_DIR)` with attributes `win`, `approach` (None until started), `samples: list[tuple[float, float]]`, `status_var`; methods `record_sample(distance_cm: float) -> bool` (uses the steady ball's `bottom_y`; False and a status message when there is none), `clear_samples()`, `fit_and_save(path=CALIB_FILE) -> bool` (needs 3 samples; sets `app.distance_model`; the status lists each point's error), `set_grab_spot(path=SPOT_FILE) -> bool` (needs a steady ball and a ultrasonic reading that is not "no echo" and is recent; stores `x_ratio`, the reading and the camera estimate when calibrated; sets `app.grab_spot`), `move_names() -> list[str]` (the `*.json` files of `moves_dir`), `select_move(name)`, `collect() -> bool` (refuses with a message when there is no grab spot, the car is not connected, no valid move is selected, or it is already running; switches detection on; builds `Approach(world, car, MovePlayer(car, SERVO_NAMES), move, app.grab_spot)` and starts it), `stop()`, `tick(now: float)` (feeds the tracker from `app.detector.latest_frame()`, ticks the approach, updates the status and log; called every 50 ms by `win.after`), `close()`
- Dashboard hooks: header button "Collect"; in `_send`'s manual-control branch (U D L R S, not sent by the runner) also `collect_window.stop()` when its approach is active; `_close` calls `collect_window.close()`.

Layout of the window (text sketch): a calibration row (distance entry in cm, "Record", "Clear", "Fit and save"), a samples list with each point's error, a row "Set grab spot", a move chooser (Combobox of move names), "Collect" and a red "STOP", a status line, and a log list of the last 12 decisions.

- [ ] **Step 1: Write the failing tests** (Tk tests skip without a display, like `test_moves_window.py`; a stand-in `app` with `detector` (a fake with `enabled`, `conf` and `latest_frame()`), `distance_model`, `grab_spot`, `dist_cm`, `dist_stamp`, `detect_var`, `connected`, `root`, `moves_window`; `FakeCar` from `test_moves.py`):
  - `test_record_needs_a_steady_ball`: no ball → `record_sample(40)` is False and the status says so; after three detector results with a ball, True and `samples == [(40.0, bottom_y)]`
  - `test_fit_and_save`: three samples on the line of Task 1 → `fit_and_save(tmp/'c.json')` True, the file exists, `app.distance_model.distance_cm(250)` ≈ 40; two samples → False with "3" in the status
  - `test_set_grab_spot`: steady ball at x 0.5 and a reading 16.0 stamped now → True, the saved `GrabSpot` has `x_ratio == 0.5`, `ultrasonic_cm == 16.0`; with the reading `None` ("no echo") or older than 2 s or no ball → False and a message
  - `test_collect_refuses`: with no grab spot / not connected / no move selected / a move with errors / already running → False each, nothing sent to the car
  - `test_collect_runs_to_the_move`: with a fake world feeding a centred steady ball and a reading of 16.0, a move `[wait_step(0.1)]`, `collect()` then ticks with `now` advancing → the approach's player started and the status ends with "Done"; detection was switched on
  - `test_stop_ends_collect_and_stops_the_car`: during a hop, `stop()` → approach inactive, `("stop",)` in the car's calls
  - `test_close_stops_it`
  - `test_move_names_lists_the_json_files` (temp folder with two `.json` and one `.txt`)
  - hook test in `test_dashboard_helpers.py::ManualControlHookTest`: a stand-in with `collect_window.approach.active` True and a `stop` recorder → `Dashboard._send(app, "S")` and `"U"` call it once; `"P100"` does not; with `_runner_sending` True it does not
- [ ] **Step 2: Run.** Expected: fail (`CollectWindow` missing).
- [ ] **Step 3: Implement** the adapters, `CollectWindow` (structure and theme like `MovesWindow`; button handlers are thin wrappers over the tested methods, with `simpledialog`/`messagebox` only in the wrappers), the header button, the hook and the close call.
- [ ] **Step 4: Run all tests and `py_compile`.** Expected: pass.
- [ ] **Step 5: Check on the real car** (the owner; arm in a safe pose; the car on the floor with clear space and a tape measure):
  1. Open **Collect**. Put the ball at 20, 30, 40, 60 and 80 cm in front of the camera (measured from the lens), type each distance, press **Record**, then **Fit and save**. The errors should be within a few cm; tick **Distance lines** in the camera card and check a ball sitting on a line is at that distance.
  2. Put the ball where `pick_down_up` works. Press **Set grab spot** (it should show the sensor reading, about 16 cm).
  3. Put the ball about 60 cm away, a little to one side. Choose `pick_down_up`, press **Collect**. The car should turn to face the ball, hop toward it, and play the move when the sensor reads 14–18 cm.
  4. Test the stops: **STOP** during a hop; hide the ball (stops with "lost the ball"); put a hand in front of the sensor at 16 cm with no ball (it must not pick); put a wall at 16 cm with no ball in view (must not pick).
- [ ] **Step 6: Add one line to the README layout** for `ball_distance.py`, `grab_spot.py`, `ball_approach.py` ("Collect: drive to a ball and pick it up").
- [ ] **Step 7: Commit (only when the owner asks):** `git add Dashboard/dashboard.py Dashboard/tests/test_collect_window.py Dashboard/tests/test_dashboard_helpers.py README.md`; `feat(dashboard): a Collect window that drives to a paper ball and picks it up`.
