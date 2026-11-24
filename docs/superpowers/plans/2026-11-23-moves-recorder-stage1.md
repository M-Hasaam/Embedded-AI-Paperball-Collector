# Moves Recorder and Player (Stage 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A "Moves" window in the dashboard where the owner records key steps (arm poses, drive, turn, wait), saves them as a JSON file, and test-plays them on the car and arm.

**Architecture:** The logic lives in a new pure module `Dashboard/moves.py` (step data, validation, file I/O, and a `MovePlayer` state machine that talks to the car through a small `PlayerCar` interface), so it is unit-tested with a fake car and no hardware. `dashboard.py` gets a thin adapter (`DashboardCar`) over the existing command paths (servo `V`/`H` commands, `PathRunner` for drives and turns) and the `MovesWindow` UI.

**Tech Stack:** Python 3.12 (`C:\Users\HP\AppData\Local\Programs\Python\Python312\python.exe`), tkinter, stdlib `unittest` (pytest is not installed; add no packages).

**Spec:** `docs/superpowers/specs/2026-11-23-paper-ball-pickup-design.md` (Stage 1 only: no camera, no detection)

## Global Constraints

- Distance is only accurate at 100% speed: every drive step runs at 100% straight speed; the player sets it and restores the user's value afterwards (also after an abort).
- Arm angles go through the same `V<i> <angle> <speed>` command as the sliders so the firmware's limits (including the shoulder-dependent elbow limit) stay in charge. The first command to a servo since the car restarted is preceded by `H<i> <last angle>`, as `_send_servo` does today.
- Steps run one after another; the arm and the car never move at the same time.
- Run Python and tests with the 3.12 interpreter above (`.venv` lacks OpenCV and bleak). No new packages.
- Commit only when the owner asks, only the files each task names, and with no attribution lines in the message.

## Review Focus

1. A hand-edited move file with a missing field, wrong type or out-of-range angle: reported as a list of clear errors; nothing plays; no crash. (Tasks 1 and 4)
2. STOP pressed (window STOP, the dashboard STOP/arrow keys) during an arm step, a drive, a turn or a wait: the car stops, playback ends, later steps never run. (Tasks 2 and 4)
3. The car disconnects, or a drive/turn is aborted (obstacle ahead) mid-move: playback stops and says why; it does not go on to the next arm step. (Task 2)
4. The straight-speed slider is not at 100 when a drive step runs: the drive uses 100 and the old value returns afterwards, including after an abort. (Task 2)
5. Playing an empty move, or pressing Test play while a move is already playing: refused with a message, no double playback. (Tasks 2 and 4)

## File Structure

- Create `Dashboard/moves.py`: step constructors, `validate_move`, `describe_step`, `arm_duration`, `load_move`/`save_move`, `MovePlayer`. No tkinter, no dashboard import.
- Create `Dashboard/moves/example_grab.json`: a documented example of the file format (also a test fixture).
- Create `Dashboard/tests/test_moves.py`, `Dashboard/tests/test_dashboard_helpers.py`, `Dashboard/tests/test_moves_window.py`.
- Modify `Dashboard/dashboard.py`: helpers `servo_commands`, `remaining_along_heading`; `PathRunner` backward arrival; `Dashboard.send_servo_command`, `drive_by`, `turn_by` return value, `runner_ok`; classes `DashboardCar`, `MovesWindow`; header button; `_send` and `_close` hooks.

Run all tests from the repo root:
`& "C:\Users\HP\AppData\Local\Programs\Python\Python312\python.exe" -m unittest discover -s Dashboard/tests -p "test_*.py" -v`
Each test file starts with `sys.path.insert(0, str(Path(__file__).resolve().parents[1]))` so `import moves` / `import dashboard` work.

---

### Task 1: Step data, validation, descriptions, file I/O

**Files:**
- Create: `Dashboard/moves.py`, `Dashboard/moves/example_grab.json`
- Test: `Dashboard/tests/test_moves.py`

**Interfaces:**
- Produces (all in `moves.py`):
  - `arm_step(servos: dict[int, tuple[int, int]], wait: float = 0.5) -> dict` (servo index -> (angle, speed deg/s)); returns `{"type": "arm", "servos": [{"i": 1, "angle": 150, "speed": 30}, ...], "wait": 0.5}` sorted by `i`
  - `drive_step(cm: float) -> dict` = `{"type": "drive", "cm": cm}` (+ forward, - back)
  - `turn_step(deg: float) -> dict` = `{"type": "turn", "deg": deg}` (+ left, like the dashboard)
  - `wait_step(seconds: float) -> dict` = `{"type": "wait", "seconds": seconds}`
  - `new_move(name: str) -> dict` = `{"name": name, "version": 1, "steps": []}`
  - `validate_move(move: dict, limits: list[tuple[int, int]]) -> list[str]` (empty list = valid; `limits[i]` is servo i's (min, max))
  - `describe_step(step: dict, names: list[str]) -> str`
  - `arm_duration(step: dict, current: dict[int, int]) -> float`
  - `load_move(path: Path) -> dict` (raises `MoveFileError(str)` for unreadable JSON or a top level that is not an object with a `steps` list); `save_move(path: Path, move: dict) -> None` (creates the folder, `indent=2`)
  - constants `MOVES_DIR = Path(__file__).with_name("moves")`, `ARM_MARGIN_S = 0.3`, `DRIVE_SPEED_PCT = 100`

Validation ranges (my calls, from the spec's silence): arm angle within `limits[i]`; speed 1..600 deg/s; arm `wait` 0..60 s; drive `0 < |cm| <= 300`; turn `0 < |deg| <= 360`; wait `0 < seconds <= 60`; an arm step needs at least one servo; unknown `type` is an error. Each error names the step number (1-based) and the field.

- [ ] **Step 1: Write the failing tests** in `Dashboard/tests/test_moves.py` (class `StepsTest`, `ValidateTest`, `FileTest`). Use `LIMITS = [(0,120),(80,180),(40,170),(0,180)]` and `NAMES = ["Gripper","Elbow","Shoulder","Servo 4"]`:
  - `test_arm_step_sorts_servos`: `arm_step({2:(120,30),1:(150,30)})["servos"] == [{"i":1,"angle":150,"speed":30},{"i":2,"angle":120,"speed":30}]`
  - `test_describe_each_type`: arm -> `"Arm: Elbow 150°@30/s, Shoulder 120°@30/s, then wait 0.5 s"`; `drive_step(20)` -> `"Drive: forward 20 cm"`; `drive_step(-10)` -> `"Drive: back 10 cm"`; `turn_step(90)` -> `"Turn: left 90°"`; `turn_step(-90)` -> `"Turn: right 90°"`; `wait_step(1)` -> `"Wait: 1 s"`
  - `test_arm_duration_uses_slowest_servo`: current `{1:120,2:90}`, step moves elbow to 150 at 30/s (1.0 s) and shoulder to 120 at 60/s (0.5 s), wait 0.5 -> `1.0 + ARM_MARGIN_S + 0.5` (use `assertAlmostEqual`)
  - `test_arm_duration_without_movement`: servo already at its angle, wait 0.5 -> `ARM_MARGIN_S + 0.5`
  - `test_valid_move_has_no_errors`: a move with one of each step type -> `[]`
  - `test_errors_name_step_and_field`: angle 200 for the elbow -> exactly one error containing `"Step 1"` and `"angle"`; `drive_step(0)`, `turn_step(500)`, `wait_step(0)`, an arm step with `servos: []`, and `{"type": "fly"}` each yield an error containing their step number
  - `test_missing_fields_do_not_crash`: `{"type": "arm"}`, `{"type": "drive"}`, `{"cm": "far", "type": "drive"}` and a move without `"steps"` give errors, never an exception
  - `test_save_load_roundtrip`: save to `tempfile.TemporaryDirectory()` path `sub/x.json`, load -> equal dict
  - `test_load_rejects_garbage`: a file containing `not json` and a file containing `[1,2]` both raise `MoveFileError`
  - `test_example_file_is_valid`: `load_move(MOVES_DIR / "example_grab.json")` validates with `LIMITS`

- [ ] **Step 2: Run to verify failure.** `... -m unittest discover -s Dashboard/tests -p "test_moves.py" -v`. Expected: ImportError (`moves` not found).

- [ ] **Step 3: Implement the interface above in `Dashboard/moves.py`.** One validation pass collects every error (do not stop at the first); coerce nothing silently: a non-number is an error. `describe_step` shows numbers without a trailing `.0` (`20`, not `20.0`).

- [ ] **Step 4: Create `Dashboard/moves/example_grab.json`**: name `"example_grab"`, an arm pose (elbow 150, shoulder 120, speed 30, wait 0.5), `drive 20`, `turn 90`, `wait 1`. It documents the format and is the fixture for `test_example_file_is_valid`.

- [ ] **Step 5: Run the tests.** Expected: all pass.

- [ ] **Step 6: Commit (only when the owner asks)**: `git add Dashboard/moves.py Dashboard/moves Dashboard/tests/test_moves.py`; message `feat(moves): step data, validation and JSON files for recorded arm and car moves`.

---

### Task 2: The player state machine

**Files:**
- Modify: `Dashboard/moves.py`
- Test: `Dashboard/tests/test_moves.py` (add `PlayerTest`)

**Interfaces:**
- Consumes: `arm_duration`, `describe_step`, `DRIVE_SPEED_PCT` from Task 1.
- Produces: `class PlayerCar(Protocol)` with `servo(index: int, angle: int, speed: int) -> None`, `servo_angles() -> dict[int, int]`, `start_drive(cm: float) -> bool`, `start_turn(deg: float) -> bool`, `motion_active() -> bool`, `motion_ok() -> bool`, `get_straight_speed() -> int`, `set_straight_speed(pct: int) -> None`, `stop() -> None`, `connected() -> bool`; and `class MovePlayer` with `__init__(self, car: PlayerCar, names: list[str])`, `start(self, move: dict, now: float) -> bool`, `tick(self, now: float) -> None`, `abort(self, reason: str) -> None`, attributes `active: bool`, `ok: bool | None`, `status: str`, `index: int`.

Behaviour: `start` refuses (returns False, sets `status`, calls nothing on the car) when already active, when the move has no steps, or when `car.connected()` is False. Otherwise it enters step 1 at once. An **arm** step calls `car.servo` for each servo, then waits `arm_duration(step, current)` where `current` starts as `car.servo_angles()` and is updated as steps run. A **drive** step remembers `car.get_straight_speed()` the first time, sets 100, calls `car.start_drive(cm)`; a **turn** step calls `car.start_turn(deg)`; both then wait until `car.motion_active()` is False, and abort with a reason if the car refuses to start (`False`) or `car.motion_ok()` is False afterwards. A **wait** step waits `seconds`. After the last step, or on `abort`, the remembered speed is restored (only if it was changed); `abort` also calls `car.stop()` and is a no-op when not active. `tick` aborts with `"car disconnected"` when `car.connected()` turns False. `active` becomes False before `car.stop()` is called in `abort` (so a stop hook that calls `abort` again returns at once).

- [ ] **Step 1: Write the failing tests** (`PlayerTest`, with a `FakeCar` recording every call in `self.calls` as tuples, `connected_flag`, `speed`, `motion_busy_until` ticks, `motion_result`, `start_result`):
  - `test_plays_steps_in_order`: arm, drive 20, turn 90, wait 1; ticking forward in 0.1 s steps with the fake motion finishing after 0.5 s ends with `player.ok is True`, `player.active is False`, and `calls` showing `servo` calls before `start_drive(20)` before `start_turn(90)`
  - `test_arm_waits_for_travel`: elbow 120 -> 150 at 30/s (1 s), wait 0: nothing else starts at `t0 + 1.0`, the next step starts by `t0 + 1.0 + ARM_MARGIN_S`
  - `test_drive_forces_100_and_restores`: slider at 60; during the drive `car.speed == 100`; after `ok` it is 60 again; no `set_straight_speed` calls at all when the move has no drive step
  - `test_abort_restores_speed_and_stops`: abort mid-drive -> `("stop",)` called once, speed back to 60, `ok is False`, later steps never start
  - `test_abort_twice_is_harmless`: second abort adds no calls
  - `test_disconnect_aborts`: `connected_flag = False` mid-move -> status contains `"disconnected"`, `("stop",)` called
  - `test_failed_motion_aborts_and_skips_arm_steps`: `motion_result = False` after a drive -> `ok is False`, the arm step after it never calls `servo`
  - `test_refuses_empty_and_double_start`: `start` on `{"steps": []}` -> False with status `"Nothing to play"` and no calls; a second `start` while active -> False, no new calls
  - `test_refuses_when_not_connected`: -> False, status `"Car not connected"`
  - `test_drive_start_refused_aborts`: `start_result = False` -> `ok is False`, status contains `"could not start"`

- [ ] **Step 2: Run to verify failure.** Expected: `AttributeError`/`ImportError` for `MovePlayer`.

- [ ] **Step 3: Implement `PlayerCar` and `MovePlayer`** as specified. `tick(now)` and `start(move, now)` take the time as an argument (no clock inside), so tests drive time by hand. `status` reads like `"Step 2/4: Drive: forward 20 cm"`, `"Done"`, `"Stopped: <reason>"`.

- [ ] **Step 4: Run the tests.** Expected: all pass.

- [ ] **Step 5: Commit (only when the owner asks)**: `git add Dashboard/moves.py Dashboard/tests/test_moves.py`; message `feat(moves): a player that runs recorded steps one after another and stops safely`.

---

### Task 3: Dashboard plumbing (helpers, runner fix, adapter)

**Files:**
- Modify: `Dashboard/dashboard.py` (module level near `return_steps`; `PathRunner.tick` arrival at the `moving` state; `Dashboard` near `_send_servo`, `turn_by`, `_runner_finished`; new class `DashboardCar` before `PulseWindow`)
- Test: `Dashboard/tests/test_dashboard_helpers.py`

**Interfaces:**
- Consumes: `PlayerCar` (Task 2).
- Produces in `dashboard.py`:
  - `servo_commands(index: int, angle: int, speed: int, last_angle: int | None = None) -> list[str]`: `["V<i> <angle> <speed>"]`, preceded by `"H<i> <last_angle>"` when `last_angle` is not None
  - `remaining_along_heading(x: float, y: float, heading_deg: float, tx: float, ty: float) -> float`: how far ahead (along the car's heading, 0 = up/+y, + = left) the target point still is; negative once passed
  - `Dashboard.send_servo_command(self, index: int, angle: int, speed: int) -> None` (the old `_send_servo` body; also sets `servo_scales[index]`, `speed_scales[index]`, `last_angles`; uses `servo_commands` with `last_angle` only on the first command since the car started, i.e. when `index not in self.servo_commanded`)
  - `Dashboard.drive_by(self, cm: float) -> bool`, `Dashboard.turn_by(self, delta) -> bool` (existing, now returns whether the runner started), `Dashboard.runner_ok: bool` (set in `_runner_finished`)
  - `class DashboardCar(app)` implementing `PlayerCar`

- [ ] **Step 1: Write the failing tests** in `Dashboard/tests/test_dashboard_helpers.py` (`import dashboard`; the module has no side effects at import under Python 3.12):
  - `test_servo_commands_plain`: `servo_commands(1, 150, 30) == ["V1 150 30"]`
  - `test_servo_commands_first_command_sets_held_angle`: `servo_commands(1, 150, 30, 120) == ["H1 120", "V1 150 30"]`
  - `test_remaining_ahead`: heading 0, car (0,0), target (0,10) -> 10.0; heading 90 (facing left), target (-10,0) -> 10.0; target already passed (0,-2) at heading 0 -> -2.0 (`assertAlmostEqual`)
  - `test_runner_stops_backward_move_at_target`: a stub tracker (`types.SimpleNamespace`: `x=0.0, y=-5.0, cmd="D", cm_per_s_100=30.0, corners=[], heading_now=lambda now: 0.0`), `PathRunner(stub, sent.append, lambda: 100, lambda: None)`, then set `runner.active=True; runner.steps=[["move", -20.0, 0.0, -20.0]]; runner.idx=0; runner.state="moving"; runner.until=1e9; runner.move_from=(0.0, 0.0); runner.coast_cm=10.0`. `runner.tick(1.0, 0.0)` sends nothing; after `stub.y = -15.0`, `runner.tick(2.0, 0.0)` sends `"S"` (arrival within the coast distance). Also assert the same stub moved forward (`steps=[["move", 20.0, 0.0, 20.0]]`, `y=15.0`, `cmd="U"`) still stops (existing behaviour kept).

- [ ] **Step 2: Run to verify failure.** Expected: `AttributeError: module 'dashboard' has no attribute 'servo_commands'`.

- [ ] **Step 3: Implement the two module-level helpers**, then change the `moving` arrival test in `PathRunner.tick` to use `remaining_along_heading` for every move that has a target point: forward arrives when `remaining <= max(0.5, coast_cm)`, backward when `remaining >= -max(0.5, coast_cm)`. Moves without a target point stay time-limited.

- [ ] **Step 4: Run the tests.** Expected: the four new tests pass.

- [ ] **Step 5: Refactor the dashboard methods.** `_send_servo(index)` becomes `self.send_servo_command(index, int(scale), int(speed scale))`; `send_servo_command` sets the two sliders (so Add-step captures what was played), sends each line of `servo_commands(...)` through `self._send`, and keeps the `last_angles`/memory-save behaviour. Add `drive_by(cm)` modelled on `turn_by`: not-ready or runner-active returns False; sends `"S"`; target point `tx = x - sin(h)*cm`, `ty = y + cos(h)*cm` from `tracker.heading_now(now)`; `runner.start([["move", cm, tx, ty]], f"Drive {cm:+.0f} cm", now)`; returns its result. Make `turn_by` return True/False the same way. Set `self.runner_ok = True` in `__init__` and `self.runner_ok = ok` first thing in `_runner_finished`.

- [ ] **Step 6: Add `DashboardCar`**: `servo` -> `app.send_servo_command`; `servo_angles` -> `{i: int(app.servo_scales[i].get())}`; `start_drive`/`start_turn` -> `app.drive_by`/`app.turn_by`; `motion_active` -> `app.runner.active`; `motion_ok` -> `app.runner_ok`; `get_straight_speed` -> `int(app.car_speed.get())`; `set_straight_speed(pct)` -> `app.car_speed.set(pct)` then `app._send_speed()`; `stop` -> `app.path_abort()`; `connected` -> `app.connected.is_set()`.

- [ ] **Step 7: Verify nothing regressed.** Run the whole test folder (expected: all pass) and `python -m py_compile Dashboard/dashboard.py`. Then, with the car on and the arm disconnected, open the dashboard and check by hand: a slider still moves its servo; "All default" still works; "Return to start" / a ±90° turn button still run.

- [ ] **Step 8: Commit (only when the owner asks)**: `git add Dashboard/dashboard.py Dashboard/tests/test_dashboard_helpers.py`; message `refactor(dashboard): servo and drive/turn commands the Moves player can call; backward drives stop at their target`.

---

### Task 4: The Moves window

**Files:**
- Modify: `Dashboard/dashboard.py` (class `MovesWindow` after `PulseWindow`; `Dashboard.__init__`, header button, `open_moves_window`, `_send` hook, `_close`); `README.md` (one line in the repository layout)
- Test: `Dashboard/tests/test_moves_window.py`

**Interfaces:**
- Consumes: everything from Tasks 1-3.
- Produces: `class MovesWindow(app, car=None)` (`car` defaults to `DashboardCar(app)`) with attributes `win`, `move` (the dict being edited), `player` (a `MovePlayer`); methods `add_arm_pose(wait: float) -> None` (captures all four `app.servo_scales`/`speed_scales`), `add_drive(cm: float)`, `add_turn(deg: float)`, `add_wait(seconds: float)`, `delete_selected()`, `move_selected(delta: int)`, `edit_selected_value(value: float)` (drive/turn/wait steps) and `recapture_selected_pose()` (arm steps), `errors() -> list[str]`, `save(path: Path)`, `load(path: Path)`, `play()`, `stop()`, `close()`. The buttons call these through small dialog wrappers (`simpledialog.askfloat`, `filedialog`, a `messagebox.askokcancel("The arm and car will move. Is the area clear?")` before playing).
- Dashboard hooks: `Dashboard.moves_window` (None until opened); in `_send`, the existing manual-control branch for `U D L R S` also calls `moves_window.player.abort("manual control")` when a player is active; `_close` calls `moves_window.close()`.

Layout (text sketch): a step list on top; rows of buttons `Add arm pose | Add drive (cm) | Add turn (deg) | Add wait (s)`, `Edit | Delete | Up | Down`, `Save... | Open...`, then `Test play | STOP` and a status line. The window polls `player.tick(time.monotonic())` every 50 ms with `win.after`, and its list text is `describe_step(step, SERVO_NAMES)`.

- [ ] **Step 1: Write the failing tests** in `Dashboard/tests/test_moves_window.py` (skip the whole class with `unittest.skipIf` when `tk.Tk()` raises `TclError`; build a fake app: `root`, `_button`, four `tk.Scale` objects for `servo_scales` and four for `speed_scales`, `connected`; pass a fake `car` from Task 2's `FakeCar` pattern):
  - `test_add_arm_pose_captures_all_sliders`: sliders set to 90, 150, 120, 45 with speeds 30 -> `window.move["steps"][0]` equals `arm_step({0:(90,30),1:(150,30),2:(120,30),3:(45,30)}, wait=0.5)`
  - `test_add_delete_and_reorder`: add drive 20 then turn 90, `move_selected(1)` on the first swaps them, `delete_selected()` removes the selected one
  - `test_save_load_roundtrip`: save to a temp path, clear, load -> the same steps, list text shows each step's description
  - `test_play_refused_when_move_invalid`: an arm step with elbow angle 200 -> `errors()` is non-empty, `play()` does not call the car and the status shows the first error
  - `test_play_and_stop`: valid move, `play()` (the "Is the area clear?" dialog lives only in the Test play button's wrapper, not in `play()`), tick a few times, `stop()` -> `player.active is False` and the car got `("stop",)`
  - `test_second_play_while_playing_is_refused`: `play()` twice -> the car's call list has no duplicates of step 1

- [ ] **Step 2: Run to verify failure.** Expected: `AttributeError: module 'dashboard' has no attribute 'MovesWindow'`.

- [ ] **Step 3: Implement `MovesWindow`** following `PulseWindow` for structure and theme constants (`BG`, `CARD`, `FONT`, `app._button`). `errors()` = `validate_move(self.move, list(zip(SERVO_MIN_ANGLES, SERVO_MAX_ANGLES)))`. `play()` refuses on errors, an empty move, and a player that is already active; otherwise `player.start(self.move, time.monotonic())`. `close()` calls `player.abort("window closed")` and destroys the window; the default open/save folder is `MOVES_DIR`.

- [ ] **Step 4: Wire it into `Dashboard`**: `self.moves_window = None` in `__init__`; header button `"Moves"` next to `"Pulse chart"`; `open_moves_window` like `open_pulse_window`; the `_send` hook and the `_close` call described above.

- [ ] **Step 5: Run all tests and `py_compile`.** Expected: all pass.

- [ ] **Step 6: Check on the real car** (the owner, with a tape measure; take the arm's safe pose first):
  1. Open the dashboard, connect, click **Moves**. Pose the arm with the sliders, **Add arm pose**; **Add drive** 20; **Add turn** 90; **Add wait** 1. **Save...** as `Dashboard/moves/test1.json`, clear, **Open...** it again: the list is back.
  2. **Test play**: the arm moves to the pose, the car drives 20 cm (within about 1-2 cm of the tape), turns about 90 degrees, waits. Set the straight-speed slider to 50 first: during the drive it goes to 100 and is back at 50 afterwards.
  3. Press **STOP** in the Moves window mid-drive: the car stops and the status says so. Press the dashboard's STOP button or an arrow key mid-drive: the same.
  4. Edit the saved file by hand to set an elbow angle of 200: Open shows the error, **Test play** refuses.
  5. Close the dashboard while playing: the car stops.

- [ ] **Step 7: Add one line to the `README.md` repository layout**: `moves/  recorded arm and car moves (JSON), made in the dashboard's Moves window`, under `Dashboard/`.

- [ ] **Step 8: Commit (only when the owner asks)**: `git add Dashboard/dashboard.py Dashboard/tests/test_moves_window.py README.md`; message `feat(dashboard): a Moves window to record, save and replay arm and car steps`.
