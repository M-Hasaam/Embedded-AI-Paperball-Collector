# Paper-ball pickup: approach the ball, then replay a recorded grab

Date: 2026-11-23. Status: draft for approval. Nothing here is built yet.

## Intent

The robot (ESP32-S3 car with a 4-servo arm, driven over Bluetooth from the PC dashboard) should collect a paper ball without anyone driving it.
When its camera sees a paper ball it drives toward it. When the ball is at a set distance from the arm, it runs a set of moves the owner has
recorded beforehand (arm poses, gripper, a short drive), then waits for the next ball.

The owner asked "how should I record those moves?". Answer in this design: record them as **key steps** in a "Moves" window of the dashboard,
saved as a small file, replayed by a player that also drives the approach hand-off.

## Goals

1. **Moves recorder and player** (dashboard window). A move is an ordered list of steps:
   - *arm*: the four servo angles, a speed, then wait (the gripper is servo 0)
   - *drive*: straight forward or back by N cm at 100% or 50% speed, counted by the car's own wheel encoders (changed after the first
     real-car test: the PC-side runner cancelled short drives and could not stop a 5-10 cm drive exactly)
   - *turn*: N degrees on the spot (the car's own closed-loop turn)
   - *wait*: N seconds
   "Add step" captures the current slider angles; steps can be edited, reordered, deleted and test-played; moves are saved to and loaded from
   a JSON file (`Embedded/moves/<name>.json`) that can also be edited by hand.
2. **Ball detection in the dashboard**: run the v2.1 paper-ball model (`Training/paper_ball_detector/roboflow-paperball.onnx`) on the camera frames,
   draw the boxes on the camera view, and report the ball's left/right offset and its floor distance (the same method and calibration idea as
   `Embedded/esp32/WifiBridge/scripts/camera_distance.py`).
3. **Approach controller**: while "Collect" is on, repeat: ball seen (confidence at least 0.5 for several frames in a row)? turn to centre it, drive a hop
   at 100% (never past the hand-off distance), look again. When the ball is at the hand-off distance, play the recorded *grab* move, then go back to waiting.

## Non-goals (first version)

- Searching for balls: the car waits until one is in view.
- Driving back to the start after a grab.
- Choosing between several recorded moves automatically; one "grab" move.
- Working out arm angles from the ball position (inverse kinematics) or learning anything.
- Running on the phone app or on the ESP32 itself.

## Anti-goals (a failure even if it works)

- Any arm angle outside the existing limits (including the shoulder-dependent elbow limit table). The player goes through the same path as the sliders, so
  the firmware's limits stay in charge.
- The car not stopping at once on STOP/ABORT, or driving on after the camera has gone quiet.
- A recorded drive that lands in a different place each time (see the constraint on speed).

## Constraints

- **Distance is only accurate at 100% speed** (measured with a tape: within 0.5-3% at 100%, wrong by 24-34% at about 60-90%). Every drive step and every
  approach hop uses 100%; the player sets it and restores the user's speed afterwards.
- **The camera is at the front of the car, low, looking straight ahead, in the same place and direction as the ultrasonic sensor.** A low camera cannot see a ball
  that is close, so the ball will leave the picture before the grab distance. The hand-off distance must be measured (smallest distance at which the ball is still
  detected reliably), and the last stretch is a blind recorded drive.
- The camera's Wi-Fi link has been unreliable; the controller must stop and hold when frames stop arriving for about 1 s.
- The dashboard runs on Python 3.12 (`C:\Users\HP\AppData\Local\Programs\Python\Python312`), which already has OpenCV and bleak. The detector uses OpenCV's ONNX
  reader (no new packages).
- Servos stay idle after power-up until first moved; the player starts them the same way the sliders do.

## Approaches considered

*Final stretch.* A: the camera guides the car to a hand-off distance, then one recording takes over (including the last blind drive). B: camera plus ultrasonic guide
the car all the way to the grab spot (fragile: the ultrasonic beam often misses a small ball on the floor). **Picked by the owner: A.**

*Recording.* Key steps (pose, press "Add step"; editable; smooth replay with the arm's own speed control) against live capture of the slider movement (jerky, noisy,
hard to correct). **Picked by the owner: key steps.**

*Where it runs (my call).* On the PC in the dashboard, next to the camera and the Bluetooth link; the model does not fit the ESP32, and the phone app is untested on a device.

## Calls I made (the owner can overrule)

- The drive and approach use 100% speed only; hops are short so the ball is re-checked often; the turn toward the ball uses the accurate on-the-spot turn.
- The controller stops (and says why) on STOP/ABORT, on 1 s without camera frames, when the ball has been lost for several seconds, and when the ultrasonic
  distance is under a safety limit.
- Steps of a move run one after another (the arm and the car never move at the same time), and the camera is ignored while a move is playing.
- Moves are plain JSON files in `Embedded/moves/`; a "grab" move is the one the controller replays.
- Distance from the camera uses a short calibration with the ball at known distances (like `camera_distance.py --calibrate`).
- Thresholds (confidence, hop size, centring tolerance, safety distance) start as constants at the top of the code and are tuned on the real car.

## Staging

1. **Stage 1: moves recorder and player.** No camera needed; testable with the arm and car as they are. Includes the 100% drive steps and test-play.
2. **Stage 2: detection, approach, hand-off.** Detector overlay, calibration, the approach loop, and calling the grab move. Needs the hand-off distance measured.

Each stage gets its own plan.

## Left to the builder

Window layout and key bindings; the exact JSON field names; how the player waits for a servo to arrive (the firmware reports angles); the calibration screen;
the numbers listed above; what the status line says.

## Stage 2a: the trained model in the dashboard (approved 2026-11-24)

The first piece of Stage 2: run the v2.1 paper-ball model on the live camera picture and show what it finds. No driving yet.

- `Embedded/ball_detector.py`: the model code (picture prepared with the same letterbox as the phone app, the output decoded and
  de-duplicated), a drawing helper and a background worker. The model is `Training/paper_ball_detector/roboflow-paperball.onnx`,
  read with the OpenCV the dashboard already has (about 32 ms a picture on this laptop). The worker takes the newest camera frame,
  skips frames while it is busy, and loads the model only when detection is first switched on.
- In the camera card: a **"Detect paper balls"** checkbox (off by default), a minimum-confidence slider (0.25 to 0.90, default 0.50),
  a status line ("2 balls, best 87%, 31 ms"), and a **"Test on a photo..."** button that runs the same model on a picture from disk
  and shows the result in the camera area (so the model can be checked while the camera is down), with a "Back to live" button.
- Boxes are drawn in green with their scores, the best one thicker. The model is shared by the live worker and the photo test and
  runs one picture at a time.
- Not in this stage: distance from the picture, steering the car, saving detections.
