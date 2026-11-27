// How far a straight drive has gone, from the two wheel counts. One counter sometimes goes wrong (over-counting, or even running
// backwards), so when the two differ a lot, the trusted one decides. No Arduino code here: tests/drive_wheels_test.cpp.
// The dashboard has the same rule for its own estimate (Dashboard/helpers/path_settings.py).
#pragma once

constexpr int WHEEL_LOWER = 0, WHEEL_RIGHT = 1, WHEEL_LEFT = 2;

struct WheelRule {
  int trust;          // WHEEL_RIGHT or WHEEL_LEFT decides a disagreement; WHEEL_LOWER: always the lower count (the old way)
  float differCm;     // the wheels disagree when they are further apart than this ...
  float differPct;    // ... and than this share of the larger count (so a long drive may differ by more)
};

constexpr float wheelSize(float v) { return v < 0 ? -v : v; }

constexpr bool wheelsDisagree(float left, float right, const WheelRule &rule) {
  float larger = wheelSize(left) > wheelSize(right) ? wheelSize(left) : wheelSize(right);
  float allowed = rule.differPct / 100.0f * larger;
  if (rule.differCm > allowed) allowed = rule.differCm;
  return wheelSize(left - right) > allowed;
}

// The distance that decides whether the drive has arrived (and whether the car still moves).
constexpr float reachedCm(float left, float right, const WheelRule &rule) {
  if (rule.trust != WHEEL_LOWER && wheelsDisagree(left, right, rule)) return rule.trust == WHEEL_RIGHT ? right : left;
  return left < right ? left : right;
}

// The distance the car really covered, for learning how far it rolls on.
constexpr float coveredCm(float left, float right, const WheelRule &rule) {
  if (rule.trust != WHEEL_LOWER && wheelsDisagree(left, right, rule)) return rule.trust == WHEEL_RIGHT ? right : left;
  return (left + right) / 2.0f;
}
