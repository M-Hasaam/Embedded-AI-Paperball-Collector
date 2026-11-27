// Checks of gyro_still.h. They run while compiling (static_assert), so no board is needed: any C++17 compiler will do, e.g.
// the ESP32 one:   xtensa-esp32s3-elf-g++ -std=gnu++17 -fsyntax-only tests/gyro_still_test.cpp
// (The Arduino IDE does not build this folder.)
#include "../gyro_still.h"

constexpr float noise(int i, float size) { return ((i % 3) - 1) * size; }      // -size, 0, +size, ...
constexpr float distance(float a, float b) { return a > b ? a - b : b - a; }

// 200 samples of a car: gyro = zero error + turning (changing evenly from `from` to `to` deg/s) + noise, flat on the floor
constexpr GyroStill samples(float from, float to, float gyroNoise, float shake = 0.0f, int count = 200) {
  GyroStill w;
  for (int i = 0; i < count; i++) {
    float turning = from + (to - from) * i / count;
    w.add(2.0f + noise(i, gyroNoise), 0.3f + noise(i + 1, gyroNoise), -0.1f + turning + noise(i + 2, gyroNoise),
          -0.04f + noise(i, shake), 0.01f, 0.94f + noise(i + 1, shake));
  }
  return w;
}

// a car standing still: accepted, and the mean is the gyro's zero error (what this car's sensor really shows)
constexpr GyroStill QUIET = samples(0, 0, 0.15f);
static_assert(QUIET.still(200), "a still car must be accepted");
static_assert(distance(QUIET.mean[0], 2.0f) < 0.05f && distance(QUIET.mean[1], 0.3f) < 0.05f && distance(QUIET.mean[2], -0.1f) < 0.05f,
              "the mean of the still readings is the zero error");

static_assert(!samples(0, 15, 0.15f).still(200), "a turn that speeds up is movement");
static_assert(!samples(0, 0, 5.0f).still(200), "a car shaken about is movement");
static_assert(!samples(0, 0, 0.15f, 0.12f).still(200), "a quiet gyro with a shaking accelerometer is movement (carried)");
static_assert(!samples(30, 30, 0.15f).still(200), "30 deg/s is more than any zero error: the car is turning");
static_assert(!samples(0, 0, 0.15f, 0.0f, 50).still(200), "too few readings prove nothing");

// should the stored zero be replaced by what a still window shows?
constexpr float RIGHT[3] = {2.0f, 0.3f, -0.1f};
constexpr float SLIGHTLY_OFF[3] = {2.4f, 0.3f, -0.4f};
constexpr float TAKEN_WHILE_MOVING[3] = {2.1f, 13.0f, -15.0f};                 // what was seen on the car
static_assert(!gyroZeroIsWrong(QUIET, RIGHT, true), "a right zero is left alone");
static_assert(!gyroZeroIsWrong(QUIET, SLIGHTLY_OFF, true), "small errors are left to the slow correction");
static_assert(gyroZeroIsWrong(QUIET, TAKEN_WHILE_MOVING, true), "a zero taken while moving must be replaced");
static_assert(gyroZeroIsWrong(QUIET, RIGHT, false), "without a good calibration yet, the still window is the calibration");

constexpr bool emptiedByReset() {
  GyroStill w = samples(0, 0, 0.15f);
  w.reset();
  return w.n == 0 && w.mean[0] == 0.0f && !w.still(1);
}
static_assert(emptiedByReset(), "reset() starts a new window");
