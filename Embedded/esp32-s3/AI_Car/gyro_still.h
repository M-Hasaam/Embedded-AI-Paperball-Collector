// Is the car really standing still? Decided from a window of gyro and accelerometer readings, so that the gyro's zero point is
// only ever taken from a still car. (A calibration made while the car was being handled once left a zero error of 15 deg/s:
// the heading ran away while the car stood on the floor.) No Arduino code here: tests/gyro_still_test.cpp checks it.
#pragma once

constexpr float GYRO_STILL_STD_DPS = 0.6f;    // a still gyro's readings scatter less than this (this car's: about 0.1)
constexpr float ACC_STILL_STD_G = 0.03f;      // ... and the accelerometer's less than this
constexpr float GYRO_ZERO_MAX_DPS = 20.0f;    // the MPU-6050 data sheet's largest zero error: a steady reading above it is movement
constexpr float GYRO_ZERO_TOL_DPS = 1.0f;     // a stored zero further than this from a still window's mean is replaced

// Running mean and scatter of the six readings: gyro x, y, z (deg/s, before any zero correction), accelerometer x, y, z (g).
struct GyroStill {
  int n = 0;
  float mean[6] = {0, 0, 0, 0, 0, 0};
  float m2[6] = {0, 0, 0, 0, 0, 0};           // sum of squared distances from the mean (Welford's method)

  constexpr void reset() {
    n = 0;
    for (int i = 0; i < 6; i++) { mean[i] = 0; m2[i] = 0; }
  }

  constexpr void add(float gx, float gy, float gz, float ax, float ay, float az) {
    const float value[6] = {gx, gy, gz, ax, ay, az};
    n++;
    for (int i = 0; i < 6; i++) {
      float delta = value[i] - mean[i];
      mean[i] += delta / n;
      m2[i] += delta * (value[i] - mean[i]);
    }
  }

  // At least `samples` readings that hardly differ from each other, with a gyro mean small enough to be a zero error.
  constexpr bool still(int samples) const {
    if (n < samples || n < 2) return false;
    for (int i = 0; i < 6; i++) {
      float limit = i < 3 ? GYRO_STILL_STD_DPS : ACC_STILL_STD_G;
      if (m2[i] / n > limit * limit) return false;
      if (i < 3 && (mean[i] > GYRO_ZERO_MAX_DPS || mean[i] < -GYRO_ZERO_MAX_DPS)) return false;
    }
    return true;
  }
};

// Whether a window that has been found still shows that the stored zero (deg/s, x y z) has to be replaced by its mean.
constexpr bool gyroZeroIsWrong(const GyroStill& window, const float (&zero)[3], bool calibrated) {
  if (!calibrated) return true;
  for (int i = 0; i < 3; i++) {
    float off = window.mean[i] - zero[i];
    if (off > GYRO_ZERO_TOL_DPS || off < -GYRO_ZERO_TOL_DPS) return true;
  }
  return false;
}
