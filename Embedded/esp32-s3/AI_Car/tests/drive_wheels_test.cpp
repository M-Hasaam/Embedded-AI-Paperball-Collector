// Checks of drive_wheels.h, run by compiling (static_assert):
//   xtensa-esp32s3-elf-g++ -std=gnu++17 -fsyntax-only tests/drive_wheels_test.cpp
#include "../drive_wheels.h"

constexpr bool near(float a, float b) { return a - b < 0.01f && b - a < 0.01f; }
constexpr WheelRule RIGHT{WHEEL_RIGHT, 4.0f, 15.0f}, LEFT{WHEEL_LEFT, 4.0f, 15.0f}, LOWER{WHEEL_LOWER, 4.0f, 15.0f};

// seen on the car: on a 10 cm drive the left counter said 65.4 cm and the right one 9.8
static_assert(wheelsDisagree(65.4f, 9.8f, RIGHT), "that is a disagreement");
static_assert(near(reachedCm(65.4f, 9.8f, RIGHT), 9.8f), "the right wheel decides when to stop");
static_assert(near(coveredCm(65.4f, 9.8f, RIGHT), 9.8f), "and how far the car went");
static_assert(near(reachedCm(65.4f, 9.8f, LEFT), 65.4f), "unless the left one is trusted");
static_assert(near(reachedCm(65.4f, 9.8f, LOWER), 9.8f), "the old way: the lower count");
static_assert(near(reachedCm(25.0f, -1700.0f, LOWER), -1700.0f), "the old way believes a counter that runs backwards");
static_assert(near(reachedCm(25.0f, -1700.0f, LEFT), 25.0f), "trusting the other one does not");

// wheels that agree: the car stops on the lower count (an encoder chattering must not end a drive early), the distance is the mean
static_assert(!wheelsDisagree(20.0f, 22.0f, RIGHT), "2 cm apart is normal");
static_assert(near(reachedCm(22.0f, 20.0f, RIGHT), 20.0f) && near(coveredCm(22.0f, 20.0f, RIGHT), 21.0f), "agreeing wheels");
static_assert(!wheelsDisagree(100.0f, 91.0f, RIGHT), "on a long drive the allowed difference grows: 15% of 100 cm");
static_assert(wheelsDisagree(100.0f, 80.0f, RIGHT), "20 cm on 100 cm is too much");
static_assert(wheelsDisagree(9.0f, 4.0f, RIGHT) && !wheelsDisagree(7.0f, 4.0f, RIGHT), "on a short drive 4 cm is the limit");
static_assert(wheelsDisagree(-20.0f, 5.0f, RIGHT), "opposite signs");
