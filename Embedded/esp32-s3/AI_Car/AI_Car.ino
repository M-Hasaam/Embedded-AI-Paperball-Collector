#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <Wire.h>
#include <math.h>

// HC-SR04 ultrasonic sensor pins.
// The sensor is powered from the S3's 3V3 pin (on the converter's 5V it sent the S3 into a brownout restart loop
// at power-on), so ECHO is 3.3V and goes straight to the pin. If you power it from 5V again, put a divider on ECHO
// (1k in series, 2k to GND): the S3 pins are not 5V tolerant.
const int TRIG_PIN = 1;   // moved from GPIO 10 to the header with the MPU (TX/RX side of the board)
const int ECHO_PIN = 2;   // moved from GPIO 11

// Active buzzer: + to BUZZER_PIN, - to GND. Beeps while an obstacle is closer than BUZZER_ON_CM.
// (An active buzzer has its own tone generator; all 8 PWM channels are already used by wheels and servos.)
const int BUZZER_PIN = 14;
const float BUZZER_ON_CM = 20.0f;                 // alarm starts below this distance
const float BUZZER_OFF_CM = 23.0f;                // alarm stops above this (gap avoids flicker at the limit)
// A single false short echo (noise on the ECHO line, a stray reflection) used to start the alarm with nothing in front of the
// car. Now a reading below the sensor's 2 cm minimum counts as noise, the distance used is the median of the last three
// readings, and the alarm needs BUZZER_CONFIRM close readings in a row.
const float SENSOR_MIN_CM = 2.0f;
const int BUZZER_CONFIRM = 2;
const unsigned long BUZZER_BEEP_MS = 120;         // beep on / off time while alarming

const char* DISTANCE_CHAR_UUID = "11223344-5566-7788-99AA-BBCCDDEEFF00";
const unsigned long DISTANCE_INTERVAL_MS = 100;   // time between distance readings
const unsigned long ECHO_TIMEOUT_US = 25000;      // ~4.3 m max range

// MPU-6050 (I2C). AD0 -> GND gives address 0x68.
const int MPU_SDA_PIN = 41;
const int MPU_SCL_PIN = 42;
const uint8_t MPU_ADDR = 0x68;
const unsigned long MPU_READ_INTERVAL_MS = 20;    // sensor read / filter update
const unsigned long MPU_NOTIFY_INTERVAL_MS = 40;  // BLE + Serial output (25 Hz: fresh heading for gyro-controlled turns)
const float MPU_FILTER_ALPHA = 0.98f;             // complementary filter: gyro vs accel weight
const float MPU_GYRO_LSB_PER_DPS = 65.5f;         // gyro range +-500 deg/s
const float MPU_YAW_DEADBAND_DPS = 0.3f;          // ignore tiny gyro noise so heading doesn't creep
const int MPU_CAL_SAMPLES = 200;                  // gyro offset calibration (keep the car still)
const char* MPU_CHAR_UUID = "22334455-6677-8899-AABB-CCDDEEFF0011";

// Built-in RGB status LED (GPIO 48 on most ESP32-S3 boards; if your board shows no colour, its LED is on another pin).
void statusLed(uint8_t r, uint8_t g, uint8_t b) {
#ifdef RGB_BUILTIN
  neopixelWrite(RGB_BUILTIN, r, g, b);
#endif
}

// Start-up step marker kept in RTC memory (survives a brownout/crash reset), so the next boot can show on the LED
// how far the previous start-up got. Steps: 1 motors, 2 sensor pins, 3 encoders, 4 MPU, 5 BLE init,
// 6 GATT service, 7 advertising started (setup finished), 8 main loop running, 9 servos started.
#define BOOT_MAGIC 0xB007CAFEu
RTC_NOINIT_ATTR uint32_t bootMagic;
RTC_NOINIT_ATTR uint8_t bootStage;
void markStage(uint8_t step) { bootMagic = BOOT_MAGIC; bootStage = step; }

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
BLECharacteristic* pDistanceCharacteristic = nullptr;
BLECharacteristic* pMpuCharacteristic = nullptr;

bool deviceConnected = false;
unsigned long lastDistanceRead = 0;
bool distanceAlarm = false;                       // true while an obstacle is too close

bool mpuOk = false;
float mpuPitch = 0, mpuRoll = 0;                  // degrees
float mpuYaw = 0;                                 // heading change in degrees, + = counter-clockwise from above
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0; // deg/s measured while still
volatile bool mpuCalRequested = false;
volatile bool mpuYawResetRequested = false;
float mpuAx = 0, mpuAy = 0, mpuAz = 0;            // g
float mpuGx = 0, mpuGy = 0, mpuGz = 0;            // deg/s
unsigned long lastMpuRead = 0, lastMpuNotify = 0, lastMpuMicros = 0, lastMpuRetry = 0;

const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 7;

// Keep the existing wheel mapping unchanged.
const int LU = IN2;
const int LD = IN1;
const int RU = IN3;
const int RD = IN4;

// Wheel encoders (VCC -> 3V3, GND, D0 -> pin; A0 left unconnected).
const int ENCODER_LEFT_PIN = 8;
const int ENCODER_RIGHT_PIN = 3;   // GPIO 3 is a strapping pin: if the board fails to boot, move this wire

// Set these two to match your hardware (distance and speed depend on them).
const int ENCODER_SLOTS_PER_REV = 40;             // calibrated: 100 cm of travel gave ~196 ticks (0.51 cm/tick) with a 6.5 cm wheel
const float WHEEL_DIAMETER_CM = 6.5f;
const float ENCODER_CM_PER_TICK = 3.14159265f * WHEEL_DIAMETER_CM / ENCODER_SLOTS_PER_REV;
const unsigned long ENCODER_DEBOUNCE_US = 300;    // ignore pulses closer together than this (noise)
const unsigned long ENCODER_NOTIFY_INTERVAL_MS = 200;
const char* ENCODER_CHAR_UUID = "33445566-7788-99AA-BBCC-DDEEFF001122";

BLECharacteristic* pEncoderCharacteristic = nullptr;
BLECharacteristic* pServoStatusCharacteristic = nullptr;
const char* SERVO_STATUS_CHAR_UUID = "44556677-8899-AABB-CCDD-EEFF00112233";
volatile long encoderLeftTicks = 0, encoderRightTicks = 0;     // signed: + forward, - reverse
volatile int encoderLeftDir = 1, encoderRightDir = 1;          // direction of the last motor command
volatile unsigned long encoderLeftLastUs = 0, encoderRightLastUs = 0;
unsigned long lastEncoderNotify = 0;
long lastEncoderLeftTicks = 0, lastEncoderRightTicks = 0;

// A single-channel encoder cannot sense direction, so the sign comes from the motor command.
void IRAM_ATTR encoderLeftIsr() {
  unsigned long now = micros();
  if (now - encoderLeftLastUs < ENCODER_DEBOUNCE_US) return;
  encoderLeftLastUs = now;
  encoderLeftTicks += encoderLeftDir;
}

void IRAM_ATTR encoderRightIsr() {
  unsigned long now = micros();
  if (now - encoderRightLastUs < ENCODER_DEBOUNCE_US) return;
  encoderRightLastUs = now;
  encoderRightTicks += encoderRightDir;
}

const int NUM_SERVOS = 4;
// Servo index -> GPIO: 0 gripper (GPIO 15), 1 elbow (GPIO 16), 2 shoulder (GPIO 17), 3 spare "Servo 4" (GPIO 18).
// (The base-rotation servo was removed from the arm, so the indices were renamed to match the wiring.)
const int SERVO_PINS[NUM_SERVOS] = {15, 16, 17, 18};

// Servo PWM: 50 Hz, 14-bit, pulse 500..2400 us (same range as before).
const int SERVO_FREQ_HZ = 50;
const int SERVO_PWM_BITS = 14;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

// The servos do NOT move at power-up: their signal pins stay silent until a servo is commanded for the first time
// (see attachServo). That keeps the arm still when the board restarts, avoids a sudden jump to a default angle, and
// keeps the servo signal wires quiet while the radio starts (that made the board brown out right at power-on).
bool servoAttached[4] = {false, false, false, false};   // one flag per servo (NUM_SERVOS is declared above)

const int SERVO_ANGLE_MIN[NUM_SERVOS] = {0, 80, 40, 0};         // lowest angle per servo (elbow 80, shoulder 40)
const int SERVO_ANGLE_MAX[NUM_SERVOS] = {120, 180, 170, 180};  // highest angle per servo (gripper 120, shoulder 170)
// These are compiled into the firmware, so they hold after every restart and re-flash. writeServo() clamps to them too.

int clampElbowForShoulder(int angle);   // defined below, with the elbow limit table

void writeServo(int index, int angle) {
  if (!servoAttached[index]) return;
  if (angle < SERVO_ANGLE_MIN[index]) angle = SERVO_ANGLE_MIN[index];   // last line of defence: no path can drive a servo outside its limits
  if (angle > SERVO_ANGLE_MAX[index]) angle = SERVO_ANGLE_MAX[index];
  if (index == 1) angle = clampElbowForShoulder(angle);     // the elbow also has a limit that depends on the shoulder angle (below)
  uint32_t us = SERVO_MIN_US + (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180;
  uint32_t duty = (uint64_t)us * ((1UL << SERVO_PWM_BITS) - 1) / (1000000UL / SERVO_FREQ_HZ);
  ledcWrite(SERVO_PINS[index], duty);
}

// Wheel speed: PWM on the motor-driver pins, one setting for all wheels (command P<0-100>, percent).
const int MOTOR_PINS[4] = {LU, LD, RU, RD};
const int MOTOR_PWM_FREQ_HZ = 1000;
const int MOTOR_PWM_BITS = 8;
const uint32_t MOTOR_PWM_MAX = (1UL << MOTOR_PWM_BITS) - 1;
int motorSpeedPct = 100;    // wheel speed for driving straight (U, D), percent
int turnSpeedPct = 70;      // wheel speed for turning on the spot (L, R), percent
int motorState[4] = {0, 0, 0, 0};  // last requested direction (1 = on) for LU, LD, RU, RD

// Straight-drive assist (see updateStraightAssist): power scale of each side, 1.0 = full. The faster side is
// reduced so both sides keep the same speed. Command G<mode>: bit 0 (1) = gyro heading hold, bit 1 (2) = encoder
// matching, so G0 = off, G1 = gyro, G2 = encoders, G3 = both (the two errors are averaged).
int straightMode = 0;
float leftScale = 1.0f, rightScale = 1.0f;

// Start-up angle of each servo: {gripper, elbow, shoulder, servo 4}. The elbow (servo 1) starts at 120: at 90 it
// hits the ultrasonic sensor.
#define SERVO_DEFAULTS {90, 120, 90, 90}
const int SERVO_DEFAULT_ANGLES[NUM_SERVOS] = SERVO_DEFAULTS;
int currentAngles[NUM_SERVOS] = SERVO_DEFAULTS;
int targetAngles[NUM_SERVOS] = SERVO_DEFAULTS;
const int SERVO_SPEED_DEFAULT = 30;                // deg/s a servo moves at unless a command says otherwise (so nothing is ever instant)
int servoSpeeds[NUM_SERVOS] = {SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT, SERVO_SPEED_DEFAULT};  // degrees per second
unsigned long lastServoUpdate[NUM_SERVOS] = {0, 0, 0, 0};
int servoLastDir[NUM_SERVOS] = {0, 0, 0, 0};       // direction of each servo's last step: +1 up, -1 down, 0 none yet
float servoAccum[NUM_SERVOS] = {0, 0, 0, 0};       // degrees still owed from the elapsed time (see updateServos)
const int SERVO_SPEED_MAX = 600;                   // highest speed (deg/s) a command may ask for; the servo itself may be slower
const int SERVO_BACKOFF_DEG = 5;                   // when the overload guard stops a servo, it steps this far back from the load

// ---- Elbow limit that depends on the shoulder angle ----
// The elbow can break if it goes outside this window for the shoulder's angle (measured on the arm; plotted in
// docs/arm-limits). Between two rows the TIGHTER of the two is used, and LIMIT_MARGIN keeps the elbow a little inside
// every limit that is tighter than the servo's own 80..180 range. See elbowWindow(), updateServos() and the V / H commands.
const int ELBOW = 1, SHOULDER = 2;
const int LIMIT_ROWS = 14;
const int LIMIT_SHOULDER[LIMIT_ROWS]  = {  40,  60,  70,  80,  90, 100, 110, 115, 120, 130, 140, 150, 160, 170};
const int LIMIT_ELBOW_MIN[LIMIT_ROWS] = { 135, 120, 105, 100, 100, 100, 100, 100,  95,  95,  95,  90, 100, 120};
const int LIMIT_ELBOW_MAX[LIMIT_ROWS] = { 180, 180, 180, 180, 180, 180, 180, 180, 180, 170, 170, 160, 150, 145};
const int LIMIT_MARGIN = 3;                        // degrees kept inside the table's limits (servo overshoot and play)
const int LIMIT_LOOKAHEAD = 5;                     // the elbow moves clear of the next few shoulder degrees ahead of the shoulder
int elbowDesired = SERVO_DEFAULT_ANGLES[ELBOW];    // where the elbow was last asked to go; it is pulled into the window while it moves
bool elbowWaiting = false;                         // the elbow was commanded before the shoulder started: it starts right after it
int elbowWaitStart = -1;                           // its start angle while it waits
int elbowWaitSpeed = SERVO_SPEED_DEFAULT;          // and the speed it was asked for
bool shoulderHeld = false;                         // the shoulder is waiting for the elbow to get out of the way

// Allowed elbow range for one shoulder angle.
void elbowWindowAt(int s, int &lo, int &hi) {
  lo = SERVO_ANGLE_MIN[ELBOW];
  hi = SERVO_ANGLE_MAX[ELBOW];
  if (s < LIMIT_SHOULDER[0]) s = LIMIT_SHOULDER[0];
  if (s > LIMIT_SHOULDER[LIMIT_ROWS - 1]) s = LIMIT_SHOULDER[LIMIT_ROWS - 1];
  for (int k = 0; k < LIMIT_ROWS - 1; k++) {
    if (s < LIMIT_SHOULDER[k] || s > LIMIT_SHOULDER[k + 1]) continue;   // exactly on a row: both neighbouring segments apply
    int segLo = LIMIT_ELBOW_MIN[k] > LIMIT_ELBOW_MIN[k + 1] ? LIMIT_ELBOW_MIN[k] : LIMIT_ELBOW_MIN[k + 1];
    int segHi = LIMIT_ELBOW_MAX[k] < LIMIT_ELBOW_MAX[k + 1] ? LIMIT_ELBOW_MAX[k] : LIMIT_ELBOW_MAX[k + 1];
    if (segLo > lo) lo = segLo;
    if (segHi < hi) hi = segHi;
  }
  if (lo > SERVO_ANGLE_MIN[ELBOW]) lo += LIMIT_MARGIN;
  if (hi < SERVO_ANGLE_MAX[ELBOW]) hi -= LIMIT_MARGIN;
  if (lo > hi) lo = hi = (lo + hi) / 2;
}

// Allowed elbow range for every shoulder angle from sA to sB (inclusive): the part that is allowed all the way.
void elbowWindow(int sA, int sB, int &lo, int &hi) {
  if (sA > sB) { int x = sA; sA = sB; sB = x; }
  elbowWindowAt(sA, lo, hi);
  for (int s = sA + 1; s <= sB; s++) {
    int l, h;
    elbowWindowAt(s, l, h);
    if (l > lo) lo = l;
    if (h < hi) hi = h;
  }
  if (lo > hi) lo = hi = (lo + hi) / 2;
}

// The window the elbow has to stay in right now: allowed for the shoulder's current angle and for the next few degrees it is
// about to move through. Also returns the shoulder angle it was worked out for.
void elbowWindowNow(int &sNow, int &lo, int &hi) {
  sNow = servoAttached[SHOULDER] ? currentAngles[SHOULDER] : SERVO_DEFAULT_ANGLES[SHOULDER];
  int sAhead = sNow;
  if (servoAttached[SHOULDER] && targetAngles[SHOULDER] != sNow) {
    int span = abs(targetAngles[SHOULDER] - sNow);
    if (span > LIMIT_LOOKAHEAD) span = LIMIT_LOOKAHEAD;
    sAhead = sNow + (targetAngles[SHOULDER] > sNow ? span : -span);
  }
  elbowWindow(sNow, sAhead, lo, hi);
}

// The angle the elbow limit is holding the elbow at, or -1 while no limit is acting (the asked-for angle is inside the window).
int elbowHeldAngle() {
  if (elbowWaiting || !servoAttached[ELBOW]) return -1;
  int sNow, lo, hi;
  elbowWindowNow(sNow, lo, hi);
  return (elbowDesired < lo || elbowDesired > hi) ? targetAngles[ELBOW] : -1;
}

// Text for the dashboard when a limit is acting (empty when everything is fine). It travels after a ';' in the servo status.
void elbowLimitMessage(char* out, size_t n) {
  out[0] = 0;
  if (elbowWaiting) {
    snprintf(out, n, "ELBOW waits: start the shoulder first (its limit depends on the shoulder angle)");
    return;
  }
  if (!servoAttached[ELBOW]) return;
  int sNow, lo, hi;
  elbowWindowNow(sNow, lo, hi);
  if (shoulderHeld) {
    snprintf(out, n, "SHOULDER waits: elbow must move out of the way (allowed %d-%d deg at shoulder %d)", lo, hi, sNow);
  } else if (elbowDesired < lo || elbowDesired > hi) {
    snprintf(out, n, "ELBOW LIMIT: %d deg asked, held at %d (allowed %d-%d deg at shoulder %d)",
             elbowDesired, targetAngles[ELBOW], lo, hi, sNow);
  }
}

// An elbow angle pulled into the window for the shoulder's current angle (its default angle if the shoulder has not started).
int clampElbowForShoulder(int angle) {
  int lo, hi;
  elbowWindowAt(servoAttached[SHOULDER] ? currentAngles[SHOULDER] : SERVO_DEFAULT_ANGLES[SHOULDER], lo, hi);
  return angle < lo ? lo : (angle > hi ? hi : angle);
}

// ---- Servo overload guard (state) ----
// A servo that strains or stalls pulls a lot of current, and its supply voltage dips. Each servo supply goes through a
// 3:1 resistor divider to an ADC pin; updateServoGuard() watches how far the voltage sags below its normal level and
// slows or stops the servos on that supply. Rail 0 = the two MG996R (elbow + shoulder), rail 1 = the SG90 (gripper).
// If nothing is wired to a pin, that rail is simply not guarded.
const int RAIL_PINS[2] = {9, 10};                  // ADC1 pins
const float RAIL_DIVIDER = 3.0f;                   // top = two 9.76k in series, bottom = one 9.76k: the pin sees a third of the rail
                                                   // (about 1.7 V, the middle of the ADC's accurate range; 2:1 read too high)
const int servoRail[NUM_SERVOS] = {1, 0, 0, 0};    // which rail feeds each servo: gripper, elbow, shoulder, servo 4
float sagSlowMv = 250.0f;                          // rail this far below its normal level: move at half speed (command T<slow> <stop>)
float sagStopMv = 400.0f;                          // this far below: stop moving until it recovers
bool servoGuardOn = true;                          // command O1 = guard on, O0 = off (the voltages are still reported)
float railSagMax[2] = {0, 0};                      // largest sag since the last report, so short dips are not missed
unsigned long railStopSince[2] = {0, 0};
const unsigned long SAG_HOLD_MS = 2000;            // stay stopped at least this long
float railV[2] = {0, 0}, railBase[2] = {0, 0};     // smoothed voltage and its normal level, in mV
float railNoise[2] = {0, 0};                       // largest spread (highest - lowest of 9 readings) since the last report, at the rail
bool railPresent[2] = {false, false};
int railState[2] = {0, 0};                         // 0 = ok, 1 = slowed, 2 = stopped
float railSpeedScale[2] = {1.0f, 1.0f};
unsigned long railSagSince[2] = {0, 0}, railStoppedAt[2] = {0, 0};

// Starts the 50 Hz signal for one servo the first time it is commanded. The servo goes straight to that first angle
// (it has no earlier position to move from), and later commands move it gradually at the set speed.
void attachServo(int i, int firstAngle) {
  if (servoAttached[i]) return;
  if (i == ELBOW && !servoAttached[SHOULDER]) {   // the elbow's limit depends on the shoulder, so the shoulder has to start first
    elbowWaiting = true;
    if (elbowWaitStart < 0) elbowWaitStart = firstAngle;   // the first angle wins (the remembered one, if the dashboard sent it)
    Serial.println("Elbow waits: it starts as soon as the shoulder has started (its limit depends on the shoulder angle)");
    return;
  }
  if (i == ELBOW) firstAngle = clampElbowForShoulder(firstAngle);
  if (!ledcAttach(SERVO_PINS[i], SERVO_FREQ_HZ, SERVO_PWM_BITS)) {
    Serial.printf("ERROR: could not attach servo %d on pin %d\n", i, SERVO_PINS[i]);
    return;
  }
  servoAttached[i] = true;
  servoSpeeds[i] = SERVO_SPEED_DEFAULT;     // a freshly started servo always begins at the default speed
  currentAngles[i] = firstAngle;
  targetAngles[i] = firstAngle;
  writeServo(i, firstAngle);
  markStage(9);
  Serial.printf("Servo %d started at %d degrees\n", i, firstAngle);
  if (i == SHOULDER && elbowWaiting) {            // the shoulder is running now: start the elbow that was waiting for it
    elbowWaiting = false;
    attachServo(ELBOW, elbowWaitStart < 0 ? elbowDesired : elbowWaitStart);
    elbowWaitStart = -1;
    servoSpeeds[ELBOW] = elbowWaitSpeed;
  }
}

void motors(int lu, int ld, int ru, int rd) {
  motorState[0] = lu;
  motorState[1] = ld;
  motorState[2] = ru;
  motorState[3] = rd;

  // Remember each side's direction for the encoders (kept while stopped so coasting counts the same way).
  if (lu || ld) encoderLeftDir = lu ? 1 : -1;
  if (ru || rd) encoderRightDir = ru ? 1 : -1;

  bool turning = (lu && rd) || (ld && ru);   // the two sides run in opposite directions: turning on the spot
  uint32_t duty = MOTOR_PWM_MAX * (turning ? turnSpeedPct : motorSpeedPct) / 100;
  for (int i = 0; i < 4; i++) {
    float scale = (i < 2) ? leftScale : rightScale;  // MOTOR_PINS = {LU, LD, RU, RD}: 0,1 = left side, 2,3 = right
    ledcWrite(MOTOR_PINS[i], motorState[i] ? (uint32_t)(duty * scale) : 0);
  }
}

void handleCommand(String message) {
  message.trim();
  if (message.length() == 0) return;

  // Motor commands: U, D, L, R, S
  if (message.length() == 1) {
    char command = message.charAt(0);
    Serial.print("Motor command: ");
    Serial.println(command);

    switch (command) {
      case 'U': motors(1, 0, 1, 0); break;
      case 'D': motors(0, 1, 0, 1); break;
      case 'L': motors(0, 1, 1, 0); break;
      case 'R': motors(1, 0, 0, 1); break;
      case 'S': motors(0, 0, 0, 0); break;
      case 'Z': mpuYawResetRequested = true; break;  // heading back to 0
      case 'C': mpuCalRequested = true; break;       // re-calibrate gyro (car must be still)
      case 'E':                                      // reset encoder distance to 0
        noInterrupts();
        encoderLeftTicks = 0;
        encoderRightTicks = 0;
        interrupts();
        lastEncoderLeftTicks = 0;
        lastEncoderRightTicks = 0;
        break;
      default: Serial.println("Unknown motor command"); break;
    }
    return;
  }

  // Straight-drive assist: G0 = off, G1 = gyro, G2 = encoders, G3 = both.
  if (message.charAt(0) == 'G' && message.length() == 2 && message.charAt(1) >= '0' && message.charAt(1) <= '3') {
    straightMode = message.charAt(1) - '0';
    Serial.printf("Straight assist mode %d (1 = gyro, 2 = encoders, 3 = both, 0 = off)\n", straightMode);
    return;
  }

  // Servo overload guard: O1 = on, O0 = off; T<slow> <stop> = sag thresholds in mV (slow down / stop the servos).
  if (message.charAt(0) == 'O' && message.length() == 2 && (message.charAt(1) == '0' || message.charAt(1) == '1')) {
    servoGuardOn = (message.charAt(1) == '1');
    Serial.printf("Servo overload guard: %s\n", servoGuardOn ? "on" : "off");
    return;
  }
  if (message.charAt(0) == 'T') {
    int slow = -1, stop = -1;
    if (sscanf(message.c_str(), "T%d %d", &slow, &stop) == 2 && slow >= 20 && stop > slow && stop <= 3000) {
      sagSlowMv = (float)slow;
      sagStopMv = (float)stop;
      Serial.printf("Servo guard thresholds: slow at %d mV, stop at %d mV\n", slow, stop);
    } else {
      Serial.println("Invalid threshold command. Use: T250 500");
    }
    return;
  }

  // Turn speed: Q<0-100> (percent) for turning on the spot (L, R). Also applies to a turn that is running right now.
  if (message.charAt(0) == 'Q') {
    int pct = -1;
    if (sscanf(message.c_str(), "Q%d", &pct) == 1 && pct >= 0 && pct <= 100) {
      turnSpeedPct = pct;
      motors(motorState[0], motorState[1], motorState[2], motorState[3]);
      Serial.printf("Turn speed: %d%%\n", pct);
    } else {
      Serial.println("Invalid turn speed. Use: Q60");
    }
    return;
  }

  // Drive speed: P<0-100> (percent) for driving straight forward or backward (U, D). Also applies to wheels that are running right now.
  if (message.charAt(0) == 'P') {
    int pct = -1;
    if (sscanf(message.c_str(), "P%d", &pct) == 1 && pct >= 0 && pct <= 100) {
      motorSpeedPct = pct;
      motors(motorState[0], motorState[1], motorState[2], motorState[3]);
      Serial.printf("Wheel speed: %d%%\n", pct);
    } else {
      Serial.println("Invalid speed. Use: P75");
    }
    return;
  }

  // H<servo-index> <angle>: "this servo is probably at this angle now" (the dashboard remembers where it left it). A servo
  // that has not been started yet starts its signal at that angle, so the next V command glides from there at the set
  // speed instead of snapping to the target. Does nothing for a servo that is already running.
  if (message.charAt(0) == 'H') {
    int servoIndex = -1, angle = -1;
    if (sscanf(message.c_str(), "H%d %d", &servoIndex, &angle) == 2 && servoIndex >= 0 && servoIndex < NUM_SERVOS &&
        angle >= 0 && angle <= 180) {
      if (angle < SERVO_ANGLE_MIN[servoIndex]) angle = SERVO_ANGLE_MIN[servoIndex];
      if (angle > SERVO_ANGLE_MAX[servoIndex]) angle = SERVO_ANGLE_MAX[servoIndex];
      attachServo(servoIndex, angle);
    } else {
      Serial.println("Invalid hold command. Use: H1 120");
    }
    return;
  }

  // Servo command format: V<servo-index> <angle> <speed deg/sec>
  // Examples: V0 90 60, V1 120 30, V3 160 120
  if (message.charAt(0) == 'V') {
    int servoIndex = -1;
    int angle = -1;
    int speed = -1;

    int fields = sscanf(message.c_str(), "V%d %d %d", &servoIndex, &angle, &speed);
    if (fields == 2 && servoIndex >= 0 && servoIndex < NUM_SERVOS) speed = servoSpeeds[servoIndex];   // no speed given: keep the servo's speed (30 deg/s at first)
    if (fields == 2 || fields == 3) {
      if (servoIndex >= 0 && servoIndex < NUM_SERVOS &&
          angle >= 0 && angle <= 180 &&
          speed >= 1 && speed <= SERVO_SPEED_MAX) {
        if (angle < SERVO_ANGLE_MIN[servoIndex]) angle = SERVO_ANGLE_MIN[servoIndex];
        if (angle > SERVO_ANGLE_MAX[servoIndex]) angle = SERVO_ANGLE_MAX[servoIndex];
        if (servoIndex == ELBOW) elbowDesired = angle;   // updateServos() pulls the elbow into the window for the shoulder's angle
        attachServo(servoIndex, angle);   // first command for this servo: start its signal at that angle
        if (servoIndex == ELBOW && !servoAttached[ELBOW]) {   // waiting for the shoulder to start
          elbowWaitSpeed = speed;
        } else {
          targetAngles[servoIndex] = angle;
          servoSpeeds[servoIndex] = speed;
        }
        Serial.printf("Servo %d | Target: %d | Speed: %d deg/sec\n",
                      servoIndex, angle, speed);
      } else {
        Serial.println("Servo index, angle or speed out of range");
      }
    } else {
      Serial.println("Invalid servo command. Use: V0 90 60");
    }
  }
}

// Moves each servo toward its target at its set speed (degrees per second, up to SERVO_SPEED_MAX). The distance to
// move is accumulated from the elapsed time, so speeds above what one-degree-per-loop steps allow (about 200 deg/s)
// work too: the servo then moves several degrees per step.
void updateServos() {
  unsigned long now = millis();

  // The elbow's target is where it was asked to go, pulled into the window that is allowed for the shoulder's angle now
  // and for the next few degrees the shoulder is about to move through.
  if (servoAttached[ELBOW]) {
    int sNow, lo, hi;
    elbowWindowNow(sNow, lo, hi);
    targetAngles[ELBOW] = elbowDesired < lo ? lo : (elbowDesired > hi ? hi : elbowDesired);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servoAttached[i]) continue;  // not started yet: its signal pin stays silent
    if (currentAngles[i] == targetAngles[i]) { lastServoUpdate[i] = now; servoAccum[i] = 0; continue; }

    int rail = servoRail[i];
    if (railState[rail] == 2) { lastServoUpdate[i] = now; servoAccum[i] = 0; continue; }  // overload: hold still
    float speed = servoSpeeds[i] * railSpeedScale[rail];
    if (speed < 1.0f) speed = 1.0f;

    unsigned long dtMs = now - lastServoUpdate[i];
    if (dtMs > 100) dtMs = 100;  // a long pause (e.g. the loop was busy) must not turn into a jump
    lastServoUpdate[i] = now;
    servoAccum[i] += speed * dtMs / 1000.0f;
    int steps = (int)servoAccum[i];
    if (steps >= 1) {
      servoAccum[i] -= steps;
      int remaining = abs(targetAngles[i] - currentAngles[i]);
      if (steps > remaining) steps = remaining;
      int dir = (currentAngles[i] < targetAngles[i]) ? 1 : -1;
      if (i == SHOULDER && servoAttached[ELBOW]) {
        // the shoulder only moves while the elbow is inside the window for every angle it is about to pass
        while (steps > 0) {
          int lo, hi;
          elbowWindow(currentAngles[i], currentAngles[i] + dir * steps, lo, hi);
          if (currentAngles[ELBOW] >= lo && currentAngles[ELBOW] <= hi) break;
          steps--;
        }
        if (steps == 0) {          // the elbow has to move out of the way first (it is already being pulled there)
          servoAccum[i] = 0;
          if (!shoulderHeld) { shoulderHeld = true; Serial.println("Shoulder waits: the elbow has to move out of the way first"); }
          continue;
        }
        shoulderHeld = false;
      }
      currentAngles[i] += dir * steps;
      servoLastDir[i] = dir;
      writeServo(i, currentAngles[i]);
    }
  }
}

// Returns distance in cm, or -1 if no echo was received (out of range).
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

// Reads the sensor periodically, prints it and notifies the BLE client.
void updateDistance() {
  unsigned long now = millis();
  if (now - lastDistanceRead < DISTANCE_INTERVAL_MS) return;
  lastDistanceRead = now;

  float raw = readDistanceCm();
  if (raw >= 0 && raw < SENSOR_MIN_CM) raw = -1;            // below the sensor's minimum range: noise, not an obstacle

  // Median of the last three readings (no echo counts as a very large distance), so one stray value is ignored.
  static float recent[3] = {9999.0f, 9999.0f, 9999.0f};
  static int recentAt = 0;
  recent[recentAt] = raw < 0 ? 9999.0f : raw;
  recentAt = (recentAt + 1) % 3;
  float lo = recent[0] < recent[1] ? recent[0] : recent[1];
  float hi = recent[0] < recent[1] ? recent[1] : recent[0];
  float median = recent[2] < lo ? lo : (recent[2] > hi ? hi : recent[2]);
  float cm = median >= 9999.0f ? -1.0f : median;

  // Alarm: on after BUZZER_CONFIRM close readings in a row, off above BUZZER_OFF_CM, off when nothing is in range (-1).
  static int closeReadings = 0;
  if (cm < 0) { distanceAlarm = false; closeReadings = 0; }
  else if (cm < BUZZER_ON_CM) { if (++closeReadings >= BUZZER_CONFIRM) distanceAlarm = true; }
  else { closeReadings = 0; if (cm > BUZZER_OFF_CM) distanceAlarm = false; }

  char buf[16];
  if (cm < 0) snprintf(buf, sizeof(buf), "-1");
  else snprintf(buf, sizeof(buf), "%.1f", cm);

  Serial.printf("Distance: %s cm\n", buf);

  if (deviceConnected && pDistanceCharacteristic) {
    pDistanceCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pDistanceCharacteristic->notify();
  }
}

// Beeps (non-blocking) while distanceAlarm is set, silent otherwise.
void updateBuzzer() {
  static unsigned long lastToggle = 0;
  static bool on = false;
  if (!distanceAlarm) {
    if (on) { on = false; digitalWrite(BUZZER_PIN, LOW); }
    return;
  }
  unsigned long now = millis();
  if (now - lastToggle >= BUZZER_BEEP_MS) {  // alternate beep / silence
    on = !on;
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    lastToggle = now;
  }
}

// Averages the gyro while the car is still to find its zero offset. Takes ~1 s.
void calibrateGyro() {
  float sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < MPU_CAL_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // GYRO_XOUT_H
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU_ADDR, (uint8_t)6) == 6) {
      int16_t g[3];
      for (int k = 0; k < 3; k++) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        g[k] = (int16_t)((hi << 8) | lo);
      }
      sx += g[0]; sy += g[1]; sz += g[2];
      n++;
    }
    delay(5);
  }
  if (n > 0) {
    gyroBiasX = sx / n / MPU_GYRO_LSB_PER_DPS;
    gyroBiasY = sy / n / MPU_GYRO_LSB_PER_DPS;
    gyroBiasZ = sz / n / MPU_GYRO_LSB_PER_DPS;
  }
  mpuYaw = 0;
  Serial.printf("Gyro bias: %.2f %.2f %.2f deg/s\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

bool mpuInit() {
  // If SDA or SCL is held low (short to GND, unpowered module, missing ground) the I2C driver
  // can crash the whole board. Check the lines first and skip the MPU instead of crashing.
  Wire.end();
  pinMode(MPU_SDA_PIN, INPUT_PULLUP);
  pinMode(MPU_SCL_PIN, INPUT_PULLUP);
  delay(2);
  if (digitalRead(MPU_SDA_PIN) == LOW || digitalRead(MPU_SCL_PIN) == LOW) return false;
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return false;
  uint8_t who = Wire.read();
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1: wake up, use X-gyro clock
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  // CONFIG: digital low-pass filter ~44 Hz (less noise)
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG: +-500 deg/s so fast hand turns don't clip
  Wire.write(0x08);
  Wire.endTransmission();
  // Accel stays at default +-2g (16384 LSB/g).

  delay(100);
  calibrateGyro();
  return true;
}

// Reads the MPU and updates the tilt angles (complementary filter).
void updateMpu() {
  unsigned long now = millis();
  if (mpuOk && mpuCalRequested) {
    mpuCalRequested = false;
    calibrateGyro();
    lastMpuMicros = micros();
  }
  if (mpuYawResetRequested) {
    mpuYawResetRequested = false;
    mpuYaw = 0;
  }
  if (!mpuOk) {
    // Not found yet: retry every 2 s (e.g. wired after boot) and list I2C devices.
    if (now - lastMpuRetry >= 2000) {
      lastMpuRetry = now;
      mpuOk = mpuInit();
      lastMpuMicros = micros();
      if (mpuOk) {
        Serial.println("MPU-6050 ready");
      } else {
        if (digitalRead(MPU_SDA_PIN) == LOW || digitalRead(MPU_SCL_PIN) == LOW) {
          Serial.println("MPU not found: SDA/SCL held LOW (short to GND, no power or no ground on the MPU?)");
        } else {
          Serial.print("MPU not found. I2C devices:");
          int found = 0;
          Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
          for (uint8_t a = 1; a < 127; a++) {
            Wire.beginTransmission(a);
            if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
          }
          Wire.end();
          Serial.println(found ? "" : " none");
        }
      }
    }
    return;
  }
  if (now - lastMpuRead >= MPU_READ_INTERVAL_MS) {
    lastMpuRead = now;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU_ADDR, (uint8_t)14) == 14) {
      int16_t raw[7];
      for (int i = 0; i < 7; i++) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        raw[i] = (int16_t)((hi << 8) | lo);
      }
      mpuAx = raw[0] / 16384.0f;
      mpuAy = raw[1] / 16384.0f;
      mpuAz = raw[2] / 16384.0f;
      mpuGx = raw[4] / MPU_GYRO_LSB_PER_DPS - gyroBiasX;
      mpuGy = raw[5] / MPU_GYRO_LSB_PER_DPS - gyroBiasY;
      mpuGz = raw[6] / MPU_GYRO_LSB_PER_DPS - gyroBiasZ;

      unsigned long us = micros();
      float dt = (us - lastMpuMicros) / 1e6f;
      lastMpuMicros = us;
      if (dt <= 0 || dt > 0.5f) dt = MPU_READ_INTERVAL_MS / 1000.0f;

      float accRoll = atan2f(mpuAy, mpuAz) * 180.0f / M_PI;
      float accPitch = atan2f(-mpuAx, sqrtf(mpuAy * mpuAy + mpuAz * mpuAz)) * 180.0f / M_PI;
      if (fabsf(mpuGz) > MPU_YAW_DEADBAND_DPS) {
        mpuYaw += mpuGz * dt;
        if (mpuYaw > 180.0f) mpuYaw -= 360.0f;
        if (mpuYaw < -180.0f) mpuYaw += 360.0f;
      }
      mpuRoll = MPU_FILTER_ALPHA * (mpuRoll + mpuGx * dt) + (1 - MPU_FILTER_ALPHA) * accRoll;
      mpuPitch = MPU_FILTER_ALPHA * (mpuPitch + mpuGy * dt) + (1 - MPU_FILTER_ALPHA) * accPitch;
    }
  }
}

// Sends "yaw,pitch,roll,gx,gy,gz,ax,ay,az" (deg x3, deg/s x3, g x3) over BLE.
void notifyMpu() {
  unsigned long now = millis();
  if (now - lastMpuNotify < MPU_NOTIFY_INTERVAL_MS) return;
  lastMpuNotify = now;

  char buf[96];
  if (mpuOk) {
    snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f",
             mpuYaw, mpuPitch, mpuRoll, mpuGx, mpuGy, mpuGz, mpuAx, mpuAy, mpuAz);
  } else {
    snprintf(buf, sizeof(buf), "NA");
  }
  Serial.printf("MPU: %s\n", buf);

  if (deviceConnected && pMpuCharacteristic) {
    pMpuCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pMpuCharacteristic->notify();
  }
}

// Sends "leftTicks,rightTicks,leftCm,rightCm,leftCmPerS,rightCmPerS" over BLE (+ = forward).
void updateEncoders() {
  unsigned long now = millis();
  unsigned long dt = now - lastEncoderNotify;
  if (dt < ENCODER_NOTIFY_INTERVAL_MS) return;
  lastEncoderNotify = now;

  noInterrupts();
  long left = encoderLeftTicks;
  long right = encoderRightTicks;
  interrupts();

  float seconds = dt / 1000.0f;
  float leftSpeed = (left - lastEncoderLeftTicks) * ENCODER_CM_PER_TICK / seconds;
  float rightSpeed = (right - lastEncoderRightTicks) * ENCODER_CM_PER_TICK / seconds;
  lastEncoderLeftTicks = left;
  lastEncoderRightTicks = right;

  char buf[80];
  snprintf(buf, sizeof(buf), "%ld,%ld,%.1f,%.1f,%.1f,%.1f",
           left, right, left * ENCODER_CM_PER_TICK, right * ENCODER_CM_PER_TICK, leftSpeed, rightSpeed);
  Serial.printf("Encoder: %s\n", buf);

  if (deviceConnected && pEncoderCharacteristic) {
    pEncoderCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pEncoderCharacteristic->notify();
  }
}

// ---- Servo overload guard -----------------------------------------------------------------------------------
// Watches each servo supply and reacts to a voltage sag: at sagSlowMv the servos on that supply move at half
// speed, at sagStopMv they stop where they are (targets are set to the current angles) until the voltage recovers.
void updateServoGuard() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 20) return;
  last = now;
  for (int r = 0; r < 2; r++) {
    uint32_t samples[9];                                    // median of 9 readings: a noise spike cannot move it
    for (int k = 0; k < 9; k++) samples[k] = analogReadMilliVolts(RAIL_PINS[r]);
    for (int a = 1; a < 9; a++) {                           // insertion sort
      uint32_t key = samples[a];
      int b = a - 1;
      while (b >= 0 && samples[b] > key) { samples[b + 1] = samples[b]; b--; }
      samples[b + 1] = key;
    }
    float pinMv = (float)samples[4];
    float spread = (float)(samples[8] - samples[0]) * RAIL_DIVIDER;   // how noisy the pin is right now, expressed at the rail
    if (spread > railNoise[r]) railNoise[r] = spread;
    railPresent[r] = (pinMv > 800.0f && pinMv < 3150.0f);  // something sensible is wired to this pin
    if (!railPresent[r]) {
      railState[r] = 0; railSpeedScale[r] = 1.0f; railBase[r] = 0; railV[r] = 0; railSagSince[r] = 0;
      continue;
    }
    float v = pinMv * RAIL_DIVIDER;                                         // rail voltage in mV
    railV[r] = (railV[r] == 0) ? v : railV[r] + 0.25f * (v - railV[r]);     // light smoothing
    if (railBase[r] == 0) railBase[r] = railV[r];
    // The normal level follows the voltage slowly in both directions (battery drift), and is frozen while the guard is
    // slowing or stopping the servos. A short rise (a servo pushes energy back into the supply) can therefore not
    // raise it and later look like a sag.
    if (railState[r] == 0 && (railBase[r] - railV[r]) < sagSlowMv) railBase[r] += 0.002f * (railV[r] - railBase[r]);
    float sag = railBase[r] - railV[r];                                      // negative = above the normal level
    if (sag > railSagMax[r]) railSagMax[r] = sag;

    if (!servoGuardOn) {      // guard switched off from the dashboard: only report, never slow or stop
      railState[r] = 0; railSpeedScale[r] = 1.0f; railSagSince[r] = 0; railStopSince[r] = 0;
      continue;
    }
    if (railState[r] == 2) {  // stopped: resume once the voltage is back and the hold time has passed
      if (sag < sagSlowMv && now - railStoppedAt[r] >= SAG_HOLD_MS) { railState[r] = 0; railSpeedScale[r] = 1.0f; }
      continue;
    }
    if (sag >= sagStopMv) {   // a real overload has to last 100 ms, so a single noisy reading cannot stop the servos
      if (railStopSince[r] == 0) railStopSince[r] = now;
    } else {
      railStopSince[r] = 0;
    }
    if (railStopSince[r] != 0 && now - railStopSince[r] >= 100) {
      railState[r] = 2;
      railStoppedAt[r] = now;
      railSpeedScale[r] = 0.0f;
      for (int i = 0; i < NUM_SERVOS; i++) {
        if (servoRail[i] != r) continue;
        int hold = currentAngles[i];
        if (servoAttached[i] && servoLastDir[i] != 0) {  // step back from the load to relieve the pushing force
          hold -= servoLastDir[i] * SERVO_BACKOFF_DEG;
          if (hold < SERVO_ANGLE_MIN[i]) hold = SERVO_ANGLE_MIN[i];
          if (hold > SERVO_ANGLE_MAX[i]) hold = SERVO_ANGLE_MAX[i];
          if (i == ELBOW) hold = clampElbowForShoulder(hold);
          currentAngles[i] = hold;
          writeServo(i, hold);
        }
        targetAngles[i] = hold;                          // and hold there until the supply has recovered
        if (i == ELBOW) elbowDesired = hold;             // (the elbow limit follows the elbow's asked-for angle)
      }
      Serial.printf("Servo overload on supply %d: sag %.0f mV, servos stopped\n", r, sag);
    } else if (sag >= sagSlowMv) {
      if (railSagSince[r] == 0) railSagSince[r] = now;
      if (now - railSagSince[r] >= 100) { railState[r] = 1; railSpeedScale[r] = 0.5f; }
    } else {
      railSagSince[r] = 0; railState[r] = 0; railSpeedScale[r] = 1.0f;
    }
  }
}

// Sends "armV,gripperV,armSagMv,gripperSagMv,armState,gripperState,...;limit message" (V = -1: no divider wired; state 0 ok, 1 slowed, 2 stopped).
void notifyServoStatus() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 200) return;
  last = now;
  char buf[200];
  snprintf(buf, sizeof(buf), "%.2f,%.2f,%.0f,%.0f,%d,%d,%.0f,%.0f,%.2f,%.2f",
           railPresent[0] ? railV[0] / 1000.0f : -1.0f, railPresent[1] ? railV[1] / 1000.0f : -1.0f,
           railPresent[0] ? railSagMax[0] : 0.0f, railPresent[1] ? railSagMax[1] : 0.0f,
           railState[0], railState[1], railNoise[0], railNoise[1],
           railPresent[0] ? railBase[0] / 1000.0f : -1.0f, railPresent[1] ? railBase[1] / 1000.0f : -1.0f);
  size_t at = strlen(buf);                                   // the angle the elbow limit is holding the elbow at (-1 = no limit acting)
  snprintf(buf + at, sizeof(buf) - at, ",%d", elbowHeldAngle());
  char msg[110];
  elbowLimitMessage(msg, sizeof(msg));                      // a limit message (or nothing) goes after the ';'
  size_t used = strlen(buf);
  snprintf(buf + used, sizeof(buf) - used, ";%s", msg);
  railNoise[0] = railNoise[1] = 0;  // start a fresh window
  railSagMax[0] = railSagMax[1] = 0;
  if (deviceConnected && pServoStatusCharacteristic) {
    pServoStatusCharacteristic->setValue((uint8_t*)buf, strlen(buf));
    pServoStatusCharacteristic->notify();
  }
}

// ---- Straight-drive assist --------------------------------------------------------------------------------
// While the car drives straight forward or backward, a PI controller lowers the power of the side that is ahead so
// the heading stays where it was when the drive started. The heading error comes from the GYRO when the MPU is
// working (it does not depend on the encoders being right or on which wire is on which pin), otherwise from the
// difference in encoder ticks between the two sides. What it learns about the motor imbalance is kept for the next
// drive in the same direction. Tuned in a simulation with wheels that differ by 10-25%.
const float STRAIGHT_DEG_PER_TICK = 1.95f;        // heading change caused by 1 tick of left-right difference (about 15 cm track)
const float STRAIGHT_KP = 0.04f;                  // power trim per tick of left-right difference
const float STRAIGHT_KI = 0.05f;                  // power trim per (tick * second)
const float STRAIGHT_TRIM_MAX = 0.35f;            // never reduce one side by more than 35%
const unsigned long STRAIGHT_PERIOD_MS = 50;      // control rate (20 Hz)
const unsigned long STRAIGHT_SETTLE_MS = 250;     // let the car start moving before correcting
bool straightActive = false;
int straightDir = 0;
float straightTrimI = 0, straightTrimFwd = 0, straightTrimRev = 0;
long straightStartL = 0, straightStartR = 0;
float straightStartYaw = 0;
unsigned long straightStartMs = 0, lastStraightMs = 0;

// trim > 0: left side is ahead, reduce it. trim < 0: right side is ahead, reduce it.
void setStraightScales(float trim) {
  float newLeft = trim > 0 ? 1.0f - trim : 1.0f;
  float newRight = trim < 0 ? 1.0f + trim : 1.0f;
  if (newLeft == leftScale && newRight == rightScale) return;
  leftScale = newLeft;
  rightScale = newRight;
  motors(motorState[0], motorState[1], motorState[2], motorState[3]);  // apply to wheels that are running now
}

void updateStraightAssist() {
  unsigned long now = millis();
  if (now - lastStraightMs < STRAIGHT_PERIOD_MS) return;
  float dt = (now - lastStraightMs) / 1000.0f;
  lastStraightMs = now;

  int dir = 0;  // +1 straight forward, -1 straight backward, 0 = stopped or turning
  if (motorState[0] && motorState[2] && !motorState[1] && !motorState[3]) dir = 1;
  else if (motorState[1] && motorState[3] && !motorState[0] && !motorState[2]) dir = -1;

  if (straightMode == 0 || dir == 0) {
    if (straightActive) {  // drive ended: remember what was learned
      if (straightDir > 0) straightTrimFwd = straightTrimI; else straightTrimRev = straightTrimI;
      straightActive = false;
    }
    setStraightScales(0.0f);  // both sides at full power again
    return;
  }

  if (!straightActive || straightDir != dir) {  // a new straight drive starts
    if (straightActive) {
      if (straightDir > 0) straightTrimFwd = straightTrimI; else straightTrimRev = straightTrimI;
    }
    straightActive = true;
    straightDir = dir;
    noInterrupts();
    straightStartL = encoderLeftTicks;
    straightStartR = encoderRightTicks;
    interrupts();
    straightStartMs = now;
    straightStartYaw = mpuYaw;
    straightTrimI = (dir > 0) ? straightTrimFwd : straightTrimRev;  // start with what was learned before
    setStraightScales(straightTrimI);
    return;
  }

  if (now - straightStartMs < STRAIGHT_SETTLE_MS) return;

  // Heading error in "ticks of left-right difference": + = the left side is ahead (the car turned right).
  bool wantGyro = (straightMode & 1) && mpuOk;
  bool wantEnc = (straightMode & 2) || ((straightMode & 1) && !mpuOk);  // encoders also back up a missing gyro
  float eGyro = 0, eEnc = 0;
  bool haveGyro = false, haveEnc = false;
  if (wantGyro) {
    float yawErr = mpuYaw - straightStartYaw;       // + = turned left (counter-clockwise), so the right side is ahead
    while (yawErr > 180.0f) yawErr -= 360.0f;
    while (yawErr < -180.0f) yawErr += 360.0f;
    // Forward: turned left = the right side is ahead. Backward it is the other way round (turned left = the LEFT side is
    // ahead), so the sign flips with the driving direction; without this the correction made a reverse drive veer more.
    eGyro = -yawErr / STRAIGHT_DEG_PER_TICK * straightDir;
    haveGyro = true;
  }
  if (wantEnc) {
    noInterrupts();
    long l = encoderLeftTicks;
    long r = encoderRightTicks;
    interrupts();
    long dl = labs(l - straightStartL);
    long dr = labs(r - straightStartR);
    long hi = dl > dr ? dl : dr, lo = dl > dr ? dr : dl;
    bool moving = (dl + dr) >= 6;
    bool trusted = !(hi > 20 && lo * 5 < hi);       // one encoder reporting almost nothing: don't trust it
    if (moving && trusted) {
      eEnc = (float)(dl - dr);                      // + = the left side has gone farther
      haveEnc = true;
    }
  }
  float e;
  if (haveGyro && haveEnc) e = 0.5f * (eGyro + eEnc);
  else if (haveGyro) e = eGyro;
  else if (haveEnc) e = eEnc;
  else return;                                      // nothing reliable to steer by yet
  straightTrimI += STRAIGHT_KI * e * dt;
  if (straightTrimI > STRAIGHT_TRIM_MAX) straightTrimI = STRAIGHT_TRIM_MAX;
  if (straightTrimI < -STRAIGHT_TRIM_MAX) straightTrimI = -STRAIGHT_TRIM_MAX;
  float trim = STRAIGHT_KP * e + straightTrimI;
  if (trim > STRAIGHT_TRIM_MAX) trim = STRAIGHT_TRIM_MAX;
  if (trim < -STRAIGHT_TRIM_MAX) trim = -STRAIGHT_TRIM_MAX;
  setStraightScales(trim);
}

class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    motors(0, 0, 0, 0);
    Serial.println("BLE Client Disconnected");
    BLEDevice::startAdvertising();
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String received = characteristic->getValue();

    if (received.length() == 0) return;

    handleCommand(received);
  }
};

void setup() {
  Serial.begin(115200);

  // Status LED: show WHY the board started (power-on, brownout, RST button, crash) for a moment.
  esp_reset_reason_t why = esp_reset_reason();
  switch (why) {
    case ESP_RST_POWERON:   statusLed(40, 40, 40); break;  // white   = normal power-on
    case ESP_RST_BROWNOUT:  statusLed(60, 0, 0);   break;  // red     = supply dipped too low
    case ESP_RST_EXT:       statusLed(0, 40, 40);  break;  // cyan    = RST button / external reset
    case ESP_RST_SW:        statusLed(40, 20, 0);  break;  // orange  = software restart
    default:                statusLed(50, 0, 50);  break;  // magenta = crash / watchdog / other
  }
  delay(1200);

  // After a brownout/crash: flash YELLOW once per start-up step the previous boot had completed (see markStage),
  // so you can see where it died. One long violet flash = it died before step 1.
  if (why != ESP_RST_POWERON && why != ESP_RST_EXT && why != ESP_RST_SW) {
    uint8_t lastStep = (bootMagic == BOOT_MAGIC) ? bootStage : 0;
    Serial.printf("Previous start died after step %d\n", lastStep);
    if (lastStep == 0) {
      statusLed(40, 0, 60); delay(800); statusLed(0, 0, 0); delay(300);
    }
    for (int i = 0; i < lastStep; i++) {
      statusLed(60, 45, 0); delay(250); statusLed(0, 0, 0); delay(250);
    }
    delay(500);
  }
  markStage(0);
  statusLed(0, 0, 40);                                      // blue    = starting up

  // Wheel PWM (1 kHz, 8-bit). Uses 4 of the 8 LEDC channels; the 4 servos use the other 4.
  for (int i = 0; i < 4; i++) {
    if (!ledcAttach(MOTOR_PINS[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS)) {
      Serial.printf("ERROR: could not attach motor pin %d\n", MOTOR_PINS[i]);
    }
  }
  motors(0, 0, 0, 0);
  markStage(1);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  markStage(2);

  // Wheel encoders: D0 pulses HIGH once per slot.
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), encoderLeftIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), encoderRightIsr, RISING);
  markStage(3);

  analogSetPinAttenuation(RAIL_PINS[0], ADC_11db);  // servo supply dividers: read up to about 3.1 V
  analogSetPinAttenuation(RAIL_PINS[1], ADC_11db);

  mpuOk = mpuInit();
  markStage(4);
  lastMpuMicros = micros();
  Serial.println(mpuOk ? "MPU-6050 ready (keep the car still while it calibrates)" : "ERROR: MPU-6050 not found (check SDA/SCL/power)");

  // Servo signals are not started here: each servo starts when it is first commanded (see attachServo).

  BLEDevice::init("AI_Car");
  markStage(5);
  // Default BLE transmit power on purpose: a stronger radio draws bigger current bursts, and this board's 3V3 rail
  // (shared with the MPU, ultrasonic sensor and encoders) is close to its limit at power-on.
  BLEDevice::setMTU(185);  // MPU notifications are longer than the default 20 bytes

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService =
    pServer->createService("12345678-1234-1234-1234-1234567890AB");

  pCharacteristic = pService->createCharacteristic(
    "87654321-4321-4321-4321-BA0987654321",
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );

  pCharacteristic->setCallbacks(new CharacteristicCallbacks());

  // Distance (cm, ASCII text, "-1" = no echo) notified to the client.
  pDistanceCharacteristic = pService->createCharacteristic(
    DISTANCE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pDistanceCharacteristic->addDescriptor(new BLE2902());

  // MPU-6050 data, ASCII "pitch,roll,ax,ay,az,gx,gy,gz" ("NA" = sensor not found).
  pMpuCharacteristic = pService->createCharacteristic(
    MPU_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pMpuCharacteristic->addDescriptor(new BLE2902());

  // Wheel encoders, ASCII "leftTicks,rightTicks,leftCm,rightCm,leftCm/s,rightCm/s".
  pEncoderCharacteristic = pService->createCharacteristic(
    ENCODER_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pEncoderCharacteristic->addDescriptor(new BLE2902());

  // Servo supply voltages and overload state, ASCII "armV,gripperV,armSagMv,gripperSagMv,armState,gripperState".
  pServoStatusCharacteristic = pService->createCharacteristic(
    SERVO_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pServoStatusCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  markStage(6);

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID("12345678-1234-1234-1234-1234567890AB");
  pAdvertising->start();
  markStage(7);

  Serial.println("AI_Car BLE Ready");
  Serial.println("Motor commands: U, D, L, R, S");
  Serial.println("Servo commands: V<0-3> <angle 0-180 (servo 0: 0-120)> <speed 1-600 deg/sec>, e.g. V0 90 60");
}

// Status light: steady green = a client is connected, slow green blink = advertising (waiting),
// fast red blink = nobody connected and NOT advertising (the car is invisible).
void updateStatusLed() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate < 100) return;
  lastUpdate = now;
  if (deviceConnected) {
    statusLed(0, 40, 0);
  } else if (BLEDevice::getAdvertising()->isAdvertising()) {
    statusLed(0, (now / 500) % 2 ? 40 : 0, 0);
  } else {
    statusLed((now / 150) % 2 ? 60 : 0, 0, 0);
  }
}

// Safety net: if nobody is connected the car must be advertising, otherwise it is invisible until a reset.
void ensureAdvertising() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck < 3000) return;
  lastCheck = now;
  if (!deviceConnected && !BLEDevice::getAdvertising()->isAdvertising()) {
    BLEDevice::startAdvertising();
    Serial.println("Advertising had stopped: restarted");
  }
}

void loop() {
  static bool loopMarked = false;
  if (!loopMarked) { markStage(8); loopMarked = true; }
  ensureAdvertising();
  updateStatusLed();
  updateServos();
  updateDistance();
  updateBuzzer();
  updateMpu();
  notifyMpu();
  updateEncoders();
  updateStraightAssist();
  updateServoGuard();
  notifyServoStatus();
  delay(2);
}
