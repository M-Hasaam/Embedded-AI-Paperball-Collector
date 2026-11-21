#include <Servo.h>

// Robotic arm on an Arduino Uno, driven from servo_control.py over USB serial.
// Command (one line):  <servo> <angle> [speed]     e.g.  "0 90 60"
//   servo  0 = Gripper, 1 = Elbow, 2 = Shoulder, 3 = Servo 4
//   angle  0-180, clamped to that servo's limits below
//   speed  1-600 degrees per second (optional; the servo keeps its last speed, 60 at first)

const int NUM_SERVOS = 4;
// Signal pins, in servo order: gripper 6, elbow 9, shoulder 10, servo 4 on pin 11.
const int SERVO_PINS[NUM_SERVOS] = {6, 9, 10, 11};

// Angle limits per servo (keep in sync with SERVO_MIN_ANGLES / SERVO_MAX_ANGLES in servo_control.py).
const int SERVO_ANGLE_MIN[NUM_SERVOS] = {0, 0, 0, 0};
const int SERVO_ANGLE_MAX[NUM_SERVOS] = {120, 180, 180, 180};   // the gripper only opens to 120

const int SPEED_DEFAULT = 60;
const int SPEED_MAX = 600;

Servo servos[NUM_SERVOS];
bool attached[NUM_SERVOS] = {false, false, false, false};  // a servo gets no signal until it is first commanded
float currentAngle[NUM_SERVOS] = {90, 90, 90, 90};
int targetAngle[NUM_SERVOS] = {90, 90, 90, 90};
int speedDegPerSec[NUM_SERVOS] = {SPEED_DEFAULT, SPEED_DEFAULT, SPEED_DEFAULT, SPEED_DEFAULT};
int lastWritten[NUM_SERVOS] = {-1, -1, -1, -1};
unsigned long lastUpdate[NUM_SERVOS] = {0, 0, 0, 0};

String inputBuffer = "";

void setup() {
  Serial.begin(9600);
  inputBuffer.reserve(32);
  // Nothing is attached here, so the servos stay limp and do not jump when the board powers up or resets.
}

void commandServo(int index, int angle, int speed) {
  if (angle < SERVO_ANGLE_MIN[index]) angle = SERVO_ANGLE_MIN[index];
  if (angle > SERVO_ANGLE_MAX[index]) angle = SERVO_ANGLE_MAX[index];
  if (!attached[index]) {
    // First command for this servo: start its signal right at the commanded angle.
    servos[index].attach(SERVO_PINS[index]);
    servos[index].write(angle);
    attached[index] = true;
    currentAngle[index] = angle;
    lastWritten[index] = angle;
    lastUpdate[index] = millis();
  }
  targetAngle[index] = angle;
  if (speed >= 1) speedDegPerSec[index] = speed > SPEED_MAX ? SPEED_MAX : speed;
}

// Moves every servo toward its target at its own speed (degrees per second).
void updateServos() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!attached[i]) continue;
    unsigned long dtMs = now - lastUpdate[i];
    lastUpdate[i] = now;
    if (dtMs > 100) dtMs = 100;
    float step = speedDegPerSec[i] * dtMs / 1000.0f;
    float diff = targetAngle[i] - currentAngle[i];
    if (diff > step) currentAngle[i] += step;
    else if (diff < -step) currentAngle[i] -= step;
    else currentAngle[i] = targetAngle[i];
    int angle = (int)(currentAngle[i] + 0.5f);
    if (angle != lastWritten[i]) {
      servos[i].write(angle);
      lastWritten[i] = angle;
    }
  }
}

void loop() {
  // Read incoming characters until a newline is reached
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        int servoIndex = -1;
        int angle = -1;
        int speed = 0;
        int fields = sscanf(inputBuffer.c_str(), "%d %d %d", &servoIndex, &angle, &speed);
        if (fields >= 2 && servoIndex >= 0 && servoIndex < NUM_SERVOS && angle >= 0 && angle <= 180) {
          commandServo(servoIndex, angle, fields == 3 ? speed : 0);
        }
      }
      inputBuffer = "";
    } else if (c != '\r') {
      if (inputBuffer.length() < 30) inputBuffer += c;
    }
  }
  updateServos();
}
