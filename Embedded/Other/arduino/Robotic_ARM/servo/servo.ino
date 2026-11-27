#include <Servo.h>
#include <avr/pgmspace.h>

// Robotic arm on an Arduino Uno, driven from servo_control.py over USB serial.
// Command (one line):  <servo> <angle> [speed]     e.g.  "0 90 60"
//   servo  0 = Gripper, 1 = Elbow, 2 = Shoulder, 3 = Servo 4
//   angle  0-180, clamped to that servo's limits below
//   speed  1-600 degrees per second (optional; the servo keeps its last speed, 60 at first)
// Replies:  "OK <servo> <target> <speed>" after each command, and "LIM <held angle or -1> <message>" whenever the
//           elbow limit (below) changes or once a second.

const int NUM_SERVOS = 4;
// Signal pins, in servo order: gripper 6, elbow 9, shoulder 10, servo 4 on pin 11.
const int SERVO_PINS[NUM_SERVOS] = {6, 9, 10, 11};

// Angle limits per servo (keep in sync with SERVO_MIN_ANGLES / SERVO_MAX_ANGLES in servo_control.py and with AI_Car.ino).
const int SERVO_ANGLE_MIN[NUM_SERVOS] = {0, 80, 40, 0};         // elbow 80, shoulder 40
const int SERVO_ANGLE_MAX[NUM_SERVOS] = {120, 180, 170, 180};   // gripper 120, shoulder 170

const int SPEED_DEFAULT = 60;
const int SPEED_MAX = 600;

Servo servos[NUM_SERVOS];
bool attached[NUM_SERVOS] = {false, false, false, false};  // a servo gets no signal until it is first commanded
float currentAngle[NUM_SERVOS] = {90, 90, 90, 90};
int targetAngle[NUM_SERVOS] = {90, 90, 90, 90};
int speedDegPerSec[NUM_SERVOS] = {SPEED_DEFAULT, SPEED_DEFAULT, SPEED_DEFAULT, SPEED_DEFAULT};
int lastWritten[NUM_SERVOS] = {-1, -1, -1, -1};
unsigned long lastUpdate[NUM_SERVOS] = {0, 0, 0, 0};

// ---- Elbow limit that depends on the shoulder angle (the same table as AI_Car.ino; plotted in docs/arm-limits) ----
// The elbow can break outside this window for the shoulder's angle. Between two rows the TIGHTER of the two is used, and
// LIMIT_MARGIN can keep the elbow further inside every limit tighter than the servo's own 80..180 range; it is 0.
const int ELBOW = 1, SHOULDER = 2;
const int LIMIT_ROWS = 14;
const byte LIMIT_SHOULDER[LIMIT_ROWS]  = {  40,  60,  70,  80,  90, 100, 110, 115, 120, 130, 140, 150, 160, 170};
const byte LIMIT_ELBOW_MIN[LIMIT_ROWS] = { 135, 120, 105, 100, 100, 100, 100, 100,  95,  95,  95,  90, 100, 120};
const byte LIMIT_ELBOW_MAX[LIMIT_ROWS] = { 180, 180, 180, 180, 180, 180, 180, 180, 180, 170, 170, 160, 150, 145};
const int LIMIT_MARGIN = 0;       // extra degrees kept inside the table's limits (was 3)
const int LIMIT_LOOKAHEAD = 5;    // the elbow moves clear of the next few shoulder degrees ahead of the shoulder
const int SHOULDER_DEFAULT = 90;  // assumed shoulder angle while the shoulder has not started
int elbowDesired = 120;           // where the elbow was last asked to go; it is pulled into the window while it moves
bool elbowWaiting = false;        // the elbow was commanded before the shoulder started: it starts right after it
int elbowWaitSpeed = SPEED_DEFAULT;
bool shoulderHeld = false;        // the shoulder is waiting for the elbow to get out of the way

String inputBuffer = "";

int roundAngle(float a) { return (int)(a + 0.5f); }

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

int shoulderNow() { return attached[SHOULDER] ? roundAngle(currentAngle[SHOULDER]) : SHOULDER_DEFAULT; }

// The window the elbow has to stay in right now: allowed for the shoulder's current angle and for the next few degrees
// it is about to move through. Also returns the shoulder angle it was worked out for.
void elbowWindowNow(int &sNow, int &lo, int &hi) {
  sNow = shoulderNow();
  int sAhead = sNow;
  if (attached[SHOULDER] && targetAngle[SHOULDER] != sNow) {
    int span = abs(targetAngle[SHOULDER] - sNow);
    if (span > LIMIT_LOOKAHEAD) span = LIMIT_LOOKAHEAD;
    sAhead = sNow + (targetAngle[SHOULDER] > sNow ? span : -span);
  }
  elbowWindow(sNow, sAhead, lo, hi);
}

int clampElbowForShoulder(int angle) {
  int lo, hi;
  elbowWindowAt(shoulderNow(), lo, hi);
  return angle < lo ? lo : (angle > hi ? hi : angle);
}

// The angle the elbow limit is holding the elbow at, or -1 while no limit is acting.
int elbowHeldAngle() {
  if (elbowWaiting || !attached[ELBOW]) return -1;
  int sNow, lo, hi;
  elbowWindowNow(sNow, lo, hi);
  return (elbowDesired < lo || elbowDesired > hi) ? targetAngle[ELBOW] : -1;
}

// Text for the controller when a limit is acting (empty when everything is fine).
void elbowLimitMessage(char *out, size_t n) {
  out[0] = 0;
  if (elbowWaiting) {
    snprintf_P(out, n, PSTR("ELBOW waits: start the shoulder first (its limit depends on the shoulder angle)"));
    return;
  }
  if (!attached[ELBOW]) return;
  int sNow, lo, hi;
  elbowWindowNow(sNow, lo, hi);
  if (shoulderHeld) {
    snprintf_P(out, n, PSTR("SHOULDER waits: elbow must move out of the way (allowed %d-%d deg at shoulder %d)"), lo, hi, sNow);
  } else if (elbowDesired < lo || elbowDesired > hi) {
    snprintf_P(out, n, PSTR("ELBOW LIMIT: %d deg asked, held at %d (allowed %d-%d deg at shoulder %d)"),
               elbowDesired, targetAngle[ELBOW], lo, hi, sNow);
  }
}

void setup() {
  Serial.begin(9600);
  inputBuffer.reserve(32);
  // Nothing is attached here, so the servos stay limp and do not jump when the board powers up or resets.
}

// Writes an angle to a servo, clamped to its limits (the elbow also to the window for the shoulder's angle).
void writeAngle(int i, int angle) {
  if (angle < SERVO_ANGLE_MIN[i]) angle = SERVO_ANGLE_MIN[i];
  if (angle > SERVO_ANGLE_MAX[i]) angle = SERVO_ANGLE_MAX[i];
  if (i == ELBOW) angle = clampElbowForShoulder(angle);
  servos[i].write(angle);
  lastWritten[i] = angle;
}

// First command for a servo: start its signal right at that angle.
void attachServo(int i, int angle) {
  if (attached[i]) return;
  if (i == ELBOW) {
    if (!attached[SHOULDER]) { elbowWaiting = true; return; }   // the elbow's limit depends on the shoulder: it starts after it
    angle = clampElbowForShoulder(angle);
  }
  servos[i].attach(SERVO_PINS[i]);
  attached[i] = true;
  currentAngle[i] = angle;
  targetAngle[i] = angle;
  lastUpdate[i] = millis();
  writeAngle(i, angle);
  if (i == SHOULDER && elbowWaiting) {                           // the shoulder is running now: start the elbow that waited
    elbowWaiting = false;
    attachServo(ELBOW, elbowDesired);
    speedDegPerSec[ELBOW] = elbowWaitSpeed;
  }
}

void commandServo(int index, int angle, int speed) {
  if (angle < SERVO_ANGLE_MIN[index]) angle = SERVO_ANGLE_MIN[index];
  if (angle > SERVO_ANGLE_MAX[index]) angle = SERVO_ANGLE_MAX[index];
  int newSpeed = speed >= 1 ? (speed > SPEED_MAX ? SPEED_MAX : speed) : speedDegPerSec[index];
  if (index == ELBOW) elbowDesired = angle;     // updateServos() pulls the elbow into the window for the shoulder's angle
  attachServo(index, angle);
  if (index == ELBOW && !attached[ELBOW]) {      // waiting for the shoulder to start
    elbowWaitSpeed = newSpeed;
    return;
  }
  targetAngle[index] = angle;
  speedDegPerSec[index] = newSpeed;
}

// Moves every servo toward its target at its own speed (degrees per second).
void updateServos() {
  unsigned long now = millis();

  // The elbow's target is where it was asked to go, pulled into the window allowed for the shoulder.
  if (attached[ELBOW]) {
    int sNow, lo, hi;
    elbowWindowNow(sNow, lo, hi);
    targetAngle[ELBOW] = elbowDesired < lo ? lo : (elbowDesired > hi ? hi : elbowDesired);
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!attached[i]) continue;
    unsigned long dtMs = now - lastUpdate[i];
    lastUpdate[i] = now;
    if (dtMs > 100) dtMs = 100;
    float step = speedDegPerSec[i] * dtMs / 1000.0f;
    float diff = targetAngle[i] - currentAngle[i];
    float next;
    if (diff > step) next = currentAngle[i] + step;
    else if (diff < -step) next = currentAngle[i] - step;
    else next = targetAngle[i];

    if (i == SHOULDER && attached[ELBOW] && roundAngle(next) != roundAngle(currentAngle[i])) {
      // the shoulder only moves while the elbow is inside the window for every angle it is about to pass
      int lo, hi;
      elbowWindow(roundAngle(currentAngle[i]), roundAngle(next), lo, hi);
      int e = roundAngle(currentAngle[ELBOW]);
      if (e < lo || e > hi) {
        shoulderHeld = true;        // the elbow has to move out of the way first (it is already being pulled there)
        continue;
      }
      shoulderHeld = false;
    }
    currentAngle[i] = next;
    int angle = roundAngle(currentAngle[i]);
    if (angle != lastWritten[i]) writeAngle(i, angle);
  }
}

// Tells the controller about a limit that is acting: "LIM <held angle or -1> <message>", when it changes and once a second.
void reportLimit() {
  static char last[110] = "";
  static unsigned long lastSent = 0;
  char msg[100];
  elbowLimitMessage(msg, sizeof(msg));
  char line[110];
  snprintf_P(line, sizeof(line), PSTR("LIM %d %s"), elbowHeldAngle(), msg);
  unsigned long now = millis();
  if (strcmp(line, last) != 0 || now - lastSent >= 1000) {
    if (strcmp(line, last) != 0 && now - lastSent < 250) return;   // do not flood the 9600 baud link
    Serial.println(line);
    strcpy(last, line);
    lastSent = now;
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
          // echo what is now in force, so the controller can show it: OK <servo> <target> <speed in force>
          Serial.print("OK "); Serial.print(servoIndex); Serial.print(' '); Serial.print(targetAngle[servoIndex]);
          Serial.print(' '); Serial.println(speedDegPerSec[servoIndex]);
        }
      }
      inputBuffer = "";
    } else if (c != '\r') {
      if (inputBuffer.length() < 30) inputBuffer += c;
    }
  }
  updateServos();
  reportLimit();
}
