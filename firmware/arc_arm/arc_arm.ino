#include <ESP32Servo.h>
#include <Keypad.h>

// ===================================
// НАЛАШТУВАННЯ
// ===================================

// Піни
const int PIN_BASE = 18;      // серво під низом, поворот
const int PIN_SHOULDER = 19;  // серво збоку, підйом руки
const int PIN_ELBOW = 21;     // серво з іншого боку, виліт руки
const int PIN_CLAW = 22;      // серво на клешні
const int PIN_JOY_Y = 35;
const int PIN_JOY_SW = 23;
byte rowPins[4] = {13, 14, 16, 17};
byte colPins[4] = {25, 26, 27, 32};

// Стартові кути = як рука стоїть при вимкненому живленні.
// Якщо при ввімкненні якийсь суглоб смикається, змініть його START.
// Клавіша 0 плавно повертає руку в цю позу.
const float BASE_START = 0;
const float SHOULDER_START = 0;
const float ELBOW_START = 180;
const float CLAW_START = 90;

// Межі кутів: звузьте, якщо серво впирається й гуде на краю
const float BASE_MIN = 0, BASE_MAX = 180;
const float SHOULDER_MIN = 0, SHOULDER_MAX = 180;
const float ELBOW_MIN = 0, ELBOW_MAX = 180;
const float CLAW_MIN = 10, CLAW_MAX = 170;

// Плавність
const float MAX_SPEED = 60;     // град/с при повністю відхиленому джойстику
const float CLAW_SPEED = 90;    // град/с для клешні
const float ACCEL = 150;        // град/с^2: розгін і гальмування
const float HOME_GAIN = 2.0;    // як м'яко рука під'їжджає в стартову позу

// Джойстик
const float DEAD_ZONE = 0.2;    // частка ходу ручки біля центру
const bool INVERT_Y = false;    // true, якщо «вгору» рухає не туди

const unsigned long TICK_MS = 20;

// ===================================
// СТАН
// ===================================

struct Joint {
  const char *name;
  int pin;
  float angle;
  float vel;
  float minA;
  float maxA;
  float start;
  float maxSpeed;
  Servo servo;
};

const int JOINTS = 4;
Joint joints[JOINTS] = {
  {"base", PIN_BASE, BASE_START, 0, BASE_MIN, BASE_MAX, BASE_START, MAX_SPEED},
  {"shoulder", PIN_SHOULDER, SHOULDER_START, 0, SHOULDER_MIN, SHOULDER_MAX, SHOULDER_START, MAX_SPEED},
  {"elbow", PIN_ELBOW, ELBOW_START, 0, ELBOW_MIN, ELBOW_MAX, ELBOW_START, MAX_SPEED},
  {"claw", PIN_CLAW, CLAW_START, 0, CLAW_MIN, CLAW_MAX, CLAW_START, CLAW_SPEED},
};

int selected = -1;
bool homing = false;
int joyCenter = 2048;
float joyFiltered = 0;

char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);

// ===================================
// ДОПОМІЖНІ
// ===================================

int angleToUs(float a) {
  return 500 + (int)(a * 2000.0f / 180.0f + 0.5f);
}

float approach(float value, float target, float step) {
  if (value < target) return min(value + step, target);
  return max(value - step, target);
}

void calibrateJoystick() {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(PIN_JOY_Y);
    delay(2);
  }
  int c = sum / 32;
  joyCenter = (c > 1000 && c < 3100) ? c : 2048;
  Serial.print("Joystick center: ");
  Serial.println(joyCenter);
}

float readJoystick() {
  int raw = analogRead(PIN_JOY_Y);
  float span = raw >= joyCenter ? 4095 - joyCenter : joyCenter;
  float v = constrain((raw - joyCenter) / span, -1.0f, 1.0f);
  if (INVERT_Y) v = -v;
  joyFiltered += 0.4f * (v - joyFiltered);
  float m = fabsf(joyFiltered);
  if (m < DEAD_ZONE) return 0;
  float s = (m - DEAD_ZONE) / (1 - DEAD_ZONE);
  return copysignf(s * s, joyFiltered);
}

// Швидкість змінюється не різкіше ACCEL, а біля межі заздалегідь гальмує
void moveJoint(Joint &j, float targetVel, float dt) {
  float room = targetVel > 0 ? j.maxA - j.angle : j.angle - j.minA;
  float vBrake = sqrtf(2 * ACCEL * max(room, 0.0f));
  if (fabsf(targetVel) > vBrake) targetVel = copysignf(vBrake, targetVel);
  j.vel = approach(j.vel, targetVel, ACCEL * dt);
  j.angle += j.vel * dt;
  if (j.angle <= j.minA || j.angle >= j.maxA) {
    j.angle = constrain(j.angle, j.minA, j.maxA);
    j.vel = 0;
  }
  j.servo.writeMicroseconds(angleToUs(j.angle));
}

void selectJoint(int i) {
  selected = i;
  homing = false;
  Serial.print("Selected: ");
  Serial.print(i + 1);
  Serial.print(" - ");
  Serial.println(joints[i].name);
}

bool joyButtonPressed() {
  static bool last = HIGH;
  static bool stable = HIGH;
  static unsigned long changed = 0;
  bool level = digitalRead(PIN_JOY_SW);
  if (level != last) {
    last = level;
    changed = millis();
  }
  if (millis() - changed > 30 && level != stable) {
    stable = level;
    return stable == LOW;
  }
  return false;
}

// ===================================
// SETUP
// ===================================

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  // По черзі, щоб не було кидка струму від усіх серво одразу
  for (int i = 0; i < JOINTS; i++) {
    Joint &j = joints[i];
    j.angle = constrain(j.start, j.minA, j.maxA);
    j.servo.setPeriodHertz(50);
    j.servo.attach(j.pin, 500, 2500);
    j.servo.writeMicroseconds(angleToUs(j.angle));
    delay(300);
  }

  calibrateJoystick();

  Serial.println("ESP32 Robot Arm Ready");
  Serial.println("1 - base | 2 - shoulder | 3 - elbow | 4 - claw | 0 - home | joystick button - next");
}

// ===================================
// LOOP
// ===================================

void loop() {
  char key = keypad.getKey();
  if (key >= '1' && key <= '4') selectJoint(key - '1');
  else if (key == '0') {
    homing = true;
    Serial.println("Home");
  }

  if (joyButtonPressed()) selectJoint((selected + 1) % JOINTS);

  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  float dt = min((now - lastTick) / 1000.0f, 0.1f);
  lastTick = now;

  float cmd = readJoystick();
  if (cmd != 0) homing = false;

  bool atHome = true;
  for (int i = 0; i < JOINTS; i++) {
    Joint &j = joints[i];
    float targetVel = 0;
    if (homing) {
      targetVel = constrain((j.start - j.angle) * HOME_GAIN, -j.maxSpeed, j.maxSpeed);
      if (fabsf(j.start - j.angle) > 0.5f) atHome = false;
    } else if (i == selected) {
      targetVel = cmd * j.maxSpeed;
    }
    moveJoint(j, targetVel, dt);
  }
  if (homing && atHome) homing = false;

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("Sel: ");
    Serial.print(selected + 1);
    Serial.print(" | Joy: ");
    Serial.print(cmd, 2);
    for (int i = 0; i < JOINTS; i++) {
      Serial.print(" | ");
      Serial.print(joints[i].name);
      Serial.print(": ");
      Serial.print(joints[i].angle, 0);
    }
    Serial.println();
  }
}
