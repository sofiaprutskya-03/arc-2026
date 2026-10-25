#include <ESP32Servo.h>
#include <Keypad.h>

// ===================================
// НАЛАШТУВАННЯ
// ===================================

// Піни
const int servoPin1 = 18;   // база
const int servoPin2 = 19;   // рука, серво A
const int servoPin3 = 21;   // рука, серво B (дзеркально до A)
const int servoPin4 = 22;   // клешня, серво безперервного обертання
const int joyY = 35;
const int joySW = 23;
byte rowPins[4] = {13, 14, 16, 17};
byte colPins[4] = {25, 26, 27, 32};

// Стартова поза (в цю позу рука стає при ввімкненні та по клавіші 0)
const float BASE_START = 90;
const float ARM_START = 90;

// Межі кутів: звузьте, якщо серво впирається й гуде на краю
const float BASE_MIN = 10, BASE_MAX = 170;
const float ARM_MIN = 10, ARM_MAX = 170;

// Плавність
const float MAX_SPEED = 60;     // град/с при повністю відхиленому джойстику
const float ACCEL = 150;        // град/с^2: розгін і гальмування
const float HOME_GAIN = 2.0;    // як м'яко рука під'їжджає додому

// Клешня (серво безперервного обертання)
const int GRIP_STOP_US = 1500;  // підберіть, якщо клешня повзе сама
const int GRIP_MAX_DELTA = 100; // максимальна швидкість клешні, мкс від STOP
const float GRIP_RAMP = 400;    // мкс/с: плавний старт і стоп клешні

// Джойстик
const float DEAD_ZONE = 0.2;    // частка ходу ручки біля центру
const bool INVERT_Y = false;    // true, якщо «вгору» рухає не туди

const unsigned long TICK_MS = 20;

// ===================================
// СТАН
// ===================================

Servo servo1, servo2, servo3, servo4;

struct Joint {
  float angle;
  float vel;
  float minA;
  float maxA;
};

Joint base = {BASE_START, 0, BASE_MIN, BASE_MAX};
Joint arm = {ARM_START, 0, ARM_MIN, ARM_MAX};
float gripUs = GRIP_STOP_US;

int selectedServo = 0;
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

void writeArm() {
  servo1.writeMicroseconds(angleToUs(base.angle));
  servo2.writeMicroseconds(angleToUs(arm.angle));
  servo3.writeMicroseconds(angleToUs(180 - arm.angle));
}

float approach(float value, float target, float step) {
  if (value < target) return min(value + step, target);
  return max(value - step, target);
}

void calibrateJoystick() {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(joyY);
    delay(2);
  }
  int c = sum / 32;
  joyCenter = (c > 1000 && c < 3100) ? c : 2048;
  Serial.print("Joystick center: ");
  Serial.println(joyCenter);
}

float readJoystick() {
  int raw = analogRead(joyY);
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
}

float homeVel(const Joint &j, float goal) {
  return constrain((goal - j.angle) * HOME_GAIN, -MAX_SPEED, MAX_SPEED);
}

void selectServo(int s) {
  selectedServo = s;
  homing = false;
  Serial.print("Selected: ");
  Serial.println(s == 1 ? "1 - base" : s == 2 ? "2 - arm" : "3 - gripper");
}

bool joyButtonPressed() {
  static bool last = HIGH;
  static unsigned long changed = 0;
  static bool stable = HIGH;
  bool level = digitalRead(joySW);
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

  pinMode(joySW, INPUT_PULLUP);
  analogReadResolution(12);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  // По черзі, щоб не було кидка струму від усіх серво одразу
  servo4.attach(servoPin4, 1000, 2000);
  servo4.writeMicroseconds(GRIP_STOP_US);
  delay(200);
  servo1.attach(servoPin1, 500, 2500);
  servo1.writeMicroseconds(angleToUs(base.angle));
  delay(300);
  servo2.attach(servoPin2, 500, 2500);
  servo3.attach(servoPin3, 500, 2500);
  writeArm();
  delay(300);

  calibrateJoystick();

  Serial.println("ESP32 Robot Arm Ready");
  Serial.println("1 - base | 2 - arm | 3 - gripper | 0 - home | joystick button - next");
}

// ===================================
// LOOP
// ===================================

void loop() {
  char key = keypad.getKey();
  if (key == '1') selectServo(1);
  else if (key == '2') selectServo(2);
  else if (key == '3') selectServo(3);
  else if (key == '0') {
    homing = true;
    Serial.println("Home");
  }

  if (joyButtonPressed()) selectServo(selectedServo % 3 + 1);

  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  float dt = min((now - lastTick) / 1000.0f, 0.1f);
  lastTick = now;

  float cmd = readJoystick();
  if (cmd != 0) homing = false;

  float baseTarget = 0, armTarget = 0;
  if (homing) {
    baseTarget = homeVel(base, BASE_START);
    armTarget = homeVel(arm, ARM_START);
    if (fabsf(base.angle - BASE_START) < 0.5f && fabsf(arm.angle - ARM_START) < 0.5f) homing = false;
  } else if (selectedServo == 1) {
    baseTarget = cmd * MAX_SPEED;
  } else if (selectedServo == 2) {
    armTarget = cmd * MAX_SPEED;
  }

  moveJoint(base, baseTarget, dt);
  moveJoint(arm, armTarget, dt);
  writeArm();

  float gripTarget = GRIP_STOP_US;
  if (selectedServo == 3 && !homing) gripTarget += cmd * GRIP_MAX_DELTA;
  gripUs = approach(gripUs, gripTarget, GRIP_RAMP * dt);
  servo4.writeMicroseconds((int)(gripUs + 0.5f));

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.print("Sel: ");
    Serial.print(selectedServo);
    Serial.print(" | Joy: ");
    Serial.print(cmd, 2);
    Serial.print(" | Base: ");
    Serial.print(base.angle, 1);
    Serial.print(" | Arm: ");
    Serial.print(arm.angle, 1);
    Serial.print(" | Grip us: ");
    Serial.println((int)gripUs);
  }
}
