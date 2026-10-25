#include <ESP32Servo.h>
#include <Keypad.h>

// ===================================
// НАЛАШТУВАННЯ
// ===================================

const int SPEED = 40;        // град/с: швидкість суглобів (менше = плавніше)
const int CLAW_STOP = 1500;  // мкс: клешня стоїть; якщо повзе сама — підібрати 1480–1520
const int CLAW_SPEED = 100;  // швидкість клешні

struct Joint {
  int pin;
  float start;  // кут при ввімкненні = як рука стоїть без живлення
  float minA;
  float maxA;
  float angle;
  float vel;
  Servo servo;
};

Joint joints[3] = {
  {18, 0, 0, 180},    // 1 — база
  {19, 0, 0, 180},    // 2 — плече
  {21, 180, 0, 180},  // 3 — лікоть
};

const int CLAW_PIN = 22;     // 4 — клешня (серво 360°)
const int JOY_Y = 35;
const int JOY_SW = 23;
byte rowPins[4] = {13, 14, 16, 17};
byte colPins[4] = {25, 26, 27, 32};

// ===================================

const float ACCEL = SPEED * 3.0f;
const float DEAD_ZONE = 0.2f;

Servo claw;
float clawUs = CLAW_STOP;
int selected = 0;
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

int angleToUs(float a) {
  return 500 + (int)(a * 2000.0f / 180.0f + 0.5f);
}

float toward(float value, float target, float step) {
  if (value < target) return min(value + step, target);
  return max(value - step, target);
}

float readJoystick() {
  int raw = analogRead(JOY_Y);
  float span = raw >= joyCenter ? 4095 - joyCenter : joyCenter;
  float v = constrain((raw - joyCenter) / span, -1.0f, 1.0f);
  joyFiltered += 0.3f * (v - joyFiltered);
  float m = fabsf(joyFiltered);
  if (m < DEAD_ZONE) return 0;
  float s = (m - DEAD_ZONE) / (1 - DEAD_ZONE);
  return copysignf(s * s, joyFiltered);
}

void moveJoint(Joint &j, float targetVel, float dt) {
  float room = targetVel > 0 ? j.maxA - j.angle : j.angle - j.minA;
  float vBrake = sqrtf(2 * ACCEL * max(room, 0.0f));
  if (fabsf(targetVel) > vBrake) targetVel = copysignf(vBrake, targetVel);
  j.vel = toward(j.vel, targetVel, ACCEL * dt);
  j.angle = constrain(j.angle + j.vel * dt, j.minA, j.maxA);
  j.servo.writeMicroseconds(angleToUs(j.angle));
}

void selectJoint(int s) {
  selected = s;
  homing = false;
  Serial.print("Selected: ");
  Serial.println(s);
}

bool joyButtonPressed() {
  static bool last = HIGH, stable = HIGH;
  static unsigned long changed = 0;
  bool level = digitalRead(JOY_SW);
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

void setup() {
  Serial.begin(9600);
  delay(500);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  claw.setPeriodHertz(50);
  claw.attach(CLAW_PIN, 1000, 2000);
  claw.writeMicroseconds(CLAW_STOP);

  for (Joint &j : joints) {
    j.angle = j.start;
    j.servo.setPeriodHertz(50);
    j.servo.attach(j.pin, 500, 2500);
    j.servo.writeMicroseconds(angleToUs(j.angle));
    delay(300);
  }

  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogRead(JOY_Y);
  int c = sum / 32;
  if (c > 1000 && c < 3100) joyCenter = c;

  Serial.println("Ready: 1 base | 2 shoulder | 3 elbow | 4 claw | 0 home | joystick button - next");
}

void loop() {
  char key = keypad.getKey();
  if (key >= '1' && key <= '4') selectJoint(key - '0');
  if (key == '0') homing = true;
  if (joyButtonPressed()) selectJoint(selected % 4 + 1);

  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (now - lastTick < 20) return;
  float dt = min((now - lastTick) / 1000.0f, 0.1f);
  lastTick = now;

  float cmd = readJoystick();
  if (cmd != 0) homing = false;

  bool atHome = true;
  for (int i = 0; i < 3; i++) {
    Joint &j = joints[i];
    float v = 0;
    if (homing) {
      v = constrain((j.start - j.angle) * 2, -(float)SPEED, (float)SPEED);
      if (fabsf(j.start - j.angle) > 0.5f) atHome = false;
    } else if (selected == i + 1) {
      v = cmd * SPEED;
    }
    moveJoint(j, v, dt);
  }
  if (atHome) homing = false;

  float clawTarget = (selected == 4 && !homing) ? CLAW_STOP + cmd * CLAW_SPEED : CLAW_STOP;
  clawUs = toward(clawUs, clawTarget, CLAW_SPEED * 4 * dt);
  claw.writeMicroseconds((int)(clawUs + 0.5f));

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("Sel %d | joy %.2f | base %.0f | shoulder %.0f | elbow %.0f | claw %d\n", selected, cmd,
                  joints[0].angle, joints[1].angle, joints[2].angle, (int)clawUs);
  }
}
