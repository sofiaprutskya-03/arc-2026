#include <ESP32Servo.h>
#include <Keypad.h>

// ===================================
// НАЛАШТУВАННЯ
// ===================================

const int SPEED = 40;          // град/с: швидкість суглобів (менше = плавніше)
const bool CLAW_360 = true;    // true: клешня на серво 360° (крутиться), false: звичайне серво на кут
const int CLAW_STOP = 1500;    // для серво 360°: сигнал «стоп», підібрати 1480–1520, якщо повзе

struct Joint {
  const char *name;
  int pin;
  float start;   // кут при ввімкненні = як суглоб стоїть без живлення
  float minA;
  float maxA;
  float angle;
  float vel;
  Servo servo;
};

Joint joints[4] = {
  {"base", 18, 0, 0, 180},
  {"shoulder", 19, 0, 0, 180},
  {"elbow", 21, 180, 0, 180},
  {"claw", 22, 90, 20, 160},
};

const int JOY_X = 34;
const int JOY_Y = 35;
const int JOY_SW = 23;
byte rowPins[4] = {13, 14, 16, 17};
byte colPins[4] = {25, 26, 27, 32};

// ===================================

const int CLAW = 3;
const float ACCEL = SPEED * 3.0f;

struct Axis {
  int pin;
  int center;
  bool ok;
  float filtered;
};

Axis axisX = {JOY_X, 2048, false, 0};
Axis axisY = {JOY_Y, 2048, false, 0};

int selected = 0;
bool homing = false;
float clawUs = CLAW_STOP;
float serialCmd = 0;
unsigned long serialCmdUntil = 0;

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

void calibrate(Axis &a) {
  long sum = 0;
  int lo = 4095, hi = 0;
  for (int i = 0; i < 32; i++) {
    int v = analogRead(a.pin);
    sum += v;
    lo = min(lo, v);
    hi = max(hi, v);
    delay(2);
  }
  a.center = sum / 32;
  a.ok = a.center > 800 && a.center < 3300 && hi - lo < 300;
}

float readAxis(Axis &a) {
  if (!a.ok) return 0;
  int raw = analogRead(a.pin);
  float span = raw >= a.center ? 4095 - a.center : a.center;
  float v = constrain((raw - a.center) / span, -1.0f, 1.0f);
  a.filtered += 0.3f * (v - a.filtered);
  float m = fabsf(a.filtered);
  if (m < 0.2f) return 0;
  float s = (m - 0.2f) / 0.8f;
  return copysignf(s * s, a.filtered);
}

float readCommand() {
  float x = readAxis(axisX);
  float y = readAxis(axisY);
  float cmd = fabsf(y) >= fabsf(x) ? y : x;
  if (millis() < serialCmdUntil) cmd = serialCmd;
  return cmd;
}

void moveJoint(Joint &j, float targetVel, float dt) {
  float room = targetVel > 0 ? j.maxA - j.angle : j.angle - j.minA;
  float vBrake = sqrtf(2 * ACCEL * max(room, 0.0f));
  if (fabsf(targetVel) > vBrake) targetVel = copysignf(vBrake, targetVel);
  j.vel = toward(j.vel, targetVel, ACCEL * dt);
  j.angle = constrain(j.angle + j.vel * dt, j.minA, j.maxA);
  j.servo.writeMicroseconds(angleToUs(j.angle));
}

void selectJoint(int i) {
  selected = i;
  homing = false;
  Serial.print(">> selected ");
  Serial.print(i + 1);
  Serial.print(" - ");
  Serial.println(joints[i].name);
}

void handleKey(char key) {
  Serial.print(">> key ");
  Serial.println(key);
  if (key >= '1' && key <= '4') selectJoint(key - '1');
  else if (key == '0') homing = true;
  else if (key == '+' || key == '-') {
    serialCmd = key == '+' ? 1 : -1;
    serialCmdUntil = millis() + 300;
  }
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

  for (int i = 0; i < 4; i++) {
    Joint &j = joints[i];
    j.angle = j.start;
    j.servo.setPeriodHertz(50);
    if (i == CLAW && CLAW_360) {
      j.servo.attach(j.pin, 1000, 2000);
      j.servo.writeMicroseconds(CLAW_STOP);
    } else {
      j.servo.attach(j.pin, 500, 2500);
      j.servo.writeMicroseconds(angleToUs(j.angle));
    }
    delay(300);
  }

  calibrate(axisX);
  calibrate(axisY);

  Serial.println();
  Serial.println("=== ESP32 Robot Arm ===");
  Serial.printf("Joystick X (GPIO %d): center %d %s\n", JOY_X, axisX.center, axisX.ok ? "OK" : "NOT FOUND");
  Serial.printf("Joystick Y (GPIO %d): center %d %s\n", JOY_Y, axisY.center, axisY.ok ? "OK" : "NOT FOUND");
  Serial.println("Keys: 1 base | 2 shoulder | 3 elbow | 4 claw | 0 home");
  Serial.println("Serial Monitor: type 1-4, 0, + or - and press Enter");
  selectJoint(0);
}

void loop() {
  char key = keypad.getKey();
  if (key) handleKey(key);
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r' && c != ' ') handleKey(c);
  }
  if (joyButtonPressed()) {
    Serial.println(">> joystick button");
    selectJoint((selected + 1) % 4);
  }

  static unsigned long lastTick = 0;
  unsigned long now = millis();
  if (now - lastTick < 20) return;
  float dt = min((now - lastTick) / 1000.0f, 0.1f);
  lastTick = now;

  float cmd = readCommand();
  if (cmd != 0) homing = false;

  bool atHome = true;
  for (int i = 0; i < 4; i++) {
    Joint &j = joints[i];
    if (i == CLAW && CLAW_360) {
      float target = (selected == CLAW && !homing) ? CLAW_STOP + cmd * 100 : CLAW_STOP;
      clawUs = toward(clawUs, target, 400 * dt);
      j.servo.writeMicroseconds((int)(clawUs + 0.5f));
      continue;
    }
    float v = 0;
    if (homing) {
      v = constrain((j.start - j.angle) * 2, -(float)SPEED, (float)SPEED);
      if (fabsf(j.start - j.angle) > 0.5f) atHome = false;
    } else if (i == selected) {
      v = cmd * SPEED;
    }
    moveJoint(j, v, dt);
  }
  if (atHome) homing = false;

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("sel %s | joyX %d joyY %d | cmd %.2f | base %.0f shoulder %.0f elbow %.0f claw %.0f\n",
                  joints[selected].name, analogRead(JOY_X), analogRead(JOY_Y), cmd, joints[0].angle,
                  joints[1].angle, joints[2].angle, CLAW_360 ? clawUs : joints[CLAW].angle);
  }
}
