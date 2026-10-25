#include <ESP32Servo.h>
#include <Keypad.h>

// ===== НАЛАШТУВАННЯ =====

//                       база  плече  лікоть  клешня
const int SERVO_PIN[4] = {18,   19,    21,     22};
float angle[4]         = {90,   90,    90,     90};   // кут при ввімкненні: середина, щоб рух був в обидва боки
const int MIN_ANGLE[4] = {0,    0,     0,      20};
const int MAX_ANGLE[4] = {180,  180,   180,    160};

const float SPEED = 60;   // град/с, коли джойстик відхилено до кінця
const int JOY_Y = 35;

byte rowPins[4] = {13, 14, 16, 17};
byte colPins[4] = {25, 26, 27, 32};

// ========================

const char *NAMES[4] = {"base", "shoulder", "elbow", "claw"};

char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);

Servo servo[4];
int selected = 0;
int joyCenter = 2048;

void writeServo(int i) {
  servo[i].writeMicroseconds(500 + (int)(angle[i] * 2000 / 180));
}

void setup() {
  Serial.begin(9600);
  analogReadResolution(12);

  for (int i = 0; i < 4; i++) {
    servo[i].setPeriodHertz(50);
    servo[i].attach(SERVO_PIN[i], 500, 2500);
    writeServo(i);
    delay(300);
  }

  long sum = 0;
  for (int i = 0; i < 20; i++) sum += analogRead(JOY_Y);
  joyCenter = sum / 20;

  Serial.println("Ready. 1 base | 2 shoulder | 3 elbow | 4 claw, then move joystick up/down");
}

void loop() {
  char key = keypad.getKey();
  if (!key && Serial.available()) key = Serial.read();
  if (key >= '1' && key <= '4') {
    selected = key - '1';
    Serial.print("Selected: ");
    Serial.println(NAMES[selected]);
  }

  static unsigned long lastMove = 0;
  if (millis() - lastMove < 20) return;
  lastMove = millis();

  int dev = analogRead(JOY_Y) - joyCenter;
  float cmd = 0;
  if (dev > 300) cmd = (dev - 300) / (float)(4095 - joyCenter - 300);
  if (dev < -300) cmd = (dev + 300) / (float)(joyCenter - 300);
  cmd = constrain(cmd, -1.0f, 1.0f);

  if (cmd != 0) {
    angle[selected] = constrain(angle[selected] + cmd * SPEED * 0.02f, MIN_ANGLE[selected], MAX_ANGLE[selected]);
    writeServo(selected);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("%s | joy %d | base %.0f shoulder %.0f elbow %.0f claw %.0f\n", NAMES[selected], dev, angle[0],
                  angle[1], angle[2], angle[3]);
  }
}
