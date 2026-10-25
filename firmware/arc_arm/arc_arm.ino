#include <ESP32Servo.h>
#include <Keypad.h>

// ===================================
// СЕРВОПРИВОДИ
// ===================================

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// GPIO
const int servoPin1 = 18;
const int servoPin2 = 19;
const int servoPin3 = 21;
const int servoPin4 = 22;

// Поточні кути
int angle1 = 0;
int angle2 = 0;
int angle3 = 0;

// Активна група
int selectedServo = 0;

// ===================================
// ДЖОЙСТИК
// ===================================

const int joyX = 34;
const int joyY = 35;
const int joySW = 23;

const int JOY_CENTER = 2048;
const int DEAD_ZONE = 400;

// Інтервал зміни кута
const unsigned long moveInterval = 20;
unsigned long lastMoveTime = 0;

// ===================================
// КЛАВІАТУРА
// ===================================

const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {13, 14, 16, 17};
byte colPins[COLS] = {25, 26, 27, 32};

Keypad keypad = Keypad(
  makeKeymap(keys),
  rowPins,
  colPins,
  ROWS,
  COLS
);

// ===================================
// SETUP
// ===================================

void setup() {

  Serial.begin(9600);
  delay(1000);

  pinMode(joySW, INPUT_PULLUP);
  analogReadResolution(12);

  // Частота 50 Гц
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  // Підключення
  servo1.attach(servoPin1, 500, 2500);
  servo2.attach(servoPin2, 500, 2500);
  servo3.attach(servoPin3, 500, 2500);
  servo4.attach(servoPin4, 1000, 2000);

  // Початкові команди
  //servo1.write(90);
  //servo2.write(90);
  //servo3.write(90);
  servo4.writeMicroseconds(1500);

  Serial.println("ESP32 Robot Arm Ready");
  Serial.println("1 - Servo 1");
  Serial.println("2 - Servo 2 + Servo 3");
  Serial.println("3 - Gripper");

}

// ===================================
// LOOP
// ===================================

void loop() {

  // ---------- КЛАВІАТУРА ----------

  char key = keypad.getKey();

  if (key != NO_KEY) {

    Serial.print("Key pressed: ");
    Serial.println(key);

    if (key == '1') {
      selectedServo = 1;
    }
    else if (key == '2') {
      selectedServo = 2;
    }
    else if (key == '3') {
      selectedServo = 3;
    }

    // Зупинка клешні при перемиканні
    if (selectedServo != 3) {
      servo4.writeMicroseconds(1500);
    }
  }

  // ---------- ДЖОЙСТИК ----------

  int joyValue = analogRead(joyY);
  int deviation = joyValue - JOY_CENTER;

  int direction = 0;

  if (deviation > DEAD_ZONE) {
    direction = 7;
  }
  else if (deviation < -DEAD_ZONE) {
    direction = -7;
  }

  // ---------- КЛЕШНЯ ----------

  if (selectedServo == 3) {

    if (direction == 7) {
      servo4.writeMicroseconds(1600);
    }
    else if (direction == -7) {
      servo4.writeMicroseconds(1400);
    }
    else {
      servo4.writeMicroseconds(1500);
    }
  }
  else {
    servo4.writeMicroseconds(1500);
  }

  // ---------- ПОЗИЦІЙНІ СЕРВО ----------

  if (millis() - lastMoveTime >= moveInterval) {

    lastMoveTime = millis();

    if (selectedServo == 1) {

      angle1 = constrain(
        angle1 + direction,
        0,
        180
      );

      servo1.write(angle1);
    }

    else if (selectedServo == 2) {

      angle2 = constrain(
        angle2 + direction,
        0,
        180
      );

      angle3 = 180 - angle2;

      servo2.write(angle2);
      servo3.write(angle3);
    }
  }

  // ---------- ДІАГНОСТИКА ----------

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 300) {

    lastPrint = millis();

    Serial.print("Joystick: ");
    Serial.print(joyValue);

    Serial.print(" | Selected: ");
    Serial.print(selectedServo);

    Serial.print(" | Direction: ");
    Serial.println(direction);
  }
}
