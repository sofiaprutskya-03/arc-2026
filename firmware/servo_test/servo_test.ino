#include <ESP32Servo.h>

const int PINS[4] = {18, 19, 21, 22};
const char *NAMES[4] = {"1 base (GPIO 18)", "2 shoulder (GPIO 19)", "3 elbow (GPIO 21)", "4 claw (GPIO 22)"};

Servo servo;

void setup() {
  Serial.begin(9600);
  servo.setPeriodHertz(50);
}

void loop() {
  for (int i = 0; i < 4; i++) {
    Serial.print("Now moving: ");
    Serial.println(NAMES[i]);
    servo.attach(PINS[i], 500, 2500);
    for (int a = 90; a <= 120; a++) { servo.write(a); delay(20); }
    for (int a = 120; a >= 60; a--) { servo.write(a); delay(20); }
    for (int a = 60; a <= 90; a++) { servo.write(a); delay(20); }
    servo.detach();
    delay(1000);
  }
}
