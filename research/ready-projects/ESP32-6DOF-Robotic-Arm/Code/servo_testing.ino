#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ============================================================
//  CHANGE THIS NUMBER TO TEST DIFFERENT SERVO:
//  0 = BASE    (MG996R) - test last
//  1 = LOWER   (MG996R) - test 5th
//  2 = MID     (MG996R) - test 4th
//  3 = UPPER   (SG90)   - test 3rd
//  4 = WRIST   (MG90S)  - test 2nd
//  5 = GRIPPER (SG90)   - test FIRST
// ============================================================
#define TEST_CHANNEL 9

#define SERVO_MIN 150
#define SERVO_MAX 600

String servoInfo[] = {
  "Ch0 — BASE    (MG996R) — inside white base",
  "Ch1 — LOWER   (MG996R) — first joint up",
  "Ch2 — MID     (MG996R) — second joint",
  "Ch3 — UPPER   (MG90S)   — third joint",
  "Ch8 — WRIST   (MG90S)  — fourth joint",
  "Ch9 — GRIPPER (MG90S)   — claw at top"
};

int currentAngle = 90;

int degreesToPulse(int deg) {
  return map(constrain(deg, 0, 180), 0, 180, SERVO_MIN, SERVO_MAX);
}

void moveServo(int deg) {
  currentAngle = constrain(deg, 0, 180);
  pca.setPWM(TEST_CHANNEL, 0, degreesToPulse(currentAngle));
  Serial.print(">> "); Serial.print(currentAngle); Serial.println(" deg");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);
  delay(300);

  // Only move the test servo to 90, everything else untouched
  moveServo(90);

  Serial.println("\n============================================");
  Serial.println("  SINGLE SERVO TEST");
  Serial.println("============================================");
  Serial.print("  Testing: "); Serial.println(servoInfo[TEST_CHANNEL]);
  Serial.println("  All other servos: NOT activated");
  Serial.println("============================================");
  Serial.println("  Commands:");
  Serial.println("  +      → +5 degrees");
  Serial.println("  -      → -5 degrees");
  Serial.println("  ++     → +15 degrees");
  Serial.println("  --     → -15 degrees");
  Serial.println("  90     → center (90 deg)");
  Serial.println("  0      → minimum");
  Serial.println("  180    → maximum");
  Serial.println("  [num]  → go to exact angle e.g. 45");
  Serial.println("  sweep  → auto sweep 0→180→0 (full test)");
  Serial.println("============================================");
  Serial.print("  Current angle: "); Serial.println(currentAngle);
  Serial.println("============================================\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String lower = cmd;
    lower.toLowerCase();

    if (lower == "+")      moveServo(currentAngle + 5);
    else if (lower == "++")     moveServo(currentAngle + 15);
    else if (lower == "-")      moveServo(currentAngle - 5);
    else if (lower == "--")     moveServo(currentAngle - 15);
    else if (lower == "90")     moveServo(90);
    else if (lower == "0")      moveServo(0);
    else if (lower == "180")    moveServo(180);

    else if (lower == "sweep") {
      Serial.println(">> Sweeping 90 → 0 → 180 → 90...");
      for (int i = 90; i >= 0; i -= 5)  { moveServo(i); delay(80); }
      for (int i = 0; i <= 180; i += 5) { moveServo(i); delay(80); }
      for (int i = 180; i >= 90; i -= 5){ moveServo(i); delay(80); }
      Serial.println(">> Sweep done! Did it move smoothly?");
    }

    else {
      // Try as exact angle
      bool isNum = true;
      for (int i = 0; i < lower.length(); i++) {
        if (!isDigit(lower[i])) { isNum = false; break; }
      }
      if (isNum && lower.length() > 0) {
        int angle = lower.toInt();
        if (angle >= 0 && angle <= 180) moveServo(angle);
        else Serial.println("!! Angle must be 0-180");
      } else {
        Serial.print("!! Unknown: "); Serial.println(cmd);
      }
    }
  }
}