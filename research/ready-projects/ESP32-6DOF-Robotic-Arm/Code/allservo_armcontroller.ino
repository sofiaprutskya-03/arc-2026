#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define NUM_SERVOS 6
#define SERVO_MIN  200
#define SERVO_MAX  550

// Physical PCA9685 channels
int CHANNELS[NUM_SERVOS] = {0, 1, 2, 3, 8, 9};

const char* labels[] = {"Base   (MG996R)", "Lower  (MG996R)", "Mid    (MG996R)", "Upper  (MG90S) ", "Wrist  (MG90S) ", "Gripper(MG90S) "};
const char* shorts[] = {"Base", "Lower", "Mid", "Upper", "Wrist", "Gripper"};

// ── PER SERVO SAFE ANGLE LIMITS ─────────────────────────────

int ANGLE_MIN[NUM_SERVOS] = { 0,  0,  0,  0,  0,  30};
int ANGLE_MAX[NUM_SERVOS] = {180, 180, 180, 180, 180, 120};
//                           Base Lwr  Mid  Upr  Wst  Grip


int angles[NUM_SERVOS] = {90, 90, 90, 90, 90, 90};
int selected = 0;

int degreesToPulse(int deg) {
  return map(constrain(deg, 0, 180), 0, 180, SERVO_MIN, SERVO_MAX);
}

void moveServo(int idx, int deg) {
  // Clamp to per-servo safe limits
  angles[idx] = constrain(deg, ANGLE_MIN[idx], ANGLE_MAX[idx]);
  pca.setPWM(CHANNELS[idx], 0, degreesToPulse(angles[idx]));
}

void centerAll() {
  for (int i = 0; i < NUM_SERVOS; i++) moveServo(i, 90);
}

void sweepServo(int idx) {
  Serial.print(">> Sweeping "); Serial.print(shorts[idx]);
  Serial.print(" ("); Serial.print(ANGLE_MIN[idx]);
  Serial.print(" to "); Serial.print(ANGLE_MAX[idx]); Serial.println(")");

  // Sweep only within safe limits
  for (int i = 90; i >= ANGLE_MIN[idx]; i -= 5)          { moveServo(idx, i); delay(80); }
  for (int i = ANGLE_MIN[idx]; i <= ANGLE_MAX[idx]; i += 5){ moveServo(idx, i); delay(80); }
  for (int i = ANGLE_MAX[idx]; i >= 90; i -= 5)           { moveServo(idx, i); delay(80); }

  Serial.print(">> "); Serial.print(shorts[idx]); Serial.println(" sweep done!");
}

void printStatus() {
  Serial.println("\n--- ALL ANGLES ---");
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (i == selected) Serial.print(" >> ");
    else Serial.print("    ");
    Serial.print("Ch"); Serial.print(CHANNELS[i]);
    Serial.print(" "); Serial.print(labels[i]);
    Serial.print(": "); Serial.print(angles[i]);
    Serial.print(" deg");
    Serial.print("  [limit: "); Serial.print(ANGLE_MIN[i]);
    Serial.print("-"); Serial.print(ANGLE_MAX[i]); Serial.println("]");
  }
  Serial.println("------------------");
  Serial.print("Selected: "); Serial.print(selected);
  Serial.print(" - "); Serial.println(shorts[selected]);
  Serial.println("Commands: + - ++ -- [angle] | select 0-5 | sweep | center | status | limits\n");
}

void printLimits() {
  Serial.println("\n--- CURRENT SAFE LIMITS ---");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("  "); Serial.print(shorts[i]);
    Serial.print(": "); Serial.print(ANGLE_MIN[i]);
    Serial.print(" to "); Serial.print(ANGLE_MAX[i]); Serial.println(" deg");
  }
  Serial.println("----------------------------");
  Serial.println("To change: setmin [deg] or setmax [deg]\n");
}

void printHelp() {
  Serial.println("\n========================================");
  Serial.println("  6-SERVO TUNER (with safe limits)");
  Serial.println("========================================");
  Serial.println("  SELECT:");
  Serial.println("  select 0 -> Base    Ch0  MG996R");
  Serial.println("  select 1 -> Lower   Ch1  MG996R");
  Serial.println("  select 2 -> Mid     Ch2  MG996R");
  Serial.println("  select 3 -> Upper   Ch3  MG90S");
  Serial.println("  select 4 -> Wrist   Ch8  MG90S");
  Serial.println("  select 5 -> Gripper Ch9  MG90S");
  Serial.println("");
  Serial.println("  MOVE SELECTED SERVO:");
  Serial.println("  +        -> +5 deg");
  Serial.println("  -        -> -5 deg");
  Serial.println("  ++       -> +15 deg");
  Serial.println("  --       -> -15 deg");
  Serial.println("  [0-180]  -> exact angle (clamped to limits)");
  Serial.println("  sweep    -> auto sweep within safe limits");
  Serial.println("");
  Serial.println("  LIMITS (adjust if needed):");
  Serial.println("  setmin [deg] -> set min for selected servo");
  Serial.println("  setmax [deg] -> set max for selected servo");
  Serial.println("  limits       -> show all current limits");
  Serial.println("");
  Serial.println("  OTHER:");
  Serial.println("  center   -> all servos to 90");
  Serial.println("  status   -> show all angles + limits");
  Serial.println("  help     -> show this menu");
  Serial.println("========================================");
  Serial.println("  !! Gripper limited to 60-120 deg !!");
  Serial.println("     to prevent overheating");
  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50);
  delay(300);

  Serial.println(">> Centering all servos...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, 90);
    delay(300);
    Serial.print("   Ch"); Serial.print(CHANNELS[i]);
    Serial.print(" "); Serial.print(shorts[i]);
    Serial.println(" -> 90");
  }
  Serial.println(">> All centered!");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String lower = cmd;
    lower.toLowerCase();

    if (lower.startsWith("select ")) {
      int idx = lower.substring(7).toInt();
      if (idx >= 0 && idx < NUM_SERVOS) {
        selected = idx;
        Serial.print(">> Selected "); Serial.print(selected);
        Serial.print(" - Ch"); Serial.print(CHANNELS[selected]);
        Serial.print(" "); Serial.print(shorts[selected]);
        Serial.print(" ["); Serial.print(ANGLE_MIN[selected]);
        Serial.print("-"); Serial.print(ANGLE_MAX[selected]); Serial.println("]");
      } else {
        Serial.println("!! Must be 0-5");
      }
    }

    else if (lower.startsWith("setmin ")) {
      int val = lower.substring(7).toInt();
      if (val >= 0 && val < ANGLE_MAX[selected]) {
        ANGLE_MIN[selected] = val;
        Serial.print(">> "); Serial.print(shorts[selected]);
        Serial.print(" min set to "); Serial.println(val);
      } else Serial.println("!! Invalid value");
    }

    else if (lower.startsWith("setmax ")) {
      int val = lower.substring(7).toInt();
      if (val > ANGLE_MIN[selected] && val <= 180) {
        ANGLE_MAX[selected] = val;
        Serial.print(">> "); Serial.print(shorts[selected]);
        Serial.print(" max set to "); Serial.println(val);
      } else Serial.println("!! Invalid value");
    }

    else if (lower == "+")      { moveServo(selected, angles[selected] + 5);  Serial.print(shorts[selected]); Serial.print(": "); Serial.println(angles[selected]); }
    else if (lower == "++")     { moveServo(selected, angles[selected] + 15); Serial.print(shorts[selected]); Serial.print(": "); Serial.println(angles[selected]); }
    else if (lower == "-")      { moveServo(selected, angles[selected] - 5);  Serial.print(shorts[selected]); Serial.print(": "); Serial.println(angles[selected]); }
    else if (lower == "--")     { moveServo(selected, angles[selected] - 15); Serial.print(shorts[selected]); Serial.print(": "); Serial.println(angles[selected]); }
    else if (lower == "center") { centerAll(); Serial.println(">> All -> 90"); }
    else if (lower == "status") { printStatus(); }
    else if (lower == "limits") { printLimits(); }
    else if (lower == "help")   { printHelp(); }
    else if (lower == "sweep")  { sweepServo(selected); }

    else {
      bool isNum = true;
      for (int i = 0; i < lower.length(); i++) {
        if (!isDigit(lower[i])) { isNum = false; break; }
      }
      if (isNum && lower.length() > 0) {
        int angle = lower.toInt();
        moveServo(selected, angle);
        Serial.print(shorts[selected]); Serial.print(": "); Serial.println(angles[selected]);
      } else {
        Serial.print("!! Unknown: "); Serial.println(cmd);
        Serial.println("   Type 'help' for commands");
      }
    }
  }
}