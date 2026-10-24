// ESP 32 Receiver FOR Robotic Arm as CPE101 FINAL PROJECT
// Copyright by Arm :D
#define DEBUG 0

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

Servo myservo;

//*****      PIN DEFINING       *****//
#define Base_EnablePin          21
#define Base_DirectionPin       19
#define Base_PulsePin           18

#define Elbow_EnablePin         5
#define Elbow_DirectionPin      17
#define Elbow_PulsePin          16

#define Shoulder_EnablePin      25
#define Shoulder_DirectionPin   26
#define Shoulder_PulsePin       27

#define Elbow_BumpPin 13
#define Shoulder_BumpPin 12

#define Servo_PWM 33

#define MAX_GRIP_ANGLE    70          // Maximum servo angle for gripping
#define MIN_GRIP_ANGLE    0          // Minimum servo angle for releasing

#define ELBOW_MAX_STEP      1000
#define SHOULDER_MAX_STEP   600
#define BASE_MAX_STEP       1200
#define ROLL_OFFSET         -0.3
#define BASE_SPEED          1000
int Elbow_Step = 0;
int Shoulder_Step = 0;
int Base_Step = 600;

//*****      STRUCT       *****//
typedef enum {
  MODE_SHOULDER = 0,
  MODE_ELBOW = 1,
} Mode;

typedef struct Data {
  Mode mode_id;
  int flexValue;
  float yaw;
  float pitch;
  float roll;
} Data;

Data DataRecieved;

// Callback function for ESP-NOW
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int lengthofstring) {
  if (lengthofstring == sizeof(DataRecieved)) {
    memcpy(&DataRecieved, incomingData, sizeof(DataRecieved));

    // Display Sensor Received Data
    DataRecieved.roll += (DataRecieved.roll <= 0) ? ROLL_OFFSET : abs(ROLL_OFFSET);

    //    Serial.print("Yaw: ");
    //    Serial.print(DataRecieved.yaw);
    Serial.print(", Pitch: ");
    Serial.print(DataRecieved.pitch);
    Serial.print(", Roll: ");
    Serial.print(DataRecieved.roll);
    Serial.print(", Flex: ");
    Serial.print(DataRecieved.flexValue);
    Serial.print(", Mode: ");
    Serial.println(DataRecieved.mode_id);
  } else {
    Serial.println("Received data size mismatch");
  }
}



// Function to control motors
void controlMotor(int pulsePin, int directionPin, float directions, int baseSpeed,
                  int isNotBumpDetected, int MAX_STEP , int *current_step, int dependent_step = 0) {

  int dynamic_step = *current_step + dependent_step;
  if (dynamic_step > 100) {
    dynamic_step = dynamic_step - 100;
  }
  if (directions > 0.7 && dynamic_step <= MAX_STEP) {
    digitalWrite(directionPin, HIGH);
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(baseSpeed);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(baseSpeed);
    (*current_step)++;
  } else if (directions < -0.7 && isNotBumpDetected) {
    if (isNotBumpDetected == 2 && dynamic_step <= 0) {
      return;
    }
    digitalWrite(directionPin, LOW);
    digitalWrite(pulsePin, HIGH);
    delayMicroseconds(baseSpeed);
    digitalWrite(pulsePin, LOW);
    delayMicroseconds(baseSpeed);
    if (!isNotBumpDetected) {
      (*current_step) = 0;
    }
    if (dynamic_step > 0) {
      (*current_step)--;
    }
  } else {
    digitalWrite(pulsePin, LOW);
  }
#if DEBUG
  Serial.print("Dynamic Step: ");
  Serial.print(dynamic_step);
  Serial.print("  Current Step: ");
  Serial.print(*current_step);
  Serial.print("  Dependent Step: ");
  Serial.println(dependent_step);
#endif
}


void InitializeRoboticArm() {
  Serial.println("Initializing Robotic Arm...");
  unsigned long timeout;
  int isShoulderNotBump = digitalRead(Shoulder_BumpPin);
  int isElbowNotBump = digitalRead(Elbow_BumpPin);

  Serial.println("Initializing Gripper...");
      Serial.print("MIN GRIP: ");
  myservo.write(MIN_GRIP_ANGLE);
  delay(5000);
      Serial.print("MAX GRIP: ");
  myservo.write(MAX_GRIP_ANGLE);
  delay(5000);
  Serial.println("Gripper initialized.");

  for (int i = 0; i < 300 && isElbowNotBump; i++) {
    digitalWrite(Elbow_DirectionPin, LOW);
    digitalWrite(Elbow_PulsePin, HIGH);
    delayMicroseconds(1000);  // Pulse width
    digitalWrite(Elbow_PulsePin, LOW);
    delayMicroseconds(1000);
    isElbowNotBump = digitalRead(Elbow_BumpPin);
  }
  Serial.println("Setting Shoulder...");
  timeout = millis(); // Start timeout
  while (isShoulderNotBump) {
    digitalWrite(Shoulder_DirectionPin, LOW);
    digitalWrite(Shoulder_PulsePin, HIGH);
    delayMicroseconds(1000);  // Pulse width
    digitalWrite(Shoulder_PulsePin, LOW);
    delayMicroseconds(1000);

    isShoulderNotBump = digitalRead(Shoulder_BumpPin);

    if (millis() - timeout > 5000) {
      Serial.println("Error: Shoulder setting timed out!");
      break;  // Exit the loop if timeout occurs
    }
  }
  Serial.println("Shoulder Finished.");
  Serial.println("Setting Elbow...");
  timeout = millis(); // Start timeout
  while (isElbowNotBump) {
    digitalWrite(Elbow_DirectionPin, LOW);
    digitalWrite(Elbow_PulsePin, HIGH);
    delayMicroseconds(1000);  // Pulse width
    digitalWrite(Elbow_PulsePin, LOW);
    delayMicroseconds(1000);

    isElbowNotBump = digitalRead(Elbow_BumpPin);

    if (millis() - timeout > 5000) {
      Serial.println("Error: Elbow homing timed out!");
      break;  // Exit the loop if timeout occurs
    }
  }
  Serial.println("Elbow Finished.");

  Serial.println("Robotic Arm Initialization Complete!");
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialization Failed");
    ESP.restart();
  }

  // Pin configuration
  pinMode(Base_EnablePin, OUTPUT);
  pinMode(Base_DirectionPin, OUTPUT);
  pinMode(Base_PulsePin, OUTPUT);

  pinMode(Shoulder_EnablePin, OUTPUT);
  pinMode(Shoulder_DirectionPin, OUTPUT);
  pinMode(Shoulder_PulsePin, OUTPUT);
  pinMode(Shoulder_BumpPin, INPUT_PULLUP);

  pinMode(Elbow_EnablePin, OUTPUT);
  pinMode(Elbow_DirectionPin, OUTPUT);
  pinMode(Elbow_PulsePin, OUTPUT);
  pinMode(Elbow_BumpPin, INPUT_PULLUP);

  // Enable motors
  digitalWrite(Base_EnablePin, HIGH);
  digitalWrite(Shoulder_EnablePin, HIGH);
  digitalWrite(Elbow_EnablePin, HIGH);

  myservo.attach(Servo_PWM);

  InitializeRoboticArm();

  // Register ESP-NOW callback
  esp_now_register_recv_cb(OnDataRecv);


}

void loop() {
  int isShoulderNotBump = digitalRead(Shoulder_BumpPin);
  int isElbowNotBump = digitalRead(Elbow_BumpPin);
  // Constrain the angle to prevent out-of-range values

  // Control Grip
  int ServoAngle = map(DataRecieved.flexValue, 250, 700, MAX_GRIP_ANGLE, MIN_GRIP_ANGLE);
  ServoAngle = constrain(ServoAngle, MIN_GRIP_ANGLE, MAX_GRIP_ANGLE);

  myservo.write(ServoAngle);
  // Control Base
  controlMotor(Base_PulsePin, Base_DirectionPin, DataRecieved.pitch, BASE_SPEED, 2, BASE_MAX_STEP, &Base_Step);

  // Control Shoulder or Elbow based on mode
  if (DataRecieved.mode_id == MODE_SHOULDER) {
    controlMotor(Shoulder_PulsePin, Shoulder_DirectionPin, DataRecieved.roll, BASE_SPEED , isShoulderNotBump, SHOULDER_MAX_STEP, &Shoulder_Step);
  } else if (DataRecieved.mode_id == MODE_ELBOW) {
    controlMotor(Elbow_PulsePin, Elbow_DirectionPin, DataRecieved.roll, BASE_SPEED , isElbowNotBump, ELBOW_MAX_STEP, &Elbow_Step, Shoulder_Step);
  }

#if DEBUG
  Serial.print("  Elbow Bump: ");
  Serial.print(isElbowNotBump);
  Serial.print("  Shoulder Bump: ");
  Serial.print(isShoulderNotBump);
#endif
}
