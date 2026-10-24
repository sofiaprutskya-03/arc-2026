// ESP32 Sender for Robotic Arm - CPE101 FINAL PROJECT
// Copyright by Arm :D

#include <I2Cdev.h>
#include <esp_now.h>
#include <WiFi.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <Wire.h>

MPU6050 mpu;

#define ModeChangerButton 18
#define FlexSensor 36

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//*****      STRUCT       *****//
typedef struct Data
{
  int mode_id;
  int flexValue;
  float yaw;
  float pitch;
  float roll;
} Data;

Data DataToSent;

// RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0xD4, 0x8A, 0xFC, 0xC8, 0x00, 0x30};

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock

  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();

  pinMode(ModeChangerButton, INPUT);
  // Check if MPU6050 is connected
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Initialize DMP (Digital Motion Processor)
  devStatus = mpu.dmpInitialize();

  // Set offsets (calibration)
  mpu.setXGyroOffset(22);
  mpu.setYGyroOffset(48);
  mpu.setZGyroOffset(29);
  mpu.setXAccelOffset(-1242);
  mpu.setYAccelOffset(2076);
  mpu.setZAccelOffset(962);

  // Check DMP initialization
  if (devStatus == 0) {
    //    mpu.CalibrateAccel(6);
    //    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (1);  // Halt execution if initialization fails
  }

  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Change mode based on button press
    DataToSent.mode_id = digitalRead(ModeChangerButton) == HIGH ? 0 : 1;
    DataToSent.flexValue = analogRead(FlexSensor);

    // Convert radians to degrees
    DataToSent.yaw = ypr[0];
    DataToSent.pitch = ypr[1];
    DataToSent.roll = ypr[2];

    if (abs(DataToSent.pitch) > 1.3 && abs(DataToSent.roll) > 0.7) {  // Sensor is roughly upside down
      DataToSent.pitch = 0.0; 
      DataToSent.roll = 0.0; 
    } else {
      // Apply normal filtering/clamping logic when not upside down
      DataToSent.pitch = constrain(DataToSent.pitch, -1,1);
      DataToSent.roll = constrain(DataToSent.roll, -1,1);
    }


    // Print sensor data
    Serial.printf("\nYaw: %.2f, Pitch: %.2f, Roll: %.2f, Flex: %d, Mode: %d ",
                  DataToSent.yaw, DataToSent.pitch, DataToSent.roll, DataToSent.flexValue , DataToSent.mode_id);

    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&DataToSent, sizeof(DataToSent));
    if (result != ESP_OK) {
      Serial.printf("Error sending data: %d ", result);
    }
  }
}
