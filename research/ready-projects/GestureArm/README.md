<!-- prettier-ignore -->
<div align="center">

# GestureArm

*An ESP32 hand-gesture controller for a stepper-driven robotic arm.*

![Arduino](https://img.shields.io/badge/Arduino-ESP32-00979d?style=flat-square&logo=arduino&logoColor=white)
![ESP-NOW](https://img.shields.io/badge/Wireless-ESP--NOW-0f766e?style=flat-square)
![MPU6050](https://img.shields.io/badge/Sensor-MPU6050-475569?style=flat-square)
![Servo](https://img.shields.io/badge/Actuator-servo%20%2B%20steppers-7c3aed?style=flat-square)

[Overview](#overview) • [Features](#features) • [Hardware](#hardware) • [Get started](#get-started) • [Controls](#controls) • [Calibration](#calibration)

</div>

GestureArm is a wireless robotic-arm control project built with two ESP32 boards. A glove-mounted sender reads wrist orientation from an MPU6050, grip input from a flex sensor, and a mode button, then streams gesture data over ESP-NOW to a receiver ESP32 that drives the robotic arm.

The receiver converts pitch, roll, flex, and mode data into step/direction signals for the base, shoulder, and elbow axes, plus PWM control for a servo gripper.

> [!NOTE]
> The repository is still named `Hand-Gestured-Controlled-Robotic-Arm`, but **GestureArm** is a shorter project name that fits better in a README, demo slide, and GitHub About section.

## Overview

The project is split into two Arduino sketches:

```text
Glove sender
  ESP32 + MPU6050 + flex sensor + mode button
  Reads wrist motion and finger bend
  Sends compact gesture packets through ESP-NOW

Robotic arm receiver
  ESP32 + stepper drivers + servo + limit switches
  Receives gesture packets
  Moves base, shoulder, elbow, and gripper
```

The current control model is intentionally direct:

- Wrist pitch controls the base axis.
- Wrist roll controls either the shoulder or elbow.
- The mode button selects shoulder mode or elbow mode.
- The flex sensor opens and closes the gripper.

> [!CAUTION]
> The receiver moves the gripper and homes the shoulder and elbow during startup. Keep the mechanism clear, verify the limit switches, and be ready to disconnect motor power before uploading, resetting, or testing the receiver.

## Features

- **Wireless glove control** using ESP-NOW between two ESP32 boards.
- **MPU6050 DMP orientation tracking** for yaw, pitch, and roll.
- **Flex-sensor gripper control** mapped to a servo angle.
- **Three stepper-driven axes** for base, shoulder, and elbow movement.
- **Mode-based axis selection** to control shoulder or elbow with the same wrist roll gesture.
- **Limit-switch homing** for shoulder and elbow startup alignment.
- **Step bounds and movement thresholds** to reduce accidental travel.
- **Serial diagnostics** for sensor readings, ESP-NOW delivery, and receiver state.

## Hardware

### Required components

- 2 ESP32 development boards
- MPU6050 accelerometer/gyroscope module
- Flex sensor with a suitable voltage-divider circuit
- Momentary or toggle mode-selection button
- 3 stepper motors with step/direction/enable drivers
- 1 servo motor for the gripper
- 2 limit switches for shoulder and elbow homing
- Robotic arm frame, wiring, and suitable external motor power supplies

> [!IMPORTANT]
> Do not power motors from the ESP32 board. Use a properly rated external supply for the motor drivers and connect grounds where required by your driver setup.

### Sender pinout

| Component | ESP32 pin |
| --- | ---: |
| Mode button | GPIO 18 |
| Flex sensor analog output | GPIO 36 |
| MPU6050 SDA | Default ESP32 I2C SDA, usually GPIO 21 |
| MPU6050 SCL | Default ESP32 I2C SCL, usually GPIO 22 |

The sender configures the mode button as `INPUT`, so the external circuit must provide a defined high or low level.

### Receiver pinout

| Axis/component | Enable | Direction | Step/PWM |
| --- | ---: | ---: | ---: |
| Base stepper driver | GPIO 21 | GPIO 19 | GPIO 18 |
| Shoulder stepper driver | GPIO 25 | GPIO 26 | GPIO 27 |
| Elbow stepper driver | GPIO 5 | GPIO 17 | GPIO 16 |
| Gripper servo | - | - | GPIO 33 |

| Limit switch | ESP32 pin | Input mode |
| --- | ---: | --- |
| Shoulder | GPIO 12 | `INPUT_PULLUP` |
| Elbow | GPIO 13 | `INPUT_PULLUP` |

## Software requirements

- Arduino IDE, Arduino CLI, PlatformIO, or another ESP32-capable Arduino build environment
- ESP32 board support package
- `WiFi.h`, `esp_now.h`, and `Wire.h` from ESP32 Arduino support
- I2Cdevlib MPU6050 support:
  - `I2Cdev.h`
  - `MPU6050_6Axis_MotionApps612.h`
- ESP32Servo library for `ESP32Servo.h`

The repository includes [`Library_ESP32_Servo.rar`](./ESP32Reciever/data/Library_ESP32_Servo.rar). Extract or install the servo library where your Arduino toolchain can find it.

## Get started

### 1. Prepare the receiver

1. Install ESP32 board support and the required libraries.
2. Wire the receiver ESP32 to the stepper drivers, gripper servo, and limit switches.
3. Open [`ESP32Reciever/ESP32Reciever.ino`](./ESP32Reciever/ESP32Reciever.ino).
4. Select the receiver board and port.
5. Upload the sketch.
6. Open Serial Monitor at `115200` baud and note the ESP32 station MAC address.

> [!TIP]
> The folder name is `ESP32Reciever` in the repository. The spelling is kept in paths so links and commands match the actual project structure.

### 2. Prepare the sender

1. Wire the sender ESP32 to the MPU6050, flex sensor, and mode button.
2. Open [`ESP32Sender/ESP32Sender.ino`](./ESP32Sender/ESP32Sender.ino).
3. Replace `broadcastAddress` with the receiver ESP32 MAC address:

```cpp
uint8_t broadcastAddress[] = {0xD4, 0x8A, 0xFC, 0xC8, 0x00, 0x30};
```

4. Select the sender board and port.
5. Upload the sketch.
6. Open Serial Monitor at `115200` baud and confirm MPU6050 initialization and ESP-NOW delivery.

### 3. Test incrementally

1. Run the sender alone and confirm pitch, roll, flex, and mode values change as expected.
2. Power the receiver logic without motor power and confirm ESP-NOW packets arrive.
3. Test the gripper servo range.
4. Test homing with limit switches installed and reachable.
5. Enable motor power and test each axis at low speed or low current first.

## Controls

| Input | Result |
| --- | --- |
| Wrist pitch above or below the movement threshold | Rotates the base |
| Wrist roll with mode set to shoulder | Moves the shoulder |
| Wrist roll with mode set to elbow | Moves the elbow |
| Flex sensor bend | Opens or closes the gripper |

The receiver uses a `0.7` pitch/roll threshold before commanding stepper movement. Flex readings are mapped from `250..700` to gripper angles from `70..0` degrees and constrained to the configured servo range.

## Calibration

The checked-in constants are specific to the original hardware build. Review these before operating a different arm:

| Area | Values to review |
| --- | --- |
| MPU6050 | Gyro and accelerometer offsets in `ESP32Sender.ino` |
| Wrist gestures | `ROLL_OFFSET`, pitch threshold, and roll threshold |
| Gripper | Flex range `250..700`, `MIN_GRIP_ANGLE`, `MAX_GRIP_ANGLE` |
| Motion limits | `BASE_MAX_STEP`, `SHOULDER_MAX_STEP`, `ELBOW_MAX_STEP` |
| Homing | Limit-switch polarity and startup travel direction |
| Motors | Direction levels, speed, current limit, and mechanical range |

Both sketches define the transmitted `Data` structure. Keep the field order and field types compatible when changing the wireless protocol.

## Project structure

```text
.
├── ESP32Sender/
│   └── ESP32Sender.ino          # Glove sensors and ESP-NOW transmitter
├── ESP32Reciever/
│   ├── ESP32Reciever.ino        # Arm control and ESP-NOW receiver
│   └── data/
│       └── Library_ESP32_Servo.rar
└── README.md
```

## Verification

This project does not include automated tests. Use serial output and staged hardware checks instead:

- Sender reports successful MPU6050 connection and DMP initialization.
- Sender prints `Delivery Success` after ESP-NOW sends.
- Receiver prints changing pitch, roll, flex, and mode values.
- Shoulder and elbow homing stop when their limit switches are triggered.
- Base, shoulder, elbow, and gripper move in the expected direction and stay within safe travel limits.

## Troubleshooting

| Issue | Check |
| --- | --- |
| Sender prints delivery failures | Confirm the receiver MAC address, both boards are powered, and both sketches use ESP-NOW in station mode. |
| MPU6050 fails to initialize | Check I2C wiring, power, ground, library installation, and sensor address. |
| Gripper moves backward | Swap the mapped angle range or adjust `MIN_GRIP_ANGLE` and `MAX_GRIP_ANGLE`. |
| Axis moves the wrong way | Reverse the direction level for that motor driver or adjust wiring. |
| Homing does not stop | Verify limit-switch wiring, `INPUT_PULLUP` behavior, and switch placement before enabling motor power. |
