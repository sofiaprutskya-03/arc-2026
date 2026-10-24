#include <Arduino.h>
#include <Preferences.h>
#include <Keypad.h>
#include <AccelStepper.h>

#include "config.h"
#include "robot_types.h"
#include "pin_check.h"

#define ENABLE_MIN_AND_MAX_CONSTRAINTS
#include "ServoEasing.hpp"

ServoEasing servoBase, servoShoulder, servoElbow, servoClaw;
Servo servoWrist;

#if RAIL_INVERT_DIR
AccelStepper rail(AccelStepper::HALF4WIRE, PIN_STEPPER_IN4, PIN_STEPPER_IN2, PIN_STEPPER_IN3, PIN_STEPPER_IN1);
#else
AccelStepper rail(AccelStepper::HALF4WIRE, PIN_STEPPER_IN1, PIN_STEPPER_IN3, PIN_STEPPER_IN2, PIN_STEPPER_IN4);
#endif

const char KEYMAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'},
};
byte keypadRowPins[4] = KEYPAD_ROW_PINS;
byte keypadColPins[4] = KEYPAD_COL_PINS;
Keypad keypad(makeKeymap(KEYMAP), keypadRowPins, keypadColPins, 4, 4);

Preferences prefs;

enum Joint : uint8_t { J_BASE, J_SHOULDER, J_ELBOW, J_WRIST, J_COUNT };
const int16_t JOINT_MIN[J_COUNT] = {BASE_MIN, SHOULDER_MIN, ELBOW_MIN, WRIST_MIN};
const int16_t JOINT_MAX[J_COUNT] = {BASE_MAX, SHOULDER_MAX, ELBOW_MAX, WRIST_MAX};
const char *const JOINT_NAME[J_COUNT] = {"база", "плече", "лікоть", "зап'ясток"};

const uint32_t CAL_MAGIC = 0xA4C30000UL | (ARM_CELLS << 8) | RAIL_SLOTS;
const int32_t RAIL_MAX_STEPS = (int32_t)(RAIL_LENGTH_MM * RAIL_STEPS_PER_MM);

Calibration cal;
Mode mode = MODE_MANUAL;

Jog jog[J_COUNT];
float wristAngle = 90;
bool clawClosed = false;

bool railHomed = false;
bool railPowered = false;
bool railWasRunning = false;
uint32_t railIdleSince = 0;

char pendingPrefix = 0;
uint32_t pendingSince = 0;

uint8_t jobInput[4];
uint8_t jobInputLen = 0;
uint8_t lastJob[4];
bool lastJobValid = false;

const uint8_t MAX_STEPS = 20;
AutoStep steps[MAX_STEPS];
uint8_t stepCount = 0, stepIndex = 0;
bool stepStarted = false;
uint32_t stepStartMs = 0, stepTimeoutMs = 0;

uint32_t homingStartMs = 0;

JoyAxis joyX = {PIN_JOY_X, JOY_INVERT_X, 2048, 0, false};
JoyAxis joyY = {PIN_JOY_Y, JOY_INVERT_Y, 2048, 0, false};
bool joystickOk = false;
bool joyNeedsCenter = true;
int8_t joyRailDir = 0;
float joyRailSpeed = 0;

void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(SERIAL_BAUD);
    delay(300);
    Serial.println(F("\nARC 2026 robot, збірка " __DATE__ " " __TIME__));

    prefs.begin("arc", false);
    loadCal();

    servoBase.attach(PIN_SERVO_BASE, SAFE_BASE, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoShoulder.attach(PIN_SERVO_SHOULDER, SAFE_SHOULDER, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoElbow.attach(PIN_SERVO_ELBOW, SAFE_ELBOW, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoClaw.attach(PIN_SERVO_CLAW, CLAW_OPEN, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    if (HAS_WRIST_SERVO) {
        servoWrist.setPeriodHertz(50);
        servoWrist.attach(PIN_SERVO_WRIST, SERVO_US_0DEG, SERVO_US_180DEG);
        writeWrist(cal.wristLevel ? levelWristAngle() : 90);
    }

    servoBase.setMinMaxConstraint(BASE_MIN, BASE_MAX);
    servoShoulder.setMinMaxConstraint(SHOULDER_MIN, SHOULDER_MAX);
    servoElbow.setMinMaxConstraint(ELBOW_MIN, ELBOW_MAX);
    servoClaw.setMinMaxConstraint(min(CLAW_OPEN, CLAW_CLOSED), max(CLAW_OPEN, CLAW_CLOSED));
    servoBase.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoShoulder.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoElbow.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoClaw.setEasingType(EASE_QUADRATIC_IN_OUT);

    rail.setAcceleration(RAIL_ACCEL);
    rail.setMaxSpeed(RAIL_MAX_SPEED);
    rail.disableOutputs();
    restoreRail();

#if PIN_RAIL_LIMIT >= 0
    pinMode(PIN_RAIL_LIMIT, INPUT);
#endif

    keypad.setDebounceTime(20);
    keypad.setHoldTime(ZERO_HOLD_MS);
    setupJoystick();

    printHelp();
    printStatus();
}

void loop() {
    rail.run();
    pollKeypad();
    rail.run();
    pollSerial();
    rail.run();
    joystickTick();
    rail.run();
    servoTick();
    rail.run();
    railHousekeeping();

    if (pendingPrefix && millis() - pendingSince > 3000) {
        pendingPrefix = 0;
        Serial.println(F("Запис скасовано (немає цифри)."));
    }
    if (mode == MODE_AUTO_RUN) autoTick();
    else if (mode == MODE_HOMING) homingTick();
}
