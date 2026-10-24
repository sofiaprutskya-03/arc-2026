#pragma once
#include "config.h"

namespace pin_check {

constexpr int KEYPAD_ROWS[] = KEYPAD_ROW_PINS;
constexpr int KEYPAD_COLS[] = KEYPAD_COL_PINS;

constexpr int OUTPUTS[] = {
    PIN_STEPPER_IN1, PIN_STEPPER_IN2, PIN_STEPPER_IN3, PIN_STEPPER_IN4,
    PIN_SERVO_BASE,  PIN_SERVO_SHOULDER, PIN_SERVO_ELBOW, PIN_SERVO_CLAW,
    HAS_WRIST_SERVO ? PIN_SERVO_WRIST : -1,
};

constexpr int INPUTS[] = {
    JOYSTICK_ENABLED ? PIN_JOY_X : -1,
    JOYSTICK_ENABLED ? PIN_JOY_Y : -1,
    JOYSTICK_ENABLED ? PIN_JOY_BUTTON : -1,
    PIN_RAIL_LIMIT,
};

constexpr int uses(int pin) {
    int n = 0;
    for (int p : KEYPAD_ROWS) n += p == pin;
    for (int p : KEYPAD_COLS) n += p == pin;
    for (int p : OUTPUTS) n += p == pin;
    for (int p : INPUTS) n += p == pin;
    return n;
}

constexpr bool unique() {
    for (int p : KEYPAD_ROWS) if (uses(p) != 1) return false;
    for (int p : KEYPAD_COLS) if (uses(p) != 1) return false;
    for (int p : OUTPUTS) if (p >= 0 && uses(p) != 1) return false;
    for (int p : INPUTS) if (p >= 0 && uses(p) != 1) return false;
    return true;
}

constexpr bool inputOnly(int pin) {
    return pin >= 34 && pin <= 39;
}

constexpr bool forbidden(int pin) {
    return (pin >= 6 && pin <= 11) || pin == 12 || pin == 0 || pin == 1 || pin == 3;
}

constexpr bool outputsValid() {
    for (int p : OUTPUTS) if (p >= 0 && (inputOnly(p) || forbidden(p))) return false;
    for (int p : KEYPAD_ROWS) if (inputOnly(p) || forbidden(p)) return false;
    for (int p : KEYPAD_COLS) if (inputOnly(p) || forbidden(p)) return false;
    return true;
}

constexpr bool joystickButtonValid() {
    return !JOYSTICK_ENABLED || PIN_JOY_BUTTON < 0 || !inputOnly(PIN_JOY_BUTTON);
}

static_assert(unique(), "config.h: один GPIO призначено двом пристроям");
static_assert(outputsValid(), "config.h: серво, двигун і клавіатура не можуть стояти на GPIO 0, 1, 3, 6-12, 34-39");
static_assert(joystickButtonValid(), "config.h: кнопці джойстика потрібна підтяжка, GPIO 34-39 не підходять");

}
