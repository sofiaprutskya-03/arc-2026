// Типи даних прошивки. Винесені в окремий файл, щоб Arduino IDE коректно
// згенерувала прототипи функцій.
#pragma once
#include <stdint.h>
#include "config.h"

struct ArmPose {
    int16_t base;
    int16_t shoulder;
    int16_t elbow;
};

enum Mode : uint8_t {
    MODE_MANUAL,      // керування з клавіатури / Serial у реальному часі
    MODE_AUTO_INPUT,  // введення A B C D
    MODE_AUTO_RUN,    // виконання циклу AB -> CD
    MODE_HOMING       // пошук нуля рейки по кінцевику
};

enum StepKind : uint8_t { STEP_CLAW, STEP_ARM, STEP_RAIL, STEP_WAIT };

struct AutoStep {
    StepKind kind;
    ArmPose pose;      // для STEP_ARM
    int32_t value;     // кут клешні / кроки рейки / мс очікування
    uint16_t speed;    // град/с для STEP_ARM
    const char *name;
};

// Калібрування, що зберігається у флеш (Preferences/NVS)
struct Calibration {
    uint32_t magic;
    ArmPose cells[ARM_CELLS];
    int32_t railSteps[RAIL_SLOTS];
    int16_t wristOffset;
    uint8_t wristLevel;
};

// Стан плавного ручного руху одного суглоба
struct Jog {
    float pos;
    float vel;
    int8_t dir;
    bool active;
};
