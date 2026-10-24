#pragma once
#include <stdint.h>
#include "config.h"

struct ArmPose {
    int16_t base;
    int16_t shoulder;
    int16_t elbow;
};

enum Mode : uint8_t {
    MODE_MANUAL,
    MODE_AUTO_INPUT,
    MODE_AUTO_RUN,
    MODE_HOMING
};

enum StepKind : uint8_t { STEP_CLAW, STEP_ARM, STEP_RAIL, STEP_WAIT };

struct AutoStep {
    StepKind kind;
    ArmPose pose;
    int32_t value;
    uint16_t speed;
    const char *name;
};

struct Calibration {
    uint32_t magic;
    ArmPose cells[ARM_CELLS];
    int32_t railSteps[RAIL_SLOTS];
    int16_t wristOffset;
    uint8_t wristLevel;
    uint16_t cellTaught;
    uint16_t railTaught;
};

struct Jog {
    float pos;
    float vel;
    float cmd;
    bool active;
};
