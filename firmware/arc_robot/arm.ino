ServoEasing &easingServo(uint8_t j) {
    switch (j) {
    case J_BASE: return servoBase;
    case J_SHOULDER: return servoShoulder;
    default: return servoElbow;
    }
}

float currentAngle(uint8_t j) {
    if (j == J_WRIST) return wristAngle;
    int us = easingServo(j).getCurrentMicroseconds();
    return (us - SERVO_US_0DEG) * 180.0f / (SERVO_US_180DEG - SERVO_US_0DEG);
}

bool wristLevelActive() {
    return HAS_WRIST_SERVO && cal.wristLevel;
}

void writeWrist(float deg) {
    if (!HAS_WRIST_SERVO) return;
    wristAngle = constrain(deg, (float)WRIST_MIN, (float)WRIST_MAX);
    int us = SERVO_US_0DEG + (int)(wristAngle * (SERVO_US_180DEG - SERVO_US_0DEG) / 180.0f + 0.5f);
    servoWrist.writeMicroseconds(us);
}

void writeJoint(uint8_t j, float deg) {
    if (j == J_WRIST) {
        writeWrist(deg);
    } else {
        easingServo(j).write(constrain(deg, (float)JOINT_MIN[j], (float)JOINT_MAX[j]));
    }
}

float levelWristAngle() {
    return cal.wristOffset + WRIST_K_SHOULDER * currentAngle(J_SHOULDER) + WRIST_K_ELBOW * currentAngle(J_ELBOW);
}

ArmPose clampPose(ArmPose p) {
    p.base = constrain(p.base, BASE_MIN, BASE_MAX);
    p.shoulder = constrain(p.shoulder, SHOULDER_MIN, SHOULDER_MAX);
    p.elbow = constrain(p.elbow, ELBOW_MIN, ELBOW_MAX);
    return p;
}

ArmPose safePose() {
    return {SAFE_BASE, SAFE_SHOULDER, SAFE_ELBOW};
}

ArmPose abovePose(ArmPose cell) {
    cell.shoulder += SHOULDER_UP_DIR * APPROACH_SHOULDER_DELTA;
    cell.elbow += APPROACH_ELBOW_DELTA;
    return clampPose(cell);
}

uint32_t moveArmTo(ArmPose target, uint16_t degPerSec) {
    target = clampPose(target);
    const int16_t goal[3] = {target.base, target.shoulder, target.elbow};
    float maxDelta = 0;
    for (uint8_t j = 0; j < 3; j++) {
        jog[j].cmd = 0;
        jog[j].vel = 0;
        jog[j].active = false;
        maxDelta = max(maxDelta, fabsf(goal[j] - currentAngle(j)));
    }
    uint32_t ms = max<uint32_t>(100, (uint32_t)(maxDelta * 1000.0f / degPerSec));
    for (uint8_t j = 0; j < 3; j++) {
        easingServo(j).setEaseToD(goal[j], (uint_fast16_t)ms);
    }
    return ms;
}

uint32_t moveClaw(bool close) {
    clawClosed = close;
    int target = close ? CLAW_CLOSED : CLAW_OPEN;
    uint32_t ms = max<uint32_t>(100, abs(target - servoClaw.getCurrentAngle()) * 1000UL / CLAW_SPEED_DEG_S);
    servoClaw.setEaseToD(target, (uint_fast16_t)ms);
    return ms;
}

void jogTick(float dt) {
    for (uint8_t j = 0; j < J_COUNT; j++) {
        Jog &g = jog[j];
        if (g.cmd == 0 && g.vel == 0) {
            g.active = false;
            continue;
        }
        if (j == J_WRIST && (!HAS_WRIST_SERVO || cal.wristLevel)) {
            g.cmd = 0;
            g.vel = 0;
            continue;
        }
        if (!g.active) {
            if (j != J_WRIST) easingServo(j).stop();
            g.pos = currentAngle(j);
            g.active = true;
        }
        float targetVel = constrain(g.cmd, -1.0f, 1.0f) * JOG_SPEED_DEG_S;
        float room = targetVel > 0 ? JOINT_MAX[j] - g.pos : g.pos - JOINT_MIN[j];
        float vBrake = sqrtf(2.0f * JOG_ACCEL_DEG_S2 * max(room, 0.0f));
        if (fabsf(targetVel) > vBrake) targetVel = copysignf(vBrake, targetVel);

        float dv = JOG_ACCEL_DEG_S2 * dt;
        if (g.vel < targetVel) g.vel = min(g.vel + dv, targetVel);
        else g.vel = max(g.vel - dv, targetVel);

        g.pos += g.vel * dt;
        if (g.pos <= JOINT_MIN[j] || g.pos >= JOINT_MAX[j]) {
            g.pos = constrain(g.pos, (float)JOINT_MIN[j], (float)JOINT_MAX[j]);
            g.vel = 0;
        }
        writeJoint(j, g.pos);
    }
}

void servoTick() {
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    if (now - lastMs < SERVO_REFRESH_MS) return;
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    if (dt > 0.1f) dt = 0.1f;

    jogTick(dt);
    updateAllServos();
    if (wristLevelActive()) writeWrist(levelWristAngle());
}

void stopArm() {
    stopAllServos();
    for (uint8_t j = 0; j < J_COUNT; j++) {
        jog[j].cmd = 0;
        jog[j].vel = 0;
        jog[j].active = false;
    }
}
