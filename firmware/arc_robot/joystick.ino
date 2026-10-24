bool calibrateAxis(JoyAxis &a) {
    long sum = 0;
    int lo = 4095, hi = 0;
    for (uint8_t i = 0; i < 32; i++) {
        int v = analogRead(a.pin);
        sum += v;
        lo = min(lo, v);
        hi = max(hi, v);
        delay(2);
    }
    a.center = sum / 32;
    a.filtered = 0;
    a.engaged = false;
    return a.center > 1000 && a.center < 3100 && hi - lo < 200;
}

void setupJoystick() {
    if (!JOYSTICK_ENABLED) return;
#if PIN_JOY_BUTTON >= 0
    pinMode(PIN_JOY_BUTTON, INPUT_PULLUP);
#endif
    analogReadResolution(12);
    bool okX = calibrateAxis(joyX);
    bool okY = calibrateAxis(joyY);
    joystickOk = okX && okY;
    if (joystickOk) {
        Serial.printf("Джойстик: центр X=%d Y=%d. X — платформа, Y — плече, кнопка — клешня.\n", joyX.center,
                      joyY.center);
    } else {
        Serial.printf("Джойстик ВИМКНЕНО: на старті немає стабільного центру (X=%d Y=%d). "
                      "Перевірте V->3V3, G->GND, X->%d, Y->%d і не чіпайте ручку під час старту.\n",
                      joyX.center, joyY.center, PIN_JOY_X, PIN_JOY_Y);
    }
}

float readAxis(JoyAxis &a) {
    int raw = analogRead(a.pin);
    float span = raw >= a.center ? 4095 - a.center : a.center;
    float v = constrain((raw - a.center) / span, -1.0f, 1.0f);
    if (a.invert) v = -v;
    a.filtered += 0.4f * (v - a.filtered);
    float m = fabsf(a.filtered);
    if (m < JOY_DEADZONE) return 0;
    float s = (m - JOY_DEADZONE) / (1.0f - JOY_DEADZONE);
    return copysignf(s * s, a.filtered);
}

bool joyButtonPressed() {
#if PIN_JOY_BUTTON >= 0
    static bool stable = HIGH, last = HIGH;
    static uint32_t changedMs = 0;
    bool level = digitalRead(PIN_JOY_BUTTON);
    if (level != last) {
        last = level;
        changedMs = millis();
    }
    if (millis() - changedMs >= 30 && level != stable) {
        stable = level;
        return stable == LOW;
    }
#endif
    return false;
}

void joyDriveShoulder(float y) {
    if (y != 0) {
        jog[J_SHOULDER].cmd = y * SHOULDER_UP_DIR;
        joyY.engaged = true;
    } else if (joyY.engaged) {
        jog[J_SHOULDER].cmd = 0;
        joyY.engaged = false;
    }
}

void joyDriveRail(float x) {
    if (x == 0) {
        if (joyX.engaged) rail.stop();
        joyX.engaged = false;
        return;
    }
    int8_t dir = x > 0 ? 1 : -1;
    float speed = max(0.15f, fabsf(x)) * RAIL_JOG_SPEED;
    if (!joyX.engaged || dir != joyRailDir || fabsf(speed - joyRailSpeed) > 0.05f * RAIL_JOG_SPEED) {
        railMoveTo(dir > 0 ? railMaxSteps() : railMinSteps(), speed);
        joyRailDir = dir;
        joyRailSpeed = speed;
    }
    joyX.engaged = true;
}

void joystickTick() {
    if (!joystickOk) return;
    static uint32_t lastMs = 0;
    if (millis() - lastMs < JOY_POLL_MS) return;
    lastMs = millis();

    float x = readAxis(joyX);
    float y = readAxis(joyY);
    bool pressed = joyButtonPressed();

    if (mode == MODE_AUTO_RUN) {
        if (fabsf(x) > 0.5f || fabsf(y) > 0.5f || pressed) abortJob("рух джойстика");
        return;
    }
    if (mode != MODE_MANUAL) return;
    if (joyNeedsCenter) {
        if (x == 0 && y == 0) joyNeedsCenter = false;
        return;
    }
    if (pressed) moveClaw(!clawClosed);
    joyDriveRail(x);
    joyDriveShoulder(y);
}
