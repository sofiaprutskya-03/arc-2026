void keyDown(char c) {
    switch (mode) {
    case MODE_MANUAL: manualKeyDown(c); break;
    case MODE_AUTO_INPUT: handleAutoInputKey(c); break;
    case MODE_AUTO_RUN: abortJob("натиснуто клавішу"); break;
    case MODE_HOMING:
        rail.stop();
        mode = MODE_MANUAL;
        Serial.println(F("Пошук нуля перервано."));
        break;
    }
}

void keyUp(char c) {
    if (mode == MODE_MANUAL) manualKeyUp(c);
}

void keyHold(char c) {
    if (mode == MODE_MANUAL && c == '#' && !rail.isRunning()) startHoming();
}

void pollKeypad() {
    if (!keypad.getKeys()) return;
    for (uint8_t i = 0; i < LIST_MAX; i++) {
        if (!keypad.key[i].stateChanged) continue;
        char c = keypad.key[i].kchar;
        switch (keypad.key[i].kstate) {
        case PRESSED: keyDown(c); break;
        case HOLD: keyHold(c); break;
        case RELEASED: keyUp(c); break;
        default: break;
        }
    }
}

// Serial не вміє «утримувати», тому клавіші руху з монітора роблять один крок
void serialJog(char c) {
    uint8_t j;
    int8_t dir;
    if (jointForKey(c, j, dir)) {
        float target = constrain(currentAngle(j) + dir * SERIAL_JOG_STEP_DEG, (float)JOINT_MIN[j], (float)JOINT_MAX[j]);
        easingServo(j).setEaseTo(target, JOG_SPEED_DEG_S);
    } else {
        int32_t delta = (int32_t)(SERIAL_JOG_STEP_MM * RAIL_STEPS_PER_MM) * (c == '4' ? -1 : 1);
        railMoveTo(rail.targetPosition() + delta, RAIL_JOG_SPEED);
    }
}

void pollSerial() {
    while (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == '\r' || c == '\n' || c == ' ') continue;
        if (c == '?') {
            printHelp();
        } else if ((c == '+' || c == '-') && HAS_WRIST_SERVO) {
            int d = c == '+' ? 1 : -1;
            if (cal.wristLevel) {
                cal.wristOffset += d;
                saveCal();
                Serial.printf("Зсув горизонту: %d\n", cal.wristOffset);
            } else {
                writeWrist(wristAngle + d * SERIAL_JOG_STEP_DEG);
                Serial.printf("Зап'ясток: %.0f\n", wristAngle);
            }
        } else if (c == 'X' && mode == MODE_MANUAL) {
            loadDefaults();
            saveCal();
            Serial.println(F("Калібрування скинуто до config.h."));
        } else if (mode == MODE_MANUAL && !pendingPrefix && strchr("12346789", c)) {
            serialJog(c);
        } else if (mode == MODE_MANUAL && c == '#') {
            if (!rail.isRunning()) startHoming();
        } else {
            keyDown(c);
        }
    }
}
