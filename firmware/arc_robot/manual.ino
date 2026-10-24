bool jointForKey(char c, uint8_t &j, int8_t &dir) {
    switch (c) {
    case '1': j = J_BASE; dir = -1; return true;
    case '3': j = J_BASE; dir = +1; return true;
    case '2': j = J_SHOULDER; dir = SHOULDER_UP_DIR; return true;
    case '8': j = J_SHOULDER; dir = -SHOULDER_UP_DIR; return true;
    case '7': j = J_ELBOW; dir = -1; return true;
    case '9': j = J_ELBOW; dir = +1; return true;
    }
    return false;
}

void teach(char prefix, uint8_t n) {
    if (prefix == 'A') {
        if (n < 1 || n > ARM_CELLS) {
            Serial.printf("Комірка має бути 1..%u\n", ARM_CELLS);
            return;
        }
        cal.cells[n - 1] = {(int16_t)lroundf(currentAngle(J_BASE)), (int16_t)lroundf(currentAngle(J_SHOULDER)),
                            (int16_t)lroundf(currentAngle(J_ELBOW))};
        cal.cellTaught |= 1u << (n - 1);
        saveCal();
        Serial.printf("Комірка %u записана: база %d, плече %d, лікоть %d\n", n, cal.cells[n - 1].base,
                      cal.cells[n - 1].shoulder, cal.cells[n - 1].elbow);
    } else {
        if (n < 1 || n > RAIL_SLOTS) {
            Serial.printf("Позиція рейки має бути 1..%u\n", RAIL_SLOTS);
            return;
        }
        if (!railHomed) {
            Serial.println(F("Спершу задайте нуль рейки ('#')."));
            return;
        }
        cal.railSteps[n - 1] = rail.currentPosition();
        cal.railTaught |= 1u << (n - 1);
        saveCal();
        Serial.printf("Позиція рейки %u записана: %.1f мм\n", n, railMm());
    }
}

void manualKeyDown(char c) {
    if (pendingPrefix) {
        char prefix = pendingPrefix;
        pendingPrefix = 0;
        if (c >= '1' && c <= '9') {
            teach(prefix, c - '0');
            return;
        }
        Serial.println(F("Запис скасовано."));
    }
    uint8_t j;
    int8_t dir;
    if (jointForKey(c, j, dir)) {
        jog[j].cmd = dir;
        return;
    }
    switch (c) {
    case '4': railMoveTo(railMinSteps(), RAIL_JOG_SPEED); break;
    case '6': railMoveTo(railMaxSteps(), RAIL_JOG_SPEED); break;
    case '5': moveClaw(!clawClosed); break;
    case '0': moveArmTo(safePose(), ARM_SPEED_DEG_S); break;
    case 'A':
    case 'B':
        pendingPrefix = c;
        pendingSince = millis();
        Serial.printf("%c: натисніть номер (1..%u)\n", c, c == 'A' ? ARM_CELLS : RAIL_SLOTS);
        break;
    case 'C': printStatus(); break;
    case 'D':
        if (!HAS_WRIST_SERVO) {
            Serial.println(F("Серво зап'ястка немає: горизонт тримає механіка."));
            break;
        }
        cal.wristLevel = !cal.wristLevel;
        saveCal();
        Serial.println(cal.wristLevel ? F("Вирівнювання клешні: УВІМК.") : F("Вирівнювання клешні: ВИМК."));
        break;
    case '*': enterAutoInput(); break;
    }
}

void manualKeyUp(char c) {
    uint8_t j;
    int8_t dir;
    if (jointForKey(c, j, dir)) {
        if (jog[j].cmd == dir) jog[j].cmd = 0;
    } else if (c == '4' || c == '6') {
        rail.stop();
    }
}
