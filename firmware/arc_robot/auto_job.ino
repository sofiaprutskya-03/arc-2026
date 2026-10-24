void addStep(StepKind kind, const char *name, int32_t value = 0, ArmPose pose = {0, 0, 0}, uint16_t speed = 0) {
    if (stepCount < MAX_STEPS) steps[stepCount++] = {kind, pose, value, speed, name};
}

void buildJob(const uint8_t job[4]) {
    ArmPose pick = clampPose(cal.cells[job[1] - 1]);
    ArmPose place = clampPose(cal.cells[job[3] - 1]);
    stepCount = 0;
    addStep(STEP_CLAW, "відкрити клешню", 0);
    addStep(STEP_ARM, "безпечна поза", 0, safePose(), ARM_SPEED_DEG_S);
    addStep(STEP_RAIL, "платформа до A", cal.railSteps[job[0] - 1]);
    addStep(STEP_ARM, "над коміркою B", 0, abovePose(pick), ARM_SPEED_DEG_S);
    addStep(STEP_ARM, "опустити до вантажу", 0, pick, ARM_SLOW_SPEED_DEG_S);
    addStep(STEP_CLAW, "захват", 1);
    addStep(STEP_WAIT, "стабілізація захвату", GRIP_SETTLE_MS);
    addStep(STEP_ARM, "підняти вантаж", 0, abovePose(pick), ARM_SLOW_SPEED_DEG_S);
    addStep(STEP_ARM, "транспортна поза", 0, safePose(), ARM_SPEED_DEG_S);
    addStep(STEP_RAIL, "платформа до C", cal.railSteps[job[2] - 1]);
    addStep(STEP_ARM, "над коміркою D", 0, abovePose(place), ARM_SPEED_DEG_S);
    addStep(STEP_ARM, "опустити вантаж", 0, place, ARM_SLOW_SPEED_DEG_S);
    addStep(STEP_CLAW, "розтиснути", 0);
    addStep(STEP_WAIT, "відпустити вантаж", GRIP_SETTLE_MS);
    addStep(STEP_ARM, "відвести руку", 0, abovePose(place), ARM_SLOW_SPEED_DEG_S);
    addStep(STEP_ARM, "вихідна поза", 0, safePose(), ARM_SPEED_DEG_S);
    if (AUTO_RETURN_RAIL_HOME) addStep(STEP_RAIL, "платформа у вихідну", cal.railSteps[0]);
}

void startJob(const uint8_t job[4]) {
    if (!railHomed) {
        Serial.println(F("Не можу стартувати: нуль рейки не задано (ручний режим, утримати '#')."));
        mode = MODE_MANUAL;
        return;
    }
    memcpy(lastJob, job, 4);
    lastJobValid = true;
    buildJob(job);
    stepIndex = 0;
    stepStarted = false;
    mode = MODE_AUTO_RUN;
    Serial.printf("СТАРТ: A=%u B=%u -> C=%u D=%u. Будь-яка клавіша = СТОП.\n", job[0], job[1], job[2], job[3]);
}

void abortJob(const char *reason) {
    rail.stop();  // AccelStepper гальмує з заданим прискоренням, без ривка
    stopArm();
    mode = MODE_MANUAL;
    Serial.printf("ЗУПИНЕНО на кроці %u (%s): %s. Ручний режим.\n", stepIndex + 1,
                  stepIndex < stepCount ? steps[stepIndex].name : "-", reason);
}

void autoTick() {
    if (stepIndex >= stepCount) {
        mode = MODE_MANUAL;
        Serial.println(F("ГОТОВО. Цикл AB->CD завершено. '*' + '#' повторить його."));
        return;
    }
    AutoStep &s = steps[stepIndex];
    if (!stepStarted) {
        uint32_t expected = 0;
        switch (s.kind) {
        case STEP_CLAW: expected = moveClaw(s.value != 0); break;
        case STEP_ARM: expected = moveArmTo(s.pose, s.speed); break;
        case STEP_RAIL:
            railMoveTo(s.value, RAIL_MAX_SPEED);
            // час руху з розгоном і запасом: ціль не досягнута за цей час = щось не так
            expected = (uint32_t)(abs(s.value - rail.currentPosition()) * 1000.0f / RAIL_MAX_SPEED +
                                  2000.0f * RAIL_MAX_SPEED / RAIL_ACCEL);
            break;
        case STEP_WAIT: expected = s.value; break;
        }
        stepStartMs = millis();
        stepTimeoutMs = expected * 3 / 2 + 2000;
        stepStarted = true;
        Serial.printf("[%u/%u] %s\n", stepIndex + 1, stepCount, s.name);
        return;
    }

    bool done = false;
    switch (s.kind) {
    case STEP_CLAW:
    case STEP_ARM: done = !isOneServoMoving(); break;
    case STEP_RAIL: done = !rail.isRunning(); break;
    case STEP_WAIT: done = millis() - stepStartMs >= (uint32_t)s.value; break;
    }
    if (done) {
        stepIndex++;
        stepStarted = false;
    } else if (millis() - stepStartMs > stepTimeoutMs) {
        abortJob("перевищено час кроку");
    }
}

void promptJobInput() {
    static const char *const prompts[4] = {"A — позиція платформи, звідки брати", "B — комірка руки, звідки брати",
                                           "C — позиція платформи, куди везти", "D — комірка руки, куди класти"};
    if (jobInputLen < 4) {
        Serial.printf("  %s (1..%u): ", prompts[jobInputLen], (jobInputLen % 2 == 0) ? RAIL_SLOTS : ARM_CELLS);
    } else {
        Serial.printf("\n  Задача: A=%u B=%u C=%u D=%u. '#' старт, 'C' очистити, '*' назад.\n", jobInput[0], jobInput[1],
                      jobInput[2], jobInput[3]);
    }
}

void enterAutoInput() {
    stopArm();
    rail.stop();
    jobInputLen = 0;
    mode = MODE_AUTO_INPUT;
    Serial.println(F("\n=== НАПІВАВТОМАТ: введіть 4 цифри A B C D, потім '#'. '*' — назад у ручний ==="));
    if (lastJobValid)
        Serial.printf("  (одразу '#' повторить останню задачу %u%u%u%u)\n", lastJob[0], lastJob[1], lastJob[2], lastJob[3]);
    promptJobInput();
}

void handleAutoInputKey(char c) {
    if (c == '*') {
        mode = MODE_MANUAL;
        Serial.println(F("\nРучний режим."));
    } else if (c == 'C') {
        jobInputLen = 0;
        Serial.println();
        promptJobInput();
    } else if (c == '#') {
        if (jobInputLen == 4) startJob(jobInput);
        else if (jobInputLen == 0 && lastJobValid) startJob(lastJob);
        else Serial.println(F("\n  Потрібно 4 цифри."));
    } else if (c >= '1' && c <= '9' && jobInputLen < 4) {
        uint8_t n = c - '0';
        uint8_t limit = (jobInputLen % 2 == 0) ? RAIL_SLOTS : ARM_CELLS;
        if (n > limit) {
            Serial.printf("\n  %u поза межами 1..%u\n", n, limit);
            promptJobInput();
            return;
        }
        Serial.println(n);
        jobInput[jobInputLen++] = n;
        if (jobInputLen == 4 && jobInput[0] == jobInput[2] && jobInput[1] == jobInput[3]) {
            Serial.println(F("  Точка взяття збігається з точкою розвантаження."));
            jobInputLen = 3;
        }
        promptJobInput();
    }
}
