float railMm() {
    return rail.currentPosition() / RAIL_STEPS_PER_MM;
}

int32_t railMinSteps() {
    return railHomed ? 0 : -RAIL_MAX_STEPS;
}

int32_t railMaxSteps() {
    return RAIL_MAX_STEPS;
}

void saveRailState(bool moving) {
    prefs.putInt("rpos", rail.currentPosition());
    prefs.putBool("rmov", moving);
    prefs.putBool("rhomed", railHomed);
}

void railMoveTo(int32_t target, float speed) {
    target = constrain(target, railMinSteps(), railMaxSteps());
    if (!rail.isRunning() && target == rail.currentPosition()) return;
    if (!railPowered) {
        rail.enableOutputs();
        railPowered = true;
    }
    if (!rail.isRunning()) saveRailState(true);
    rail.setMaxSpeed(speed);
    rail.moveTo(target);
}

void setRailZero() {
    rail.setCurrentPosition(0);
    railHomed = true;
    saveRailState(false);
    Serial.println(F("Нуль рейки задано тут."));
}

void railHousekeeping() {
    bool running = rail.isRunning();
    if (railWasRunning && !running) {
        railIdleSince = millis();
        saveRailState(false);
    }
    railWasRunning = running;
    if (!running && railPowered && millis() - railIdleSince > RAIL_RELEASE_IDLE_MS) {
        rail.disableOutputs();
        railPowered = false;
    }
#if PIN_RAIL_LIMIT >= 0
    if (mode != MODE_HOMING && digitalRead(PIN_RAIL_LIMIT) == LOW && rail.speed() < 0) {
        rail.setCurrentPosition(0);
        railHomed = true;
        Serial.println(F("Кінцевик: стоп, нуль рейки оновлено."));
    }
#endif
}

void startHoming() {
#if PIN_RAIL_LIMIT >= 0
    Serial.println(F("Пошук нуля по кінцевику..."));
    railHomed = false;
    rail.setCurrentPosition(0);
    railMoveTo(-RAIL_MAX_STEPS * 12 / 10, RAIL_JOG_SPEED / 2);
    homingStartMs = millis();
    mode = MODE_HOMING;
#else
    setRailZero();
#endif
}

void homingTick() {
#if PIN_RAIL_LIMIT >= 0
    if (digitalRead(PIN_RAIL_LIMIT) == LOW) {
        setRailZero();
        railMoveTo((int32_t)(3 * RAIL_STEPS_PER_MM), RAIL_JOG_SPEED / 2);
        mode = MODE_MANUAL;
    } else if (!rail.isRunning()) {
        rail.setCurrentPosition(0);
        Serial.println(F("ПОМИЛКА: кінцевик не спрацював на всьому ході. Перевірте проводку."));
        mode = MODE_MANUAL;
    }
#endif
}
