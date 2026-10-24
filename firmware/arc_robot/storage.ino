void loadDefaults() {
    const int16_t cells[ARM_CELLS][3] = ARM_CELL_DEFAULTS;
    const int slotsMm[RAIL_SLOTS] = RAIL_SLOT_DEFAULT_MM;
    cal.magic = CAL_MAGIC;
    for (uint8_t i = 0; i < ARM_CELLS; i++) cal.cells[i] = {cells[i][0], cells[i][1], cells[i][2]};
    for (uint8_t i = 0; i < RAIL_SLOTS; i++) cal.railSteps[i] = (int32_t)(slotsMm[i] * RAIL_STEPS_PER_MM);
    cal.wristOffset = WRIST_LEVEL_OFFSET;
    cal.wristLevel = WRIST_LEVEL_ENABLED_AT_START;
    cal.cellTaught = 0;
    cal.railTaught = 0;
}

void saveCal() {
    prefs.putBytes("cal", &cal, sizeof(cal));
}

void loadCal() {
    if (prefs.getBytes("cal", &cal, sizeof(cal)) != sizeof(cal) || cal.magic != CAL_MAGIC) {
        loadDefaults();
        Serial.println(F("Калібрування: значення з config.h (у флеш ще нічого не записано)."));
    } else {
        Serial.println(F("Калібрування завантажено з флеш."));
    }
}

void restoreRail() {
    bool homed = prefs.getBool("rhomed", false);
    bool moving = prefs.getBool("rmov", false);
    if (homed && !moving) {
        rail.setCurrentPosition(prefs.getInt("rpos", 0));
        railHomed = true;
        Serial.printf("Позиція рейки відновлена: %.0f мм. Якщо платформу рухали руками — задайте нуль знову.\n", railMm());
    } else {
        rail.setCurrentPosition(0);
        railHomed = false;
        Serial.println(F("УВАГА: позиція рейки невідома. Підведіть платформу до нуля і утримуйте '#'."));
    }
}
