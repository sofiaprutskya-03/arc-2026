void printHelp() {
    Serial.println(F(
        "\n=== РУЧНИЙ РЕЖИМ ===\n"
        " 4/6  платформа ліворуч/праворуч (тримати)\n"
        " 2/8  рука вгору/вниз (плече)     1/3  база ліво/право\n"
        " 7/9  лікоть                      5    клешня відкр./закр.\n"
        " 0    безпечна поза               #    утримати 1 с = нуль рейки\n"
        " A+цифра  запамʼятати позу руки як комірку N\n"
        " B+цифра  запамʼятати позицію платформи як N\n"
        " C    показати стан і таблицю      D    вирівнювання (якщо є серво зап'ястка)\n"
        " *    напівавтомат AB->CD\n"
        "Джойстик: X — платформа, Y — плече, кнопка — клешня; у напівавтоматі рух ручки = СТОП\n"
        "Serial: ті самі символи (крок замість утримання), +/- зап'ясток (якщо є), x скинути калібрування, ? довідка"));
}

void printStatus() {
    Serial.printf("Рейка: %.1f мм (%ld кр.)%s | база %.0f  плече %.0f  лікоть %.0f | клешня %s\n",
                  railMm(), (long)rail.currentPosition(), railHomed ? "" : " [нуль НЕ задано]",
                  currentAngle(J_BASE), currentAngle(J_SHOULDER), currentAngle(J_ELBOW),
                  clawClosed ? "закрита" : "відкрита");
    Serial.printf("  джойстик: %s\n", joystickOk ? "увімкнено" : "вимкнено");
    if (HAS_WRIST_SERVO)
        Serial.printf("  зап'ясток %.0f%s, зсув горизонту %d\n", wristAngle, cal.wristLevel ? " (авто)" : "",
                      cal.wristOffset);
    for (uint8_t i = 0; i < RAIL_SLOTS; i++)
        Serial.printf("  рейка %u: %.1f мм%s\n", i + 1, cal.railSteps[i] / RAIL_STEPS_PER_MM,
                      cal.railTaught & (1u << i) ? "" : " (не записано)");
    for (uint8_t i = 0; i < ARM_CELLS; i++)
        Serial.printf("  комірка %u: база %d, плече %d, лікоть %d%s\n", i + 1, cal.cells[i].base, cal.cells[i].shoulder,
                      cal.cells[i].elbow, cal.cellTaught & (1u << i) ? "" : " (не записано)");
}
