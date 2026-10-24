#include <Arduino.h>
#include <Preferences.h>
#include <Keypad.h>
#include <AccelStepper.h>

// ============================================================================
//  ARC 2026 — налаштування робота. Усе, що залежить від заліза, змінюється ТУТ.
//  Після складання робота пройдіть калібрування (README.md, розділ «Калібрування»).
// ============================================================================

// ---------------------------------------------------------------------------
// 1. Піни ESP32 (KS0413 / ESP32 Dev Module)
//    Не використовувати: 6–11 (flash), 12 (strapping, ламає завантаження),
//    34–39 лише вхід і без внутрішньої підтяжки.
// ---------------------------------------------------------------------------
// Кроковий 28BYJ-48 через ULN2003: IN1..IN4 на платі драйвера.
// GPIO 5 під час завантаження ESP32 коротко пульсує: двигун може ледь смикнутися, це безпечно.
#define PIN_STEPPER_IN1 16
#define PIN_STEPPER_IN2 17
#define PIN_STEPPER_IN3 5
#define PIN_STEPPER_IN4 18

// Сервоприводи (сигнальний дріт, зазвичай жовтий/помаранчевий)
// GPIO 14 і 15 видають ШІМ під час завантаження ESP32 -> серво на них смикались би
#define PIN_SERVO_BASE     13
#define PIN_SERVO_SHOULDER 33
#define PIN_SERVO_ELBOW    27
#define PIN_SERVO_WRIST    14
#define PIN_SERVO_CLAW     25

// Клавіатура 4x4: 8 дротів зліва направо = рядки 1..4, потім стовпці 1..4
#define KEYPAD_ROW_PINS {23, 22, 21, 4}
#define KEYPAD_COL_PINS {26, 14, 15, 19}

// Кінцевик нуля рейки (замикає на GND). -1 = кінцевика немає, нуль задається вручну.
// Якщо є: GPIO 34 + зовнішній резистор 10 кОм до 3.3 В.
#define PIN_RAIL_LIMIT -1

// ---------------------------------------------------------------------------
// 2. Сервоприводи SG90 / EMAX ES08MA II
// ---------------------------------------------------------------------------
// Ширина імпульсу для 0° і 180°. Зі стартовими межами нижче імпульси лишаються
// в 975–1925 мкс — безпечно для обох моделей. Якщо серво гуде на краю — звузити межі.
#define SERVO_US_0DEG   500
#define SERVO_US_180DEG 2400

// Безпечні межі кожного суглоба, градуси. Стартові значення навмисно вузькі
// (±30° від 90°, у якому руку збирають): у MeArm плече й лікоть зчеплені тягами,
// і широкі кути можуть упертися в механіку -> серво під навантаженням гріється.
// Розширюйте по 10° після перевірки клавішами, дивлячись у Serial Monitor.
#define BASE_MIN       45
#define BASE_MAX      135
#define SHOULDER_MIN   60
#define SHOULDER_MAX  120
#define ELBOW_MIN      60
#define ELBOW_MAX     120
#define WRIST_MIN      60
#define WRIST_MAX     120
#define CLAW_OPEN      90   // клешня відкрита
#define CLAW_CLOSED    60   // клешня стиснута на вантажі (не до упору, інакше серво гріється)

// Вихідна поза: 90° на всіх суглобах = положення, в якому руку збирали
#define SAFE_BASE      90
#define SAFE_SHOULDER  90
#define SAFE_ELBOW     90

// ---------------------------------------------------------------------------
// 3. Утримання горизонту клешні
//    У руці типу MeArm / Keyestudio 4DOF (база, 2 серво руки, клешня) горизонт
//    тримає механіка паралелограма, окреме серво зап'ястка не потрібне -> false.
//    Якщо зап'ясток є окремим серво, ставимо true і налаштовуємо формулу:
//    wrist = WRIST_LEVEL_OFFSET + WRIST_K_SHOULDER*shoulder + WRIST_K_ELBOW*elbow
//    Знаки залежать від того, як змонтовані серво. Порядок налаштування:
//    рухаєте плече клавішами 2/8 — якщо клешня нахиляється в той самий бік, що й
//    рука, міняєте знак WRIST_K_SHOULDER; те саме для ліктя (7/9).
//    Зсув підганяється з Serial Monitor клавішами '+' / '-' і зберігається.
// ---------------------------------------------------------------------------
#define HAS_WRIST_SERVO     false
#define WRIST_LEVEL_ENABLED_AT_START true
#define WRIST_LEVEL_OFFSET  270
#define WRIST_K_SHOULDER    (-1)
#define WRIST_K_ELBOW       (-1)

// ---------------------------------------------------------------------------
// 4. Швидкості рук
// ---------------------------------------------------------------------------
#define ARM_SPEED_DEG_S        45   // автоматичні переходи між позами
#define ARM_SLOW_SPEED_DEG_S   20   // опускання до вантажу та підйом з ним
#define CLAW_SPEED_DEG_S       60
#define GRIP_SETTLE_MS        400   // пауза після стиснення/розтиснення клешні

#define SHOULDER_UP_DIR      (+1)   // -1, якщо клавіша «2» опускає руку замість підняти
#define JOG_SPEED_DEG_S        40   // ручний режим: максимальна швидкість суглоба
#define JOG_ACCEL_DEG_S2      120   // ручний режим: розгін/гальмування (плавний старт і стоп)
#define SERIAL_JOG_STEP_DEG     5   // крок однієї команди з Serial Monitor

// Під час автоматичного підходу рука спершу стає над коміркою, потім опускається:
// «над коміркою» = поза комірки + ці зсуви (градуси)
#define APPROACH_SHOULDER_DELTA  10
#define APPROACH_ELBOW_DELTA      0

// ---------------------------------------------------------------------------
// 5. Лінійна платформа (28BYJ-48, напівкрок: 4096 кроків на оберт вала)
// ---------------------------------------------------------------------------
#define RAIL_STEPS_PER_MM    102.4f   // напр. шків GT2 20 зубів = 40 мм/оберт -> 4096/40
#define RAIL_LENGTH_MM       300      // робочий хід від нуля
#define RAIL_MAX_SPEED       800      // кроків/с; 28BYJ-48 на 5 В стабільно до ~900
#define RAIL_ACCEL           500      // кроків/с^2
#define RAIL_JOG_SPEED       600      // кроків/с у ручному режимі
#define SERIAL_JOG_STEP_MM   10       // крок однієї команди з Serial Monitor
#define RAIL_RELEASE_IDLE_MS 3000     // знеструмити котушки після простою (менше гріється й економить батареї)
#define RAIL_INVERT_DIR      false    // true, якщо «6» (від нуля) везе платформу до нуля

// ---------------------------------------------------------------------------
// 6. Координатна сітка (значення за замовчуванням; реальні записуються навчанням)
//    Позиції рейки A/C: номер 1..RAIL_SLOTS. Комірки руки B/D: номер 1..ARM_CELLS.
// ---------------------------------------------------------------------------
#define RAIL_SLOTS 4
#define RAIL_SLOT_DEFAULT_MM {20, 100, 180, 260}

#define ARM_CELLS 4
// {база, плече, лікоть} для кожної комірки — поза, в якій клешня охоплює вантаж
#define ARM_CELL_DEFAULTS { \
    {90,  75,  80}, /* 1 */ \
    {90,  75, 100}, /* 2 */ \
    {90, 100,  80}, /* 3 */ \
    {90, 100, 100}, /* 4 */ \
}

// Після циклу AB->CD повертати платформу в позицію 1 (true) чи лишати біля C (false)
#define AUTO_RETURN_RAIL_HOME false

// ---------------------------------------------------------------------------
// 7. Різне
// ---------------------------------------------------------------------------
#define SERIAL_BAUD        115200
#define SERVO_REFRESH_MS   20      // як часто оновлювати серво (50 Гц — їхня рідна частота)
#define ZERO_HOLD_MS       1000    // скільки тримати '#', щоб задати нуль рейки

// ---------------------------------------------------------------------------
// 8. Джойстик Keyestudio (контакти Y, X, B, V, G) для ручного режиму
//    V -> 3V3 ESP32 (НЕ шина 5 В: входи ESP32 витримують лише 3,3 В), G -> GND.
//    X — платформа ліворуч/праворуч, Y — плече вгору/вниз, B — клешня.
//    Якщо на старті джойстик не знайдено (не підключений, дріт відпав), він
//    вимикається сам, а керування лишається на клавіатурі.
// ---------------------------------------------------------------------------
#define JOYSTICK_ENABLED   true
#define PIN_JOY_X          34       // ADC1, лише вхід
#define PIN_JOY_Y          35       // ADC1, лише вхід
#define PIN_JOY_BUTTON     32       // кнопка B; -1 = не використовувати
#define JOY_INVERT_X       false    // true, якщо джойстик праворуч везе платформу ліворуч
#define JOY_INVERT_Y       false    // true, якщо джойстик угору опускає руку
#define JOY_DEADZONE       0.15f    // частка ходу ручки біля центру, яка ігнорується
#define JOY_POLL_MS        30

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

struct JoyAxis {
    uint8_t pin;
    bool invert;
    int center;
    float filtered;
    bool engaged;
};

namespace pin_check {

constexpr int KEYPAD_ROWS[] = KEYPAD_ROW_PINS;
constexpr int KEYPAD_COLS[] = KEYPAD_COL_PINS;

constexpr int OUTPUTS[] = {
    PIN_STEPPER_IN1, PIN_STEPPER_IN2, PIN_STEPPER_IN3, PIN_STEPPER_IN4,
    PIN_SERVO_BASE,  PIN_SERVO_SHOULDER, PIN_SERVO_ELBOW, PIN_SERVO_CLAW,
    HAS_WRIST_SERVO ? PIN_SERVO_WRIST : -1,
};

constexpr int INPUTS[] = {
    JOYSTICK_ENABLED ? PIN_JOY_X : -1,
    JOYSTICK_ENABLED ? PIN_JOY_Y : -1,
    JOYSTICK_ENABLED ? PIN_JOY_BUTTON : -1,
    PIN_RAIL_LIMIT,
};

constexpr int uses(int pin) {
    int n = 0;
    for (int p : KEYPAD_ROWS) n += p == pin;
    for (int p : KEYPAD_COLS) n += p == pin;
    for (int p : OUTPUTS) n += p == pin;
    for (int p : INPUTS) n += p == pin;
    return n;
}

constexpr bool unique() {
    for (int p : KEYPAD_ROWS) if (uses(p) != 1) return false;
    for (int p : KEYPAD_COLS) if (uses(p) != 1) return false;
    for (int p : OUTPUTS) if (p >= 0 && uses(p) != 1) return false;
    for (int p : INPUTS) if (p >= 0 && uses(p) != 1) return false;
    return true;
}

constexpr bool inputOnly(int pin) {
    return pin >= 34 && pin <= 39;
}

constexpr bool forbidden(int pin) {
    return (pin >= 6 && pin <= 11) || pin == 12 || pin == 0 || pin == 1 || pin == 3;
}

constexpr bool outputsValid() {
    for (int p : OUTPUTS) if (p >= 0 && (inputOnly(p) || forbidden(p))) return false;
    for (int p : KEYPAD_ROWS) if (inputOnly(p) || forbidden(p)) return false;
    for (int p : KEYPAD_COLS) if (inputOnly(p) || forbidden(p)) return false;
    return true;
}

constexpr bool joystickButtonValid() {
    return !JOYSTICK_ENABLED || PIN_JOY_BUTTON < 0 || !inputOnly(PIN_JOY_BUTTON);
}

static_assert(unique(), "Налаштування пінів: один GPIO призначено двом пристроям");
static_assert(outputsValid(), "Налаштування пінів: серво, двигун і клавіатура не можуть стояти на GPIO 0, 1, 3, 6-12, 34-39");
static_assert(joystickButtonValid(), "Налаштування пінів: кнопці джойстика потрібна підтяжка, GPIO 34-39 не підходять");

}

#define ENABLE_MIN_AND_MAX_CONSTRAINTS
#include "ServoEasing.hpp"

ServoEasing servoBase, servoShoulder, servoElbow, servoClaw;
Servo servoWrist;

#if RAIL_INVERT_DIR
AccelStepper rail(AccelStepper::HALF4WIRE, PIN_STEPPER_IN4, PIN_STEPPER_IN2, PIN_STEPPER_IN3, PIN_STEPPER_IN1);
#else
AccelStepper rail(AccelStepper::HALF4WIRE, PIN_STEPPER_IN1, PIN_STEPPER_IN3, PIN_STEPPER_IN2, PIN_STEPPER_IN4);
#endif

const char KEYMAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'},
};
byte keypadRowPins[4] = KEYPAD_ROW_PINS;
byte keypadColPins[4] = KEYPAD_COL_PINS;
Keypad keypad(makeKeymap(KEYMAP), keypadRowPins, keypadColPins, 4, 4);

Preferences prefs;

enum Joint : uint8_t { J_BASE, J_SHOULDER, J_ELBOW, J_WRIST, J_COUNT };
const int16_t JOINT_MIN[J_COUNT] = {BASE_MIN, SHOULDER_MIN, ELBOW_MIN, WRIST_MIN};
const int16_t JOINT_MAX[J_COUNT] = {BASE_MAX, SHOULDER_MAX, ELBOW_MAX, WRIST_MAX};

const uint32_t CAL_MAGIC = 0xA4C30000UL | (ARM_CELLS << 8) | RAIL_SLOTS;
const int32_t RAIL_MAX_STEPS = (int32_t)(RAIL_LENGTH_MM * RAIL_STEPS_PER_MM);

Calibration cal;
Mode mode = MODE_MANUAL;

Jog jog[J_COUNT];
float wristAngle = 90;
bool clawClosed = false;

bool railHomed = false;
bool railPowered = false;
bool railWasRunning = false;
uint32_t railIdleSince = 0;

char pendingPrefix = 0;
uint32_t pendingSince = 0;

uint8_t jobInput[4];
uint8_t jobInputLen = 0;
uint8_t lastJob[4];
bool lastJobValid = false;

const uint8_t MAX_STEPS = 20;
AutoStep steps[MAX_STEPS];
uint8_t stepCount = 0, stepIndex = 0;
bool stepStarted = false;
uint32_t stepStartMs = 0, stepTimeoutMs = 0;

JoyAxis joyX = {PIN_JOY_X, JOY_INVERT_X, 2048, 0, false};
JoyAxis joyY = {PIN_JOY_Y, JOY_INVERT_Y, 2048, 0, false};
bool joystickOk = false;
bool joyNeedsCenter = true;
int8_t joyRailDir = 0;
float joyRailSpeed = 0;

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
        Serial.println(F("Калібрування: значення за замовчуванням (у флеш ще нічого не записано)."));
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
    for (uint8_t i = 0; i < 4; i++) {
        bool isRail = i % 2 == 0;
        if (!((isRail ? cal.railTaught : cal.cellTaught) & (1u << (job[i] - 1)))) {
            Serial.printf("Не можу стартувати: %s %u ще не записана (ручний режим, '%c' + %u).\n",
                          isRail ? "позиція рейки" : "комірка", job[i], isRail ? 'B' : 'A', job[i]);
            mode = MODE_MANUAL;
            return;
        }
    }
    if (AUTO_RETURN_RAIL_HOME && !(cal.railTaught & 1u)) {
        Serial.println(F("Не можу стартувати: позиція рейки 1 (повернення) ще не записана ('B' + 1)."));
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
    rail.stop();
    stopArm();
    mode = MODE_MANUAL;
    joyNeedsCenter = true;
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
            Serial.println(F("Калібрування скинуто до значень за замовчуванням."));
        } else if (mode == MODE_MANUAL && !pendingPrefix && strchr("12346789", c)) {
            serialJog(c);
        } else if (mode == MODE_MANUAL && c == '#') {
            if (!rail.isRunning()) startHoming();
        } else {
            keyDown(c);
        }
    }
}

void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(SERIAL_BAUD);
    delay(300);
    Serial.println(F("\nARC 2026 robot, збірка " __DATE__ " " __TIME__));

    prefs.begin("arc", false);
    loadCal();

    servoBase.attach(PIN_SERVO_BASE, SAFE_BASE, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoShoulder.attach(PIN_SERVO_SHOULDER, SAFE_SHOULDER, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoElbow.attach(PIN_SERVO_ELBOW, SAFE_ELBOW, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    servoClaw.attach(PIN_SERVO_CLAW, CLAW_OPEN, SERVO_US_0DEG, SERVO_US_180DEG);
    delay(250);
    if (HAS_WRIST_SERVO) {
        servoWrist.setPeriodHertz(50);
        servoWrist.attach(PIN_SERVO_WRIST, SERVO_US_0DEG, SERVO_US_180DEG);
        writeWrist(cal.wristLevel ? levelWristAngle() : 90);
    }

    servoBase.setMinMaxConstraint(BASE_MIN, BASE_MAX);
    servoShoulder.setMinMaxConstraint(SHOULDER_MIN, SHOULDER_MAX);
    servoElbow.setMinMaxConstraint(ELBOW_MIN, ELBOW_MAX);
    servoClaw.setMinMaxConstraint(min(CLAW_OPEN, CLAW_CLOSED), max(CLAW_OPEN, CLAW_CLOSED));
    servoBase.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoShoulder.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoElbow.setEasingType(EASE_QUADRATIC_IN_OUT);
    servoClaw.setEasingType(EASE_QUADRATIC_IN_OUT);

    rail.setAcceleration(RAIL_ACCEL);
    rail.setMaxSpeed(RAIL_MAX_SPEED);
    rail.disableOutputs();
    restoreRail();

#if PIN_RAIL_LIMIT >= 0
    pinMode(PIN_RAIL_LIMIT, INPUT);
#endif

    keypad.setDebounceTime(20);
    keypad.setHoldTime(ZERO_HOLD_MS);
    setupJoystick();

    printHelp();
    printStatus();
}

void loop() {
    rail.run();
    pollKeypad();
    rail.run();
    pollSerial();
    rail.run();
    joystickTick();
    rail.run();
    servoTick();
    rail.run();
    railHousekeeping();

    if (pendingPrefix && millis() - pendingSince > 3000) {
        pendingPrefix = 0;
        Serial.println(F("Запис скасовано (немає цифри)."));
    }
    if (mode == MODE_AUTO_RUN) autoTick();
    else if (mode == MODE_HOMING) homingTick();
}
