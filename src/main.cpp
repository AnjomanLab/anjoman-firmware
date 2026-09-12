#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & POLARITY
// ==============================================================================
constexpr float    VBAT_DIVIDER_RATIO  = 4.0000f;
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  AS5600_ADDR         = 0x36;
constexpr uint8_t  AS5600_ANGLE_REG    = 0x0E;
constexpr uint32_t I2C_CLOCK_FREQ_HZ   = 400000;
constexpr uint32_t PWM_FREQ_HZ         = 20000;
constexpr uint8_t  PWM_RES_BITS        = 10;
constexpr uint32_t STEP_DURATION_MS    = 1400; // 1.4s per step

constexpr bool INVERT_ENC_LEFT  = true;
constexpr bool INVERT_ENC_RIGHT = false;

#if ROBOT_ID == 1
    constexpr bool INVERT_MOTOR_L = false;
    constexpr bool INVERT_MOTOR_R = true;
#elif ROBOT_ID == 2
    constexpr bool INVERT_MOTOR_L = true;
    constexpr bool INVERT_MOTOR_R = true;
#elif ROBOT_ID == 3
    constexpr bool INVERT_MOTOR_L = false;
    constexpr bool INVERT_MOTOR_R = true;
#elif ROBOT_ID == 4
    constexpr bool INVERT_MOTOR_L = true;
    constexpr bool INVERT_MOTOR_R = false;
#endif

enum MotorMode : uint8_t {
    MODE_BRAKE     = 0,
    MODE_COAST     = 1,
    MODE_DRIVE_FWD = 2,
    MODE_DRIVE_REV = 3
};

struct EncoderChannel {
    int32_t cumulativeSteps = 0;
    int16_t lastRawAngle = 0;
    int16_t lastDelta = 0;
    uint16_t currentRawAngle = 0;
    bool isFirstRead = true;
};

EncoderChannel encL;
EncoderChannel encR;

// 11 Fine Steps: 0.40 to 0.70
constexpr float fineSteps[11] = {
    0.40f, 0.43f, 0.46f, 0.49f, 0.52f, 0.55f, 
    0.58f, 0.61f, 0.64f, 0.67f, 0.70f
};

// 5 Synchronous Levels
constexpr float syncLevels[5] = {0.43f, 0.49f, 0.55f, 0.61f, 0.67f};

SPIClass SPI_SD(HSPI);
File logFile;
bool sdReady = false;

#pragma pack(push, 1)
struct TelemetryRecord {
    uint64_t timeUs;
    uint8_t  phase;
    float    pwmL;
    float    pwmR;
    uint8_t  modeL;
    uint8_t  modeR;
    uint16_t rawAngL;
    uint16_t rawAngR;
    int16_t  deltaL;
    int16_t  deltaR;
    int32_t  stepsL;
    int32_t  stepsR;
    float    vbat;
};
#pragma pack(pop)

QueueHandle_t sdQueue = nullptr;
TaskHandle_t  core0LoggerHandle = nullptr;

// ==============================================================================
// 2. ENCODER HAL
// ==============================================================================
#if ROBOT_HAS_TCA9548A
bool selectTCA(uint8_t ch) {
    if (ch > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    return (Wire.endTransmission() == 0);
}

uint16_t readAS5600(uint8_t ch) {
    if (!selectTCA(ch)) return 0xFFFF;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_REG);
    if (Wire.endTransmission() != 0) return 0xFFFF;

    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        return ((uint16_t)(msb & 0x0F) << 8) | lsb;
    }
    return 0xFFFF;
}

void pollEncoders() {
    uint16_t rawL = readAS5600(0);
    if (rawL != 0xFFFF) {
        encL.currentRawAngle = rawL;
        int16_t cur = (int16_t)rawL;
        if (encL.isFirstRead) { encL.lastRawAngle = cur; encL.isFirstRead = false; }
        else {
            int16_t d = cur - encL.lastRawAngle;
            if (d > 2048) d -= 4096;
            if (d < -2048) d += 4096;
            encL.lastDelta = -d;
            encL.cumulativeSteps += encL.lastDelta;
            encL.lastRawAngle = cur;
        }
    }

    uint16_t rawR = readAS5600(1);
    if (rawR != 0xFFFF) {
        encR.currentRawAngle = rawR;
        int16_t cur = (int16_t)rawR;
        if (encR.isFirstRead) { encR.lastRawAngle = cur; encR.isFirstRead = false; }
        else {
            int16_t d = cur - encR.lastRawAngle;
            if (d > 2048) d -= 4096;
            if (d < -2048) d += 4096;
            encR.lastDelta = d;
            encR.cumulativeSteps += encR.lastDelta;
            encR.lastRawAngle = cur;
        }
    }
}
#else
uint16_t readAS5600Direct(TwoWire &bus) {
    bus.beginTransmission(AS5600_ADDR);
    bus.write(AS5600_ANGLE_REG);
    if (bus.endTransmission() != 0) return 0xFFFF;

    bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (bus.available() >= 2) {
        uint8_t msb = bus.read();
        uint8_t lsb = bus.read();
        return ((uint16_t)(msb & 0x0F) << 8) | lsb;
    }
    return 0xFFFF;
}

void pollEncoders() {
    uint16_t rawL = readAS5600Direct(Wire);
    if (rawL != 0xFFFF) {
        encL.currentRawAngle = rawL;
        int16_t cur = (int16_t)rawL;
        if (encL.isFirstRead) { encL.lastRawAngle = cur; encL.isFirstRead = false; }
        else {
            int16_t d = cur - encL.lastRawAngle;
            if (d > 2048) d -= 4096;
            if (d < -2048) d += 4096;
            encL.lastDelta = -d;
            encL.cumulativeSteps += encL.lastDelta;
            encL.lastRawAngle = cur;
        }
    }

    uint16_t rawR = readAS5600Direct(Wire1);
    if (rawR != 0xFFFF) {
        encR.currentRawAngle = rawR;
        int16_t cur = (int16_t)rawR;
        if (encR.isFirstRead) { encR.lastRawAngle = cur; encR.isFirstRead = false; }
        else {
            int16_t d = cur - encR.lastRawAngle;
            if (d > 2048) d -= 4096;
            if (d < -2048) d += 4096;
            encR.lastDelta = d;
            encR.cumulativeSteps += encR.lastDelta;
            encR.lastRawAngle = cur;
        }
    }
}
#endif

// ==============================================================================
// 3. MOTOR DRIVER
// ==============================================================================
void setupMotors() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    ledcAttach(PIN_MOTOR_L_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_L_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_R_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_R_IN2, PWM_FREQ_HZ, PWM_RES_BITS);

    ledcWrite(PIN_MOTOR_L_IN1, 1023);
    ledcWrite(PIN_MOTOR_L_IN2, 1023);
    ledcWrite(PIN_MOTOR_R_IN1, 1023);
    ledcWrite(PIN_MOTOR_R_IN2, 1023);
}

void commandMotor(uint8_t pin1, uint8_t pin2, float duty, bool invert, MotorMode &outMode) {
    if (invert) duty = -duty;
    duty = constrain(duty, -1.0f, 1.0f);

    uint32_t val = (uint32_t)(fabs(duty) * 1023.0f);

    if (duty > 0.01f) {
        ledcWrite(pin1, val);
        ledcWrite(pin2, 0);
        outMode = MODE_DRIVE_FWD;
    } else if (duty < -0.01f) {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, val);
        outMode = MODE_DRIVE_REV;
    } else {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, 0);
        outMode = MODE_COAST;
    }
}

void applyBrake() {
    ledcWrite(PIN_MOTOR_L_IN1, 1023);
    ledcWrite(PIN_MOTOR_L_IN2, 1023);
    ledcWrite(PIN_MOTOR_R_IN1, 1023);
    ledcWrite(PIN_MOTOR_R_IN2, 1023);
}

// ==============================================================================
// 4. BATTERY VOLTAGE SAMPLER
// ==============================================================================
float readBatteryPack() {
    uint32_t sumMv = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sumMv += analogReadMilliVolts(PIN_VBAT_SENSE);
    }
    return (((float)sumMv / 4.0f) * 1e-3f) * VBAT_DIVIDER_RATIO;
}

// ==============================================================================
// 5. CORE 0 ASYNCHRONOUS SD LOGGER TASK
// ==============================================================================
void core0LoggerTask(void *pvParameters) {
    TelemetryRecord rec;
    uint32_t writeCount = 0;

    while (true) {
        if (xQueueReceive(sdQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdReady && logFile) {
                logFile.printf("%llu,%u,%.3f,%.3f,%u,%u,%u,%u,%d,%d,%ld,%ld,%.3f\n",
                               (unsigned long long)rec.timeUs, rec.phase,
                               rec.pwmL, rec.pwmR, rec.modeL, rec.modeR,
                               rec.rawAngL, rec.rawAngR, rec.deltaL, rec.deltaR,
                               (long)rec.stepsL, (long)rec.stepsR, rec.vbat);
                writeCount++;
                if (writeCount % 50 == 0) logFile.flush();
            }

            Serial.printf("%llu,%u,%.3f,%.3f,%u,%u,%u,%u,%d,%d,%ld,%ld,%.2f\n",
                          (unsigned long long)rec.timeUs, rec.phase,
                          rec.pwmL, rec.pwmR, rec.modeL, rec.modeR,
                          rec.rawAngL, rec.rawAngR, rec.deltaL, rec.deltaR,
                          (long)rec.stepsL, (long)rec.stepsR, rec.vbat);
        }
    }
}

// ==============================================================================
// 6. SETUP & REAL-TIME EXECUTION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

    analogSetAttenuation(ADC_11db);
    pinMode(PIN_VBAT_SENSE, INPUT);

#if ROBOT_HAS_TCA9548A
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
#else
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_CLOCK_FREQ_HZ);
#endif
    delay(20);

    setupMotors();

    // Init SPI3 MicroSD
    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, -1);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 4000000)) {
        sdReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/sysid_r%d.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("TimeUs,Phase,PwmCmdL,PwmCmdR,ModeL,ModeR,RawAngL,RawAngR,DeltaL,DeltaR,StepsL,StepsR,VbatV");
            logFile.flush();
        }
    }

    sdQueue = xQueueCreate(128, sizeof(TelemetryRecord));
    xTaskCreatePinnedToCore(core0LoggerTask, "SDWorker", 4096, nullptr, 1, &core0LoggerHandle, 0);

    Serial.printf("[INIT] Robot %d ready. SD: %s | ADC: GPIO %d\n", Config::ID, sdReady ? "OK" : "NO_SD", PIN_VBAT_SENSE);
    Serial.println("TimeUs,Phase,PwmCmdL,PwmCmdR,ModeL,ModeR,RawAngL,RawAngR,DeltaL,DeltaR,StepsL,StepsR,VbatV");

    // 5-second initial safety pause (allows placing robot on floor)
    delay(5000);
}

void loop() {
    static uint32_t startTimeMs = millis();
    static bool testDone = false;

    if (testDone) {
        applyBrake();
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - startTimeMs;
    uint64_t nowUs = micros();

    pollEncoders();
    float vbat = readBatteryPack();

    float cmdL = 0.0f;
    float cmdR = 0.0f;
    uint8_t phase = 0;

    // Automated 78-Second Sequence (0.40 to 0.70 PWM)
    if (elapsedMs < 3000) {
        // Phase 0: Standstill Baseline (3s)
        phase = 0; cmdL = 0.0f; cmdR = 0.0f;
    } else if (elapsedMs < 18400) {
        // Phase 1: Left Fwd (11 steps x 1.4s = 15.4s)
        phase = 1; cmdR = 0.0f;
        uint32_t idx = (elapsedMs - 3000) / STEP_DURATION_MS;
        cmdL = (idx < 11) ? fineSteps[idx] : fineSteps[10];
    } else if (elapsedMs < 33800) {
        // Phase 2: Left Rev (15.4s)
        phase = 2; cmdR = 0.0f;
        uint32_t idx = (elapsedMs - 18400) / STEP_DURATION_MS;
        cmdL = (idx < 11) ? -fineSteps[idx] : -fineSteps[10];
    } else if (elapsedMs < 49200) {
        // Phase 3: Right Fwd (15.4s)
        phase = 3; cmdL = 0.0f;
        uint32_t idx = (elapsedMs - 33800) / STEP_DURATION_MS;
        cmdR = (idx < 11) ? fineSteps[idx] : fineSteps[10];
    } else if (elapsedMs < 64600) {
        // Phase 4: Right Rev (15.4s)
        phase = 4; cmdL = 0.0f;
        uint32_t idx = (elapsedMs - 49200) / STEP_DURATION_MS;
        cmdR = (idx < 11) ? -fineSteps[idx] : -fineSteps[10];
    } else if (elapsedMs < 71600) {
        // Phase 5: Both Fwd (5 steps x 1.4s = 7.0s) -> Straight Forward ~80cm
        phase = 5;
        uint32_t idx = (elapsedMs - 64600) / STEP_DURATION_MS;
        float v = (idx < 5) ? syncLevels[idx] : syncLevels[4];
        cmdL = v; cmdR = v;
    } else if (elapsedMs < 78600) {
        // Phase 6: Both Rev (5 steps x 1.4s = 7.0s) -> Return ~80cm
        phase = 6;
        uint32_t idx = (elapsedMs - 71600) / STEP_DURATION_MS;
        float v = (idx < 5) ? -syncLevels[idx] : -syncLevels[4];
        cmdL = v; cmdR = v;
    } else {
        // Test Complete: Hard Brake
        phase = 7;
        testDone = true;
        applyBrake();
        if (logFile) {
            logFile.flush();
            logFile.close();
        }
        Serial.printf("[COMPLETE] Robot %d test finished. File saved to SD.\n", Config::ID);
        return;
    }

    // Command Motors
    MotorMode modeL = MODE_BRAKE;
    MotorMode modeR = MODE_BRAKE;
    commandMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, cmdL, INVERT_MOTOR_L, modeL);
    commandMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, cmdR, INVERT_MOTOR_R, modeR);

    // Push Record to Core 0 Queue
    TelemetryRecord rec = {};
    rec.timeUs = nowUs;
    rec.phase = phase;
    rec.pwmL = cmdL;
    rec.pwmR = cmdR;
    rec.modeL = (uint8_t)modeL;
    rec.modeR = (uint8_t)modeR;
    rec.rawAngL = encL.currentRawAngle;
    rec.rawAngR = encR.currentRawAngle;
    rec.deltaL = encL.lastDelta;
    rec.deltaR = encR.lastDelta;
    rec.stepsL = encL.cumulativeSteps;
    rec.stepsR = encR.cumulativeSteps;
    rec.vbat = vbat;

    if (sdQueue != nullptr) {
        xQueueSend(sdQueue, &rec, 0);
    }

    delay(10); // 100 Hz deterministic loop
}
