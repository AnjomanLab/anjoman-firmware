#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & SENSOR ADDRESSES
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  AS5600_ADDR         = 0x36;
constexpr uint8_t  AS5600_ANGLE_REG    = 0x0E;
constexpr uint8_t  BMI160_DEFAULT_ADDR = 0x69;
constexpr uint8_t  INA226_ADDR         = 0x40;

constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t PWM_FREQ_HZ        = 20000;
constexpr uint8_t  PWM_RES_BITS       = 10;
constexpr uint32_t SAMPLE_PERIOD_US   = 10000;  // 100 Hz strict sampling (10 ms)
constexpr uint32_t TEST_DURATION_MS   = 150000; // Exactly 150 seconds (2.5 minutes)

// Verified Shunt Resistor: R010 = 0.010 Ohm
constexpr float    SHUNT_RESISTOR_OHM  = 0.010f; 
constexpr float    VBAT_DIVIDER_RATIO  = 133.0f / 33.0f; // 100k + 33k divider

// Dedicated SPI3 instance for MicroSD
SPIClass SPI_SD(HSPI);
File logFile;
bool sdCardReady = false;

uint8_t detectedBMI160Addr = BMI160_DEFAULT_ADDR;
bool ina226Ready = false;
bool bmi160Ready = false;

enum MotorMode : uint8_t {
    MODE_BRAKE     = 0,
    MODE_COAST     = 1,
    MODE_DRIVE_FWD = 2,
    MODE_DRIVE_REV = 3
};

struct EncoderTracker {
    int32_t  cumulativeSteps = 0;
    int16_t  lastRawAngle = 0;
    int16_t  lastDelta = 0;
    uint16_t currentRawAngle = 0;
    bool     isFirstRead = true;
    float    currentRPM = 0.0f;
    int32_t  lastSteps = 0;
    uint32_t lastSpeedTimeMs = 0;
};

EncoderTracker encL;
EncoderTracker encR;

// ==============================================================================
// 2. 33-COLUMN TELEMETRY RECORD STRUCTURE
// ==============================================================================
#pragma pack(push, 1)
struct SysIDRecord {
    uint64_t timestamp_us;
    uint8_t  test_id;
    uint8_t  phase;
    uint8_t  run_id;
    float    cmd_v;
    float    cmd_omega;
    float    pwm_l;
    float    pwm_r;
    uint8_t  mode_l;
    uint8_t  mode_r;
    uint16_t raw_angle_l;
    uint16_t raw_angle_r;
    int16_t  delta_angle_l;
    int16_t  delta_angle_r;
    int32_t  steps_l;
    int32_t  steps_r;
    float    rpm_l;
    float    rpm_r;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    float    imu_temperature;
    float    battery_voltage;
    float    driver_voltage;
    float    driver_current;
    float    driver_power;
    uint8_t  encoder_valid_l;
    uint8_t  encoder_valid_r;
    uint8_t  imu_valid;
    uint8_t  ina_valid;
};
#pragma pack(pop)

QueueHandle_t sdLogQueue = nullptr;
TaskHandle_t  core0TaskHandle = nullptr;

// ==============================================================================
// 3. HARDWARE I2C & SENSOR DRIVERS
// ==============================================================================
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

bool initINA226() {
    if (!selectTCAChannel(3)) return false;
    Wire.beginTransmission(INA226_ADDR);
    if (Wire.endTransmission() != 0) return false;

    // Reset INA226
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00); Wire.write(0x80); Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Continuous Shunt and Bus, 1.1ms conversion, 4 averages (0x4227)
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00); Wire.write(0x42); Wire.write(0x27);
    return (Wire.endTransmission() == 0);
}

bool readINA226(float &vbus_V, float &current_mA, float &power_mW) {
    vbus_V = current_mA = power_mW = 0.0f;
    if (!selectTCAChannel(3)) return false;

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint16_t rawBus = (Wire.read() << 8) | Wire.read();
    vbus_V = (float)rawBus * 0.00125f;

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    int16_t rawShunt = (int16_t)((Wire.read() << 8) | Wire.read());
    float vshunt_mV = (float)rawShunt * 0.0025f;

    // Exact I = Vshunt / 0.010 Ohm
    current_mA = vshunt_mV / SHUNT_RESISTOR_OHM;
    power_mW = vbus_V * current_mA;
    return true;
}

bool initBMI160() {
    if (!selectTCAChannel(2)) return false;
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) detectedBMI160Addr = 0x69;
    else {
        Wire.beginTransmission(0x68);
        if (Wire.endTransmission() == 0) detectedBMI160Addr = 0x68;
        else return false;
    }

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0xB6); // Soft Reset
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x11); // Accel normal
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x15); // Gyro normal
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x43); Wire.write(0x00); // 2000 dps
    Wire.endTransmission();

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x41); Wire.write(0x03); // 2g
    Wire.endTransmission();
    return true;
}

bool readBMI160(float &gx, float &gy, float &gz, float &ax, float &ay, float &az, float &tempC) {
    gx = gy = gz = ax = ay = az = tempC = 0.0f;
    if (!selectTCAChannel(2)) return false;

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)12);
    if (Wire.available() < 12) return false;

    int16_t raw_gx = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t raw_gy = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t raw_gz = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t raw_ax = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t raw_ay = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t raw_az = (int16_t)(Wire.read() | (Wire.read() << 8));

    gx = (float)raw_gx / 16.4f;
    gy = (float)raw_gy / 16.4f;
    gz = (float)raw_gz / 16.4f;
    ax = (float)raw_ax * 9.80665f / 16384.0f;
    ay = (float)raw_ay * 9.80665f / 16384.0f;
    az = (float)raw_az * 9.80665f / 16384.0f;

    // Read Internal Temperature (Reg 0x20, 0x21)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x20);
    if (Wire.endTransmission() == 0) {
        Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw_t = (int16_t)(Wire.read() | (Wire.read() << 8));
            tempC = 23.0f + ((float)raw_t / 512.0f);
        }
    }
    return true;
}

uint16_t readAS5600Angle(uint8_t channel) {
    if (!selectTCAChannel(channel)) return 0xFFFF;
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

void pollEncoders(bool &validL, bool &validR) {
    uint32_t nowMs = millis();
    float dt = (float)(nowMs - encL.lastSpeedTimeMs) / 1000.0f;

    // Left Encoder (Ch0 - Inverted)
    uint16_t rawL = readAS5600Angle(0);
    validL = (rawL != 0xFFFF);
    if (validL) {
        encL.currentRawAngle = rawL;
        int16_t cur = (int16_t)rawL;
        if (encL.isFirstRead) {
            encL.lastRawAngle = cur;
            encL.isFirstRead = false;
        } else {
            int16_t delta = cur - encL.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encL.lastDelta = -delta; // Invert Left
            encL.cumulativeSteps += encL.lastDelta;
            encL.lastRawAngle = cur;
        }
    }

    // Right Encoder (Ch1 - Natural)
    uint16_t rawR = readAS5600Angle(1);
    validR = (rawR != 0xFFFF);
    if (validR) {
        encR.currentRawAngle = rawR;
        int16_t cur = (int16_t)rawR;
        if (encR.isFirstRead) {
            encR.lastRawAngle = cur;
            encR.isFirstRead = false;
        } else {
            int16_t delta = cur - encR.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encR.lastDelta = delta; // Natural Right
            encR.cumulativeSteps += encR.lastDelta;
            encR.lastRawAngle = cur;
        }
    }

    if (dt >= 0.05f) {
        encL.currentRPM = ((float)(encL.cumulativeSteps - encL.lastSteps) / Config::ENCODER_CPR) * (60.0f / dt);
        encR.currentRPM = ((float)(encR.cumulativeSteps - encR.lastSteps) / Config::ENCODER_CPR) * (60.0f / dt);
        encL.lastSteps = encL.cumulativeSteps;
        encR.lastSteps = encR.cumulativeSteps;
        encL.lastSpeedTimeMs = nowMs;
    }
}

// ==============================================================================
// 4. MOTOR DRIVER (DRV8833 PWM)
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

    // Initial Brake
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
// 5. CORE 0 ASYNCHRONOUS MICROSD LOGGER
// ==============================================================================
void core0LoggerTask(void *pvParameters) {
    SysIDRecord rec;
    uint32_t recordsLogged = 0;

    while (true) {
        if (xQueueReceive(sdLogQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdCardReady && logFile) {
                logFile.printf("%llu,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%d,%d,%ld,%ld,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u,%u,%u\n",
                               (unsigned long long)rec.timestamp_us,
                               rec.test_id, rec.phase, rec.run_id,
                               rec.cmd_v, rec.cmd_omega,
                               rec.pwm_l, rec.pwm_r,
                               rec.mode_l, rec.mode_r,
                               rec.raw_angle_l, rec.raw_angle_r,
                               rec.delta_angle_l, rec.delta_angle_r,
                               (long)rec.steps_l, (long)rec.steps_r,
                               rec.rpm_l, rec.rpm_r,
                               rec.accel_x, rec.accel_y, rec.accel_z,
                               rec.gyro_x, rec.gyro_y, rec.gyro_z,
                               rec.imu_temperature,
                               rec.battery_voltage, rec.driver_voltage,
                               rec.driver_current, rec.driver_power,
                               rec.encoder_valid_l, rec.encoder_valid_r,
                               rec.imu_valid, rec.ina_valid);

                recordsLogged++;
                if (recordsLogged % 100 == 0) {
                    logFile.flush();
                }
            }
        }
    }
}

// ==============================================================================
// 6. COMPREHENSIVE 150-SECOND GROUND SYSID SEQUENCE
// ==============================================================================
void runGroundSysIDSequence(uint32_t elapsedMs, float &cmdL, float &cmdR, uint8_t &phase, bool &forceBrake) {
    forceBrake = false;

    // 14 Fine Levels (0.10 to 0.90) for accurate deadband identification
    constexpr float fineLevels[14] = {
        0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.35f, 0.40f, 
        0.45f, 0.50f, 0.55f, 0.60f, 0.65f, 0.75f, 0.90f
    };

    // Phase 0: REST (0 to 5s) -> 2.5s Brake, 2.5s Coast Baseline
    if (elapsedMs < 5000) {
        phase = 0;
        cmdL = 0.0f; cmdR = 0.0f;
        if (elapsedMs < 2500) forceBrake = true;
    }
    // Phase 1: Left Motor Fine Forward Sweep (5 to 21.8s) [14 steps x 1.2s]
    else if (elapsedMs < 21800) {
        phase = 1;
        cmdR = 0.0f;
        uint32_t stepIdx = (elapsedMs - 5000) / 1200;
        cmdL = (stepIdx < 14) ? fineLevels[stepIdx] : 0.90f;
    }
    // Phase 2: Left Motor Fine Reverse Sweep (21.8 to 38.6s) [14 steps x 1.2s]
    else if (elapsedMs < 38600) {
        phase = 2;
        cmdR = 0.0f;
        uint32_t stepIdx = (elapsedMs - 21800) / 1200;
        cmdL = (stepIdx < 14) ? -fineLevels[stepIdx] : -0.90f;
    }
    // Phase 3: Right Motor Fine Forward Sweep (38.6 to 55.4s) [14 steps x 1.2s]
    else if (elapsedMs < 55400) {
        phase = 3;
        cmdL = 0.0f;
        uint32_t stepIdx = (elapsedMs - 38600) / 1200;
        cmdR = (stepIdx < 14) ? fineLevels[stepIdx] : 0.90f;
    }
    // Phase 4: Right Motor Fine Reverse Sweep (55.4 to 72.2s) [14 steps x 1.2s]
    else if (elapsedMs < 72200) {
        phase = 4;
        cmdL = 0.0f;
        uint32_t stepIdx = (elapsedMs - 55400) / 1200;
        cmdR = (stepIdx < 14) ? -fineLevels[stepIdx] : -0.90f;
    }
    // Phase 5: Left Dynamic Step Transient (tau_m) (72.2 to 86.2s = 14s)
    // 0 -> +0.50 (3s) -> Brake (2s) -> +0.80 (3s) -> Brake (2s) -> -0.60 (3s) -> Brake (1s)
    else if (elapsedMs < 86200) {
        phase = 5;
        cmdR = 0.0f;
        uint32_t subMs = elapsedMs - 72200;
        if (subMs < 3000)        cmdL = 0.50f;
        else if (subMs < 5000)  { cmdL = 0.0f; forceBrake = true; }
        else if (subMs < 8000)   cmdL = 0.80f;
        else if (subMs < 10000) { cmdL = 0.0f; forceBrake = true; }
        else if (subMs < 13000)  cmdL = -0.60f;
        else                    { cmdL = 0.0f; forceBrake = true; }
    }
    // Phase 6: Right Dynamic Step Transient (tau_m) (86.2 to 100.2s = 14s)
    else if (elapsedMs < 100200) {
        phase = 6;
        cmdL = 0.0f;
        uint32_t subMs = elapsedMs - 86200;
        if (subMs < 3000)        cmdR = 0.50f;
        else if (subMs < 5000)  { cmdR = 0.0f; forceBrake = true; }
        else if (subMs < 8000)   cmdR = 0.80f;
        else if (subMs < 10000) { cmdR = 0.0f; forceBrake = true; }
        else if (subMs < 13000)  cmdR = -0.60f;
        else                    { cmdR = 0.0f; forceBrake = true; }
    }
    // Phase 7: Synchronous Forward & Reverse Steps for Asymmetry (100.2 to 120.2s = 20s)
    // +0.45 (2.5s) -> -0.45 (2.5s) -> +0.60 (2.5s) -> -0.60 (2.5s) -> +0.75 (2.5s) -> -0.75 (2.5s) -> +0.90 (2.5s) -> -0.90 (2.5s)
    else if (elapsedMs < 120200) {
        phase = 7;
        uint32_t subMs = elapsedMs - 100200;
        float val = (subMs < 2500)  ? +0.45f : (subMs < 5000)  ? -0.45f :
                    (subMs < 7500)  ? +0.60f : (subMs < 10000) ? -0.60f :
                    (subMs < 12500) ? +0.75f : (subMs < 15000) ? -0.75f :
                    (subMs < 17500) ? +0.90f : -0.90f;
        cmdL = val; cmdR = val;
    }
    // Phase 8: Coast-Down Friction Test (120.2 to 136.2s = 16s)
    // Drive +0.75 (4s) -> COAST until stop (4s) -> Drive -0.75 (4s) -> COAST until stop (4s)
    else if (elapsedMs < 136200) {
        phase = 8;
        uint32_t subMs = elapsedMs - 120200;
        if (subMs < 4000)       { cmdL = 0.75f;  cmdR = 0.75f; }
        else if (subMs < 8000)  { cmdL = 0.00f;  cmdR = 0.00f; } // Coasting (Mode 1)
        else if (subMs < 12000) { cmdL = -0.75f; cmdR = -0.75f; }
        else                    { cmdL = 0.00f;  cmdR = 0.00f; } // Coasting (Mode 1)
    }
    // Phase 9: TERMINATION & BRAKE (136.2s to 150s)
    else {
        phase = 9;
        cmdL = 0.0f; cmdR = 0.0f;
        forceBrake = true;
    }
}

// ==============================================================================
// 7. SETUP & INITIALIZATION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    // Initial LED State: Dim Amber (Running Test)
    neopixelWrite(PIN_STATUS_RGB, 20, 10, 0);

    // 1. Initialize Analog Battery Divider
    analogReadResolution(12);
    pinMode(PIN_VBAT_SENSE, INPUT);

    // 2. Initialize Dedicated I2C Master Bus
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(50);

    // 3. Initialize Sensors
    bmi160Ready = initBMI160();
    ina226Ready = initINA226();
    setupMotors();

    // 4. Initialize Dedicated SPI3 MicroSD Card
    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 10000000)) {
        sdCardReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/sysid_r%d.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("timestamp_us,test_id,phase,run_id,cmd_v,cmd_omega,pwm_l,pwm_r,mode_l,mode_r,raw_angle_l,raw_angle_r,delta_angle_l,delta_angle_r,steps_l,steps_r,rpm_l,rpm_r,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,imu_temperature,battery_voltage,driver_voltage,driver_current,driver_power,encoder_valid_l,encoder_valid_r,imu_valid,ina_valid");
            logFile.flush();
        }
    }

    // 5. Start Core 0 Asynchronous Logger Task
    sdLogQueue = xQueueCreate(256, sizeof(SysIDRecord));
    xTaskCreatePinnedToCore(
        core0LoggerTask,
        "SDLoggerTask",
        4096,
        nullptr,
        1,
        &core0TaskHandle,
        0
    );

    Serial.println("======================================================================");
    Serial.printf("   ANJOMAN FIRMWARE - COMPREHENSIVE GROUND SYSID (ROBOT %d)\n", Config::ID);
    Serial.printf("   MicroSD File: /sysid_r%d.csv | Status: %s\n", Config::ID, sdCardReady ? "READY" : "FAILED");
    Serial.println("   No Wi-Fi Stack. Automatic LED Completion on GPIO 48.");
    Serial.println("======================================================================");
}

// ==============================================================================
// 8. 100 HZ HARD REAL-TIME LOOP (CORE 1)
// ==============================================================================
void loop() {
    static uint32_t startTestTimeMs = millis();
    static uint64_t nextSampleUs = micros();
    static bool testComplete = false;

    if (testComplete) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint64_t nowUs = micros();
    uint32_t elapsedMs = millis() - startTestTimeMs;

    // Strict 100 Hz Sampling Interval (every 10,000 microseconds)
    if (nowUs >= nextSampleUs) {
        nextSampleUs = nowUs + SAMPLE_PERIOD_US;

        // 1. Poll Sensors
        bool valL = false, valR = false;
        pollEncoders(valL, valR);

        float gx, gy, gz, ax, ay, az, imuTemp;
        bool imuOk = readBMI160(gx, gy, gz, ax, ay, az, imuTemp);

        float vbus_V, current_mA, power_mW;
        bool inaOk = readINA226(vbus_V, current_mA, power_mW);

        uint32_t rawMv = analogReadMilliVolts(PIN_VBAT_SENSE);
        float vbat_adc = ((float)rawMv * VBAT_DIVIDER_RATIO) / 1000.0f;

        // 2. State Machine Actuator Excitation
        float cmdL = 0.0f, cmdR = 0.0f;
        uint8_t phase = 0;
        bool forceBrake = false;
        runGroundSysIDSequence(elapsedMs, cmdL, cmdR, phase, forceBrake);

        // 3. Command Motors
        MotorMode modeL = MODE_BRAKE;
        MotorMode modeR = MODE_BRAKE;

        if (forceBrake) {
            applyBrake();
            modeL = MODE_BRAKE;
            modeR = MODE_BRAKE;
        } else {
            commandMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, cmdL, Config::INVERT_MOTOR_LEFT, modeL);
            commandMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, cmdR, Config::INVERT_MOTOR_RIGHT, modeR);
        }

        // 4. Pack into 33-Column Record and Push to MicroSD Queue
        SysIDRecord rec = {};
        rec.timestamp_us = nowUs;
        rec.test_id = 1;
        rec.phase = phase;
        rec.run_id = 1;
        rec.cmd_v = (cmdL + cmdR) / 2.0f * 0.15f;
        rec.cmd_omega = (cmdR - cmdL) * 0.5f;
        rec.pwm_l = cmdL;
        rec.pwm_r = cmdR;
        rec.mode_l = (uint8_t)modeL;
        rec.mode_r = (uint8_t)modeR;
        rec.raw_angle_l = encL.currentRawAngle;
        rec.raw_angle_r = encR.currentRawAngle;
        rec.delta_angle_l = encL.lastDelta;
        rec.delta_angle_r = encR.lastDelta;
        rec.steps_l = encL.cumulativeSteps;
        rec.steps_r = encR.cumulativeSteps;
        rec.rpm_l = encL.currentRPM;
        rec.rpm_r = encR.currentRPM;
        rec.accel_x = ax;
        rec.accel_y = ay;
        rec.accel_z = az;
        rec.gyro_x = gx;
        rec.gyro_y = gy;
        rec.gyro_z = gz;
        rec.imu_temperature = imuTemp;
        rec.battery_voltage = vbat_adc;
        rec.driver_voltage = vbus_V;
        rec.driver_current = current_mA;
        rec.driver_power = power_mW;
        rec.encoder_valid_l = valL ? 1 : 0;
        rec.encoder_valid_r = valR ? 1 : 0;
        rec.imu_valid = imuOk ? 1 : 0;
        rec.ina_valid = inaOk ? 1 : 0;

        if (sdLogQueue != nullptr) {
            xQueueSend(sdLogQueue, &rec, 0); // Non-blocking
        }

        // 5. Automatic Completion at 150 Seconds
        if (elapsedMs >= TEST_DURATION_MS) {
            testComplete = true;
            applyBrake();

            vTaskDelay(pdMS_TO_TICKS(200));
            if (logFile) {
                logFile.flush();
                logFile.close();
            }

            // Turn WS2812 RGB LED to SOLID BRIGHT GREEN (Safe to Power Off)
            neopixelWrite(PIN_STATUS_RGB, 0, 60, 0);

            Serial.println("\n======================================================================");
            Serial.printf("[TEST COMPLETE] 150s Ground SysID Finished for Robot %d.\n", Config::ID);
            Serial.println("[LED STATUS] RGB LED is now SOLID GREEN. Safe to power off or remove SD.");
            Serial.println("======================================================================");
        }
    }
    yield();
}
