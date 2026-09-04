#include <Arduino.h>
#include <Wire.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS
// ==============================================================================
constexpr uint8_t TCA9548A_ADDR       = 0x70;
constexpr uint8_t AS5600_ADDR         = 0x36;
constexpr uint8_t AS5600_ANGLE_REG    = 0x0E;
constexpr uint8_t BMI160_DEFAULT_ADDR = 0x69; // Or 0x68 depending on SDO pin
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t PWM_FREQ_HZ        = 20000;  // 20 kHz ultrasonic PWM
constexpr uint8_t  PWM_RES_BITS       = 10;     // 10-bit resolution (0 - 1023)

// Encoder State Tracking
struct EncoderState {
    int32_t cumulativeSteps = 0;
    int16_t lastRawAngle = 0;
    bool    isFirstRead = true;
};

EncoderState encL;
EncoderState encR;

uint8_t detectedBMI160Addr = BMI160_DEFAULT_ADDR;

// ==============================================================================
// 2. HARDWARE ABSTRACTION LAYER (HAL) FOR I2C SENSORS
// ==============================================================================
#if ROBOT_HAS_TCA9548A
// --- Multiplexer Routing for Robots 1, 3, and 4 ---
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

uint16_t readAS5600RawAngle(uint8_t tcaChannel) {
    if (!selectTCAChannel(tcaChannel)) return 0xFFFF;
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

void updateEncoders() {
    // Read Left Encoder on TCA Channel 0
    uint16_t rawL = readAS5600RawAngle(0);
    if (rawL != 0xFFFF) {
        int16_t currentRaw = (int16_t)rawL;
        if (encL.isFirstRead) {
            encL.lastRawAngle = currentRaw;
            encL.isFirstRead = false;
        } else {
            int16_t delta = currentRaw - encL.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encL.cumulativeSteps += delta;
            encL.lastRawAngle = currentRaw;
        }
    }

    // Read Right Encoder on TCA Channel 1
    uint16_t rawR = readAS5600RawAngle(1);
    if (rawR != 0xFFFF) {
        int16_t currentRaw = (int16_t)rawR;
        if (encR.isFirstRead) {
            encR.lastRawAngle = currentRaw;
            encR.isFirstRead = false;
        } else {
            int16_t delta = currentRaw - encR.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encR.cumulativeSteps += delta;
            encR.lastRawAngle = currentRaw;
        }
    }
}

bool initBMI160() {
    if (!selectTCAChannel(2)) return false;
    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x7E); Wire.write(0xB6); // Soft reset
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x7E); Wire.write(0x11); // Accel normal mode
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x7E); Wire.write(0x15); // Gyro normal mode
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x43); Wire.write(0x00); // 2000 deg/s
    Wire.endTransmission();

    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x41); Wire.write(0x03); // 2g
    Wire.endTransmission();
    return true;
}

void readBMI160Data(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
    gx = gy = gz = ax = ay = az = 0.0f;
    if (!selectTCAChannel(2)) return;

    Wire.beginTransmission(BMI160_DEFAULT_ADDR);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return;

    Wire.requestFrom((uint8_t)BMI160_DEFAULT_ADDR, (uint8_t)12);
    if (Wire.available() >= 12) {
        int16_t raw_gx = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_gy = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_gz = (int16_t)(Wire.read() | (Wire.read() << 8));

        int16_t raw_ax = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_ay = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_az = (int16_t)(Wire.read() | (Wire.read() << 8));

        constexpr float GYRO_SCALE  = 1.0f / 16.4f;
        constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;

        gx = (float)raw_gx * GYRO_SCALE;
        gy = (float)raw_gy * GYRO_SCALE;
        gz = (float)raw_gz * GYRO_SCALE;
        ax = (float)raw_ax * ACCEL_SCALE;
        ay = (float)raw_ay * ACCEL_SCALE;
        az = (float)raw_az * ACCEL_SCALE;
    }
}

#else
// --- Direct Dual-I2C Architecture for Robot 2 ---
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

void updateEncoders() {
    // Left Encoder on Primary I2C (Wire on GPIO 7, 8)
    uint16_t rawL = readAS5600Direct(Wire);
    if (rawL != 0xFFFF) {
        int16_t currentRaw = (int16_t)rawL;
        if (encL.isFirstRead) {
            encL.lastRawAngle = currentRaw;
            encL.isFirstRead = false;
        } else {
            int16_t delta = currentRaw - encL.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encL.cumulativeSteps += delta;
            encL.lastRawAngle = currentRaw;
        }
    }

    // Right Encoder on Secondary I2C (Wire1 on GPIO 4, 6)
    uint16_t rawR = readAS5600Direct(Wire1);
    if (rawR != 0xFFFF) {
        int16_t currentRaw = (int16_t)rawR;
        if (encR.isFirstRead) {
            encR.lastRawAngle = currentRaw;
            encR.isFirstRead = false;
        } else {
            int16_t delta = currentRaw - encR.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            encR.cumulativeSteps += delta;
            encR.lastRawAngle = currentRaw;
        }
    }
}

bool initBMI160() {
    // Detect Address on Primary I2C Bus
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() != 0) {
        Wire.beginTransmission(0x68);
        if (Wire.endTransmission() == 0) {
            detectedBMI160Addr = 0x68;
        } else {
            return false;
        }
    }

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0xB6);
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x11);
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x15);
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x43); Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x41); Wire.write(0x03);
    Wire.endTransmission();
    return true;
}

void readBMI160Data(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
    gx = gy = gz = ax = ay = az = 0.0f;

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return;

    Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)12);
    if (Wire.available() >= 12) {
        int16_t raw_gx = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_gy = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_gz = (int16_t)(Wire.read() | (Wire.read() << 8));

        int16_t raw_ax = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_ay = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_az = (int16_t)(Wire.read() | (Wire.read() << 8));

        constexpr float GYRO_SCALE  = 1.0f / 16.4f;
        constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;

        gx = (float)raw_gx * GYRO_SCALE;
        gy = (float)raw_gy * GYRO_SCALE;
        gz = (float)raw_gz * GYRO_SCALE;
        ax = (float)raw_ax * ACCEL_SCALE;
        ay = (float)raw_ay * ACCEL_SCALE;
        az = (float)raw_az * ACCEL_SCALE;
    }
}
#endif

// ==============================================================================
// 3. MOTOR DRIVER (DRV8833 DUAL H-BRIDGE PWM)
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

    // Initial Hard Brake
    ledcWrite(PIN_MOTOR_L_IN1, 1023);
    ledcWrite(PIN_MOTOR_L_IN2, 1023);
    ledcWrite(PIN_MOTOR_R_IN1, 1023);
    ledcWrite(PIN_MOTOR_R_IN2, 1023);
}

void setMotorSpeeds(float dutyL, float dutyR) {
    dutyL = constrain(dutyL, -1.0f, 1.0f);
    dutyR = constrain(dutyR, -1.0f, 1.0f);

    uint32_t valL = (uint32_t)(fabs(dutyL) * 1023.0f);
    uint32_t valR = (uint32_t)(fabs(dutyR) * 1023.0f);

    // Left Motor Control
    if (dutyL > 0.01f) {
        ledcWrite(PIN_MOTOR_L_IN1, valL);
        ledcWrite(PIN_MOTOR_L_IN2, 0);
    } else if (dutyL < -0.01f) {
        ledcWrite(PIN_MOTOR_L_IN1, 0);
        ledcWrite(PIN_MOTOR_L_IN2, valL);
    } else {
        ledcWrite(PIN_MOTOR_L_IN1, 1023);
        ledcWrite(PIN_MOTOR_L_IN2, 1023);
    }

    // Right Motor Control
    if (dutyR > 0.01f) {
        ledcWrite(PIN_MOTOR_R_IN1, valR);
        ledcWrite(PIN_MOTOR_R_IN2, 0);
    } else if (dutyR < -0.01f) {
        ledcWrite(PIN_MOTOR_R_IN1, 0);
        ledcWrite(PIN_MOTOR_R_IN2, valR);
    } else {
        ledcWrite(PIN_MOTOR_R_IN1, 1023);
        ledcWrite(PIN_MOTOR_R_IN2, 1023);
    }
}

// ==============================================================================
// 4. AUTOMATED 60-SECOND BENCH TEST FSM SEQUENCE
// ==============================================================================
void runAutomatedBenchSequence(uint32_t elapsedMs, float &pwmL, float &pwmR, uint8_t &phase) {
    // Phase 0: 0 to 5s - Static ZUPT (Rest)
    if (elapsedMs < 5000) {
        phase = 0;
        pwmL = 0.0f;
        pwmR = 0.0f;
    }
    // Phase 1: 5 to 15s - Left Motor Stiction Ramp (0% -> 50% PWM)
    else if (elapsedMs < 15000) {
        phase = 1;
        float progress = (float)(elapsedMs - 5000) / 10000.0f;
        pwmL = progress * 0.50f;
        pwmR = 0.0f;
    }
    // Phase 2: 15 to 20s - Left Motor Speed Steps (30%, 60%, 100%)
    else if (elapsedMs < 20000) {
        phase = 2;
        pwmR = 0.0f;
        if (elapsedMs < 16500)      pwmL = 0.30f;
        else if (elapsedMs < 18500) pwmL = 0.60f;
        else                        pwmL = 1.00f;
    }
    // Phase 3: 20 to 25s - Left Motor Reverse Check (-40% PWM)
    else if (elapsedMs < 25000) {
        phase = 3;
        pwmL = -0.40f;
        pwmR = 0.0f;
    }
    // Phase 4: 25 to 35s - Right Motor Stiction Ramp (0% -> 50% PWM)
    else if (elapsedMs < 35000) {
        phase = 4;
        pwmL = 0.0f;
        float progress = (float)(elapsedMs - 25000) / 10000.0f;
        pwmR = progress * 0.50f;
    }
    // Phase 5: 35 to 40s - Right Motor Speed Steps (30%, 60%, 100%)
    else if (elapsedMs < 40000) {
        phase = 5;
        pwmL = 0.0f;
        if (elapsedMs < 36500)      pwmR = 0.30f;
        else if (elapsedMs < 38500) pwmR = 0.60f;
        else                        pwmR = 1.00f;
    }
    // Phase 6: 40 to 45s - Right Motor Reverse Check (-40% PWM)
    else if (elapsedMs < 45000) {
        phase = 6;
        pwmL = 0.0f;
        pwmR = -0.40f;
    }
    // Phase 7: 45 to 55s - Dual Motor Forward Balance (50% PWM)
    else if (elapsedMs < 55000) {
        phase = 7;
        pwmL = 0.50f;
        pwmR = 0.50f;
    }
    // Phase 8: 55s to 60s+ - Full Brake & Test Complete
    else {
        phase = 8;
        pwmL = 0.0f;
        pwmR = 0.0f;
    }
}

// ==============================================================================
// 5. SETUP & 100 HZ REAL-TIME LOOP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

#if ROBOT_HAS_TCA9548A
    // Initialize Single I2C Bus on GPIO 1, 2 for Robots 1, 3, and 4
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(20);
#else
    // Initialize Direct Dual I2C Buses for Robot 2
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);   // GPIO 7, 8
    delay(20);
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_CLOCK_FREQ_HZ); // GPIO 4, 6
    delay(20);
#endif

    // Initialize BMI160
    if (!initBMI160()) {
        Serial.println("[WARN] BMI160 IMU initialization failed!");
    }

    // Initialize Motors
    setupMotors();

    // Output Pure 13-Column CSV Header
    Serial.println("TimeMs,Phase,PwmL,PwmR,RawEncL,RawEncR,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,TempESP");
}

void loop() {
    static uint32_t startTestTimeMs = millis();
    static bool testCompleted = false;

    if (testCompleted) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - startTestTimeMs;

    // 1. Read Encoders & IMU @ 100 Hz
    updateEncoders();

    float gx, gy, gz, ax, ay, az;
    readBMI160Data(gx, gy, gz, ax, ay, az);
    float tempESP = temperatureRead();

    // 2. Execute Automated FSM Sequence
    float pwmL = 0.0f;
    float pwmR = 0.0f;
    uint8_t phase = 0;
    runAutomatedBenchSequence(elapsedMs, pwmL, pwmR, phase);

    // 3. Command Motor Outputs
    setMotorSpeeds(pwmL, pwmR);

    // 4. Stream Pure CSV Data Row (100 Hz)
    Serial.printf("%lu,%u,%.3f,%.3f,%ld,%ld,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f\n",
                  elapsedMs,
                  phase,
                  pwmL,
                  pwmR,
                  (long)encL.cumulativeSteps,
                  (long)encR.cumulativeSteps,
                  gx,
                  gy,
                  gz,
                  ax,
                  ay,
                  az,
                  tempESP);

    // 5. Automatic Shutdown exactly at 60 Seconds
    if (elapsedMs >= 60000) {
        testCompleted = true;
        setMotorSpeeds(0.0f, 0.0f); // Hard electrical brake

        Serial.println("\n======================================================================");
        Serial.printf("[BENCH-TEST COMPLETE] Automated 60s benchmark finished successfully (Robot %d).\n", Config::ID);
        Serial.println("======================================================================");
    }

    delay(10); // 100 Hz Loop Rate
}
