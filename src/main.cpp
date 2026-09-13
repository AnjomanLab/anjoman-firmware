#include <Arduino.h>
#include <Wire.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & ADDRESSES
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  AS5600_ADDR         = 0x36;
constexpr uint8_t  BMI160_DEFAULT_ADDR = 0x69;
constexpr uint8_t  INA226_ADDR         = 0x40;

constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t PWM_FREQ_HZ        = 20000;
constexpr uint8_t  PWM_RES_BITS       = 10;

// Shunt resistor on MakerBotics INA226 board is R100 (0.1 Ohm)
constexpr float SHUNT_RESISTOR_OHM     = 0.100f;

// Voltage Divider on GPIO 4: (100k + 33k) / 33k = 4.0303
constexpr float VBAT_DIVIDER_RATIO     = 133.0f / 33.0f;

uint8_t detectedBMI160Addr = BMI160_DEFAULT_ADDR;
bool ina226Ready = false;
bool bmi160Ready = false;

// ==============================================================================
// 2. TCA9548A MULTIPLEXER ROUTINES
// ==============================================================================
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

// ==============================================================================
// 3. INA226 DIRECT I2C DRIVER
// ==============================================================================
bool initINA226() {
    if (!selectTCAChannel(3)) return false;

    // Verify presence at 0x40
    Wire.beginTransmission(INA226_ADDR);
    if (Wire.endTransmission() != 0) return false;

    // Reset INA226 (Reg 0x00, write 0x8000)
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00);
    Wire.write(0x80);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Configure: Continuous Shunt and Bus, 1.1ms conversion, 4 averages (Reg 0x00 = 0x4227)
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00);
    Wire.write(0x42);
    Wire.write(0x27);
    return (Wire.endTransmission() == 0);
}

bool readINA226(float &vbus_V, float &vshunt_mV, float &current_mA, float &power_mW) {
    if (!selectTCAChannel(3)) return false;

    // Read Bus Voltage (Register 0x02) - LSB = 1.25 mV
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint16_t rawBus = (Wire.read() << 8) | Wire.read();
    vbus_V = (float)rawBus * 0.00125f;

    // Read Shunt Voltage (Register 0x01) - LSB = 2.5 uV = 0.0025 mV
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    int16_t rawShunt = (int16_t)((Wire.read() << 8) | Wire.read());
    vshunt_mV = (float)rawShunt * 0.0025f;

    // Calculate Current and Power using Shunt Resistor
    current_mA = (vshunt_mV / SHUNT_RESISTOR_OHM);
    power_mW   = vbus_V * current_mA;

    return true;
}

// ==============================================================================
// 4. BMI160 & AS5600 ROUTINES
// ==============================================================================
bool initBMI160() {
    if (!selectTCAChannel(2)) return false;

    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) detectedBMI160Addr = 0x69;
    else {
        Wire.beginTransmission(0x68);
        if (Wire.endTransmission() == 0) detectedBMI160Addr = 0x68;
        else return false;
    }

    // Soft reset
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0xB6);
    Wire.endTransmission();
    delay(50);

    // Normal mode Accel & Gyro
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x11);
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x15);
    Wire.endTransmission();
    delay(50);

    return true;
}

void readBMI160(float &gz, float &ax, float &ay, float &az) {
    gz = ax = ay = az = 0.0f;
    if (!selectTCAChannel(2)) return;

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x0C);
    if (Wire.endTransmission() != 0) return;

    Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)12);
    if (Wire.available() >= 12) {
        Wire.read(); Wire.read(); // Skip Gx
        Wire.read(); Wire.read(); // Skip Gy
        int16_t raw_gz = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_ax = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_ay = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t raw_az = (int16_t)(Wire.read() | (Wire.read() << 8));

        gz = (float)raw_gz / 16.4f;
        ax = (float)raw_ax * 9.80665f / 16384.0f;
        ay = (float)raw_ay * 9.80665f / 16384.0f;
        az = (float)raw_az * 9.80665f / 16384.0f;
    }
}

uint16_t readAS5600Angle(uint8_t channel) {
    if (!selectTCAChannel(channel)) return 0xFFFF;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    if (Wire.endTransmission() != 0) return 0xFFFF;

    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        return ((uint16_t)(msb & 0x0F) << 8) | lsb;
    }
    return 0xFFFF;
}

// ==============================================================================
// 5. MOTOR CONTROLLER (DRV8833 PWM)
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

void setMotorSpeeds(float dutyL, float dutyR) {
    if (Config::INVERT_MOTOR_LEFT)  dutyL = -dutyL;
    if (Config::INVERT_MOTOR_RIGHT) dutyR = -dutyR;

    dutyL = constrain(dutyL, -1.0f, 1.0f);
    dutyR = constrain(dutyR, -1.0f, 1.0f);

    uint32_t valL = (uint32_t)(fabs(dutyL) * 1023.0f);
    uint32_t valR = (uint32_t)(fabs(dutyR) * 1023.0f);

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
// 6. SETUP & DIAGNOSTIC LOOP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1500);

    Serial.println("\n======================================================================");
    Serial.printf("   ANJOMAN FIRMWARE - SENSOR & POWER DIAGNOSTIC TEST (ROBOT %d)\n", Config::ID);
    Serial.println("======================================================================");

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

    // Initialize ADC for Voltage Divider
    analogReadResolution(12);
    pinMode(PIN_VBAT_SENSE, INPUT);

    // Initialize I2C Master Bus
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(50);

    // 1. Verify Multiplexer
    Wire.beginTransmission(TCA9548A_ADDR);
    if (Wire.endTransmission() == 0) {
        Serial.println("[OK] TCA9548A Multiplexer detected at 0x70.");
    } else {
        Serial.println("[FAIL] TCA9548A Multiplexer NOT responding at 0x70!");
    }

    // 2. Initialize INA226 on Channel 3
    ina226Ready = initINA226();
    Serial.printf("[%s] INA226 Current/Power Sensor on TCA Channel 3 (0x40)\n", 
                  ina226Ready ? "OK" : "FAIL");

    // 3. Initialize BMI160 on Channel 2
    bmi160Ready = initBMI160();
    Serial.printf("[%s] BMI160 IMU on TCA Channel 2 (0x%02X)\n", 
                  bmi160Ready ? "OK" : "FAIL", detectedBMI160Addr);

    // 4. Initialize Motors
    setupMotors();
    Serial.println("[OK] Motor Driver initialized.");

    Serial.println("\n--- STARTING LIVE DIAGNOSTIC STREAM ---");
    Serial.println("Vbat_ADC(V) | Vbus_INA(V) | Current(mA) | Power(mW) | EncL | EncR | GyroZ(dps) | MotorState");
    Serial.println("-----------------------------------------------------------------------------------------");
}

void loop() {
    static uint32_t lastPrintMs = 0;
    static uint32_t stateStartMs = millis();
    static uint8_t motorState = 0; // 0: Stop, 1: Fwd, 2: Stop, 3: Rev

    uint32_t nowMs = millis();

    // -------------------------------------------------------------
    // Motor Cycle: Stop (2s) -> Forward (3s) -> Stop (2s) -> Reverse (3s)
    // -------------------------------------------------------------
    float dutyCmd = 0.0f;
    const char* stateStr = "STOP";

    if (motorState == 0) {
        dutyCmd = 0.0f;
        stateStr = "STOP";
        if (nowMs - stateStartMs >= 2000) { motorState = 1; stateStartMs = nowMs; }
    } else if (motorState == 1) {
        dutyCmd = 0.50f; // 50% PWM Forward
        stateStr = "FWD_50%";
        if (nowMs - stateStartMs >= 3000) { motorState = 2; stateStartMs = nowMs; }
    } else if (motorState == 2) {
        dutyCmd = 0.0f;
        stateStr = "STOP";
        if (nowMs - stateStartMs >= 2000) { motorState = 3; stateStartMs = nowMs; }
    } else if (motorState == 3) {
        dutyCmd = -0.50f; // 50% PWM Reverse
        stateStr = "REV_50%";
        if (nowMs - stateStartMs >= 3000) { motorState = 0; stateStartMs = nowMs; }
    }

    setMotorSpeeds(dutyCmd, dutyCmd);

    // -------------------------------------------------------------
    // Sensor Readouts & Telemetry Stream (every 100 ms)
    // -------------------------------------------------------------
    if (nowMs - lastPrintMs >= 100) {
        lastPrintMs = nowMs;

        // 1. Read Resistor Divider ADC (GPIO 4)
        uint32_t rawMv = analogReadMilliVolts(PIN_VBAT_SENSE);
        float vbat_adc = ((float)rawMv * VBAT_DIVIDER_RATIO) / 1000.0f;

        // 2. Read INA226 on TCA Ch3
        float vbus_V = 0.0f, vshunt_mV = 0.0f, current_mA = 0.0f, power_mW = 0.0f;
        readINA226(vbus_V, vshunt_mV, current_mA, power_mW);

        // 3. Read Encoders
        uint16_t encL = readAS5600Angle(0);
        uint16_t encR = readAS5600Angle(1);

        // 4. Read BMI160
        float gz, ax, ay, az;
        readBMI160(gz, ax, ay, az);

        // Stream Formatted Diagnostic Line
        Serial.printf("%11.2f | %11.2f | %11.1f | %9.1f | %4u | %4u | %10.2f | %s\n",
                      vbat_adc,
                      vbus_V,
                      current_mA,
                      power_mW,
                      encL,
                      encR,
                      gz,
                      stateStr);
    }
    yield();
}
