#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

#include "PinMap.h"
#include "RobotConfig.h"
#include "MotorController.h"
#include "MagneticEncoder.h"
#include "BMI160_Custom.h"

// ==============================================================================
// 1. EXACT UWB CONFIGURATION (MATCHING DATA-LOGGING CODE)
// ==============================================================================
device_configuration_t UWB_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

// Actuators & Sensor Instances
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);
MagneticEncoder encL(Wire, 0x70, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, 0x70, 1, Config::INVERT_ENCODER_RIGHT);
BMI160_Custom   imu(Wire, 0x69, 2);

// TCA9548A Channel Selector
bool selectTCA(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

// ==============================================================================
// 2. DIAGNOSTIC SUITE
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1500);

    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);

    Serial.println("\n==================================================================");
    Serial.printf("ANJOMAN COMPREHENSIVE HARDWARE DIAGNOSTIC - ROBOT %u\n", Config::ID);
    Serial.println("==================================================================");

    // --------------------------------------------------------------------------
    // TEST 1: I2C BUS & MULTIPLEXER @ 400 kHz
    // --------------------------------------------------------------------------
    Serial.printf("[TEST 1] Initializing I2C Master @ 400 kHz (SDA=%u, SCL=%u)... ", PIN_I2C0_SDA, PIN_I2C0_SCL);
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);
    Wire.setTimeOut(50);
    Serial.println("OK");

    Serial.print("[TEST 2] Probing TCA9548A Multiplexer at 0x70... ");
    Wire.beginTransmission(0x70);
    if (Wire.endTransmission() == 0) {
        Serial.println("PASS (0x70 Acknowledged)");
    } else {
        Serial.println("FAIL! (TCA9548A not responding)");
    }

    // --------------------------------------------------------------------------
    // TEST 3 & 4: AS5600 ENCODERS
    // --------------------------------------------------------------------------
    Serial.print("[TEST 3] Testing Left Encoder AS5600 (TCA Channel 0)... ");
    if (encL.begin()) {
        encL.update(0.01f);
        uint16_t angleL = encL.getRawAngle();
        Serial.printf("PASS (Raw Angle: %u / 4096)\n", angleL);
    } else {
        Serial.println("FAIL! (No response on TCA Channel 0)");
    }

    Serial.print("[TEST 4] Testing Right Encoder AS5600 (TCA Channel 1)... ");
    if (encR.begin()) {
        encR.update(0.01f);
        uint16_t angleR = encR.getRawAngle();
        Serial.printf("PASS (Raw Angle: %u / 4096)\n", angleR);
    } else {
        Serial.println("FAIL! (No response on TCA Channel 1)");
    }

    // --------------------------------------------------------------------------
    // TEST 5: BMI160 IMU SENSOR
    // --------------------------------------------------------------------------
    Serial.print("[TEST 5] Testing BMI160 IMU (TCA Channel 2)... ");
    if (imu.begin()) {
        delay(50);
        if (imu.readSensorData()) {
            Serial.printf("PASS (AccZ: %.2f m/s^2, GyroZ: %.2f dps)\n", imu.getAccZ(), imu.getGyroZ());
        } else {
            Serial.println("WARNING (Init OK, but sensor read returned false)");
        }
    } else {
        Serial.println("FAIL! (BMI160 failed to initialize)");
    }

    // --------------------------------------------------------------------------
    // TEST 6: INA226 POWER MONITOR (Channel 3)
    // --------------------------------------------------------------------------
    Serial.print("[TEST 6] Testing INA226 Power Monitor (TCA Channel 3)... ");
    if (selectTCA(3)) {
        Wire.beginTransmission(0x40);
        Wire.write(0x02); // Bus voltage register
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x40, (uint8_t)2) >= 2) {
            uint16_t raw = (Wire.read() << 8) | Wire.read();
            float vBat = (float)raw * 0.00125f;
            Serial.printf("PASS (V_Bat = %.2f V)\n", vBat);
        } else {
            Serial.println("FAIL! (INA226 not responding at 0x40)");
        }
    } else {
        Serial.println("FAIL! (Cannot switch to Channel 3)");
    }

    // --------------------------------------------------------------------------
    // TEST 7: DW1000 UWB (EXACT DATA-LOGGING DRIVER STRUCTURE)
    // --------------------------------------------------------------------------
    Serial.print("[TEST 7] Initializing DW1000 UWB (Reset & SPI2)... ");
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(10);
    pinMode(PIN_UWB_RST, INPUT);
    delay(25);

    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    delay(10);

    DW1000Ng::initializeNoInterrupt(PIN_UWB_CS, PIN_UWB_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setDeviceAddress(Config::ID);
    DW1000Ng::setNetworkId(0xDECA);
    DW1000Ng::setAntennaDelay(16436);

    float tUwb = DW1000Ng::getTemperature();
    float vUwb = DW1000Ng::getBatteryVoltage();

    if (vUwb > 1.0f && vUwb < 4.0f) {
        Serial.printf("PASS (Internal VDD = %.2f V, Temp = %.1f C)\n", vUwb, tUwb);
    } else {
        Serial.printf("FAIL! (Invalid read: VDD = %.2f V, Temp = %.1f C)\n", vUwb, tUwb);
    }

    // --------------------------------------------------------------------------
    // TEST 8: MOTOR ACTUATOR DRIVE & ENCODER TICK VERIFICATION
    // --------------------------------------------------------------------------
    Serial.println("\n------------------------------------------------------------------");
    Serial.println("[TEST 8] MOTOR ACTUATION & ENCODER COUPLING TEST (20 kHz PWM)");
    Serial.printf("Left Motor Pins : IN1=%u, IN2=%u\n", PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2);
    Serial.printf("Right Motor Pins: IN1=%u, IN2=%u\n", PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2);
    Serial.println("------------------------------------------------------------------");

    motorL.begin(20000, 10);
    motorR.begin(20000, 10);

    MotorSysIDParams paramsL = { Config::DEADBAND_FWD_L, Config::DEADBAND_REV_L, Config::GAIN_RPM_FWD_L, Config::GAIN_RPM_REV_L, 7.40f };
    MotorSysIDParams paramsR = { Config::DEADBAND_FWD_R, Config::DEADBAND_REV_R, Config::GAIN_RPM_FWD_R, Config::GAIN_RPM_REV_R, 7.40f };
    motorL.setCalibration(paramsL);
    motorR.setCalibration(paramsR);

    // --- TEST LEFT MOTOR ---
    Serial.println(">>> Testing LEFT MOTOR: Spinning at 55% duty for 1.2 seconds...");
    encL.update(0.01f);
    int32_t startTicksL = encL.getCumulativeSteps();

    motorL.setOpenLoopDuty(0.55f);
    for (int i = 0; i < 120; i++) {
        delay(10);
        encL.update(0.01f);
    }
    motorL.brake();
    int32_t endTicksL = encL.getCumulativeSteps();
    int32_t deltaL = endTicksL - startTicksL;

    Serial.printf("Left Motor Result : StartTicks=%ld, EndTicks=%ld, Delta=%ld ticks -> %s\n",
                  (long)startTicksL, (long)endTicksL, (long)deltaL,
                  (abs(deltaL) > 100) ? "PASS (Motor Spun & Encoder Verified)" : "FAIL (No rotation or encoder dead!)");

    delay(800);

    // --- TEST RIGHT MOTOR ---
    Serial.println(">>> Testing RIGHT MOTOR: Spinning at 55% duty for 1.2 seconds...");
    encR.update(0.01f);
    int32_t startTicksR = encR.getCumulativeSteps();

    motorR.setOpenLoopDuty(0.55f);
    for (int i = 0; i < 120; i++) {
        delay(10);
        encR.update(0.01f);
    }
    motorR.brake();
    int32_t endTicksR = encR.getCumulativeSteps();
    int32_t deltaR = endTicksR - startTicksR;

    Serial.printf("Right Motor Result: StartTicks=%ld, EndTicks=%ld, Delta=%ld ticks -> %s\n",
                  (long)startTicksR, (long)endTicksR, (long)deltaR,
                  (abs(deltaR) > 100) ? "PASS (Motor Spun & Encoder Verified)" : "FAIL (No rotation or encoder dead!)");

    Serial.println("\n==================================================================");
    Serial.println("ALL HARDWARE TESTS COMPLETE. MOTORS LOCKED IN BRAKE.");
    Serial.println("==================================================================");
}

void loop() {
    delay(1000);
}
