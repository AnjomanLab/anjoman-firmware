#include <Arduino.h>
#include <Wire.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & SENSOR REGISTERS
// ==============================================================================
constexpr uint8_t TCA9548A_ADDR       = 0x70;
constexpr uint8_t BMI160_DEFAULT_ADDR = 0x69; // Or 0x68 based on SDO pin
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t SAMPLE_PERIOD_US   = 10000;   // 200 Hz Sampling Rate (5 ms period)

uint8_t detectedBMI160Addr = BMI160_DEFAULT_ADDR;
bool imuReady = false;

// ==============================================================================
// 2. BMI160 HARDWARE DRIVER WITH INTERNAL TEMPERATURE EXTRACTION
// ==============================================================================
#if ROBOT_HAS_TCA9548A
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}
#endif

bool initBMI160() {
#if ROBOT_HAS_TCA9548A
    if (!selectTCAChannel(2)) return false;
#endif

    // Detect Address (0x69 or 0x68)
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) {
        detectedBMI160Addr = 0x69;
    } else {
        Wire.beginTransmission(0x68);
        if (Wire.endTransmission() == 0) {
            detectedBMI160Addr = 0x68;
        } else {
            return false;
        }
    }

    // 1. Soft Reset BMI160
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); // CMD register
    Wire.write(0xB6); // Soft reset
    Wire.endTransmission();
    delay(50);

    // 2. Power Up Accel & Gyro into Normal Mode
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E);
    Wire.write(0x11); // Accel normal mode
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E);
    Wire.write(0x15); // Gyro normal mode
    Wire.endTransmission();
    delay(50);

    // 3. Configure Gyro Range: +/- 2000 deg/s (16.4 LSB / deg/s)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x43); // GYRO_RANGE
    Wire.write(0x00);
    Wire.endTransmission();

    // 4. Configure Accel Range: +/- 2g (16384 LSB / g)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x41); // ACC_RANGE
    Wire.write(0x03);
    Wire.endTransmission();

    return true;
}

// Burst Read: 12 bytes of Gyro/Accel (0x0C..0x17) + 2 bytes of Temperature (0x20..0x21)
bool readBMI160Full(float &gx, float &gy, float &gz, float &ax, float &ay, float &az, float &imuTemp) {
    gx = gy = gz = ax = ay = az = imuTemp = 0.0f;

#if ROBOT_HAS_TCA9548A
    if (!selectTCAChannel(2)) return false;
#endif

    // 1. Read 12-Byte Inertial Data Burst (Gyro X..Z, Accel X..Z)
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

    // Convert to deg/s and m/s^2
    constexpr float GYRO_SCALE_DPS    = 1.0f / 16.4f;
    constexpr float ACCEL_SCALE_MPS2  = 9.80665f / 16384.0f;

    gx = (float)raw_gx * GYRO_SCALE_DPS;
    gy = (float)raw_gy * GYRO_SCALE_DPS;
    gz = (float)raw_gz * GYRO_SCALE_DPS;

    ax = (float)raw_ax * ACCEL_SCALE_MPS2;
    ay = (float)raw_ay * ACCEL_SCALE_MPS2;
    az = (float)raw_az * ACCEL_SCALE_MPS2;

    // 2. Read BMI160 Silicon Temperature (Registers 0x20: TEMPERATURE_0 and 0x21: TEMPERATURE_1)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x20);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t temp_lsb = Wire.read();
        uint8_t temp_msb = Wire.read();
        int16_t raw_temp = (int16_t)(temp_lsb | (temp_msb << 8));

        // Official Bosch formula: Temp(°C) = 23.0 + (raw_temp / 512.0)
        imuTemp = 23.0f + ((float)raw_temp / 512.0f);
    }

    return true;
}

// ==============================================================================
// 3. SETUP & HIGH-RATE REAL-TIME SAMPLING
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

#if ROBOT_HAS_TCA9548A
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
#else
    // Robot 2 Direct Pins (GPIO 7, 8)
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
#endif
    delay(20);

    imuReady = initBMI160();

    // Pure 10-Column Metrology Header
    Serial.println("TimeUs,GyroX_dps,GyroY_dps,GyroZ_dps,AccelX_mps2,AccelY_mps2,AccelZ_mps2,ImuTempC,EspTempC,Status");
}

void loop() {
    static uint64_t nextSampleUs = micros();
    uint64_t nowUs = micros();

    // Strict 200 Hz Sampling Loop (every 5000 microseconds)
    if (nowUs >= nextSampleUs) {
        nextSampleUs = nowUs + SAMPLE_PERIOD_US;

        float gx, gy, gz, ax, ay, az, imuTemp;
        bool ok = readBMI160Full(gx, gy, gz, ax, ay, az, imuTemp);
        float espTemp = temperatureRead();

        uint8_t status = (ok && imuReady) ? 1 : 0;

        // Stream High-Precision CSV Row
        Serial.printf("%llu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.1f,%u\n",
                      (unsigned long long)nowUs,
                      gx,
                      gy,
                      gz,
                      ax,
                      ay,
                      az,
                      imuTemp,
                      espTemp,
                      status);
    }
    yield();
}
