#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & CONFIGURATION
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  BMI160_DEFAULT_ADDR = 0x69;
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t SAMPLE_PERIOD_US   = 10000;  // Strict 100 Hz (10 ms period)
constexpr uint32_t TEST_DURATION_MS   = 180000; // 180 Seconds (3 Minutes = 18,000 samples)

// Dedicated SPI3 instance for MicroSD
SPIClass SPI_SD(HSPI);
File logFile;
bool sdCardReady = false;

uint8_t detectedBMI160Addr = BMI160_DEFAULT_ADDR;
bool imuReady = false;

#pragma pack(push, 1)
struct IMURecord {
    uint64_t timeUs;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float imuTemp;
    float espTemp;
    uint8_t status;
};
#pragma pack(pop)

QueueHandle_t sdQueue = nullptr;
TaskHandle_t  core0TaskHandle = nullptr;

// ==============================================================================
// 2. TCA9548A & BMI160 DRIVER WITH INTERNAL TEMPERATURE
// ==============================================================================
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
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

    // Soft reset BMI160
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0xB6);
    Wire.endTransmission();
    delay(50);

    // Power up Accel and Gyro into Normal Mode
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x11); // Accel normal mode
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x7E); Wire.write(0x15); // Gyro normal mode
    Wire.endTransmission();
    delay(50);

    // Range: +/- 2000 deg/s (16.4 LSB / deg/s)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x43); Wire.write(0x00);
    Wire.endTransmission();

    // Range: +/- 2g (16384 LSB / g)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x41); Wire.write(0x03);
    Wire.endTransmission();

    return true;
}

bool readBMI160Full(float &gx, float &gy, float &gz, float &ax, float &ay, float &az, float &imuTemp) {
    gx = gy = gz = ax = ay = az = imuTemp = 0.0f;
    if (!selectTCAChannel(2)) return false;

    // 1. Read 12-Byte Inertial Data Burst (0x0C..0x17)
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

    constexpr float GYRO_SCALE_DPS   = 1.0f / 16.4f;
    constexpr float ACCEL_SCALE_MPS2 = 9.80665f / 16384.0f;

    gx = (float)raw_gx * GYRO_SCALE_DPS;
    gy = (float)raw_gy * GYRO_SCALE_DPS;
    gz = (float)raw_gz * GYRO_SCALE_DPS;
    ax = (float)raw_ax * ACCEL_SCALE_MPS2;
    ay = (float)raw_ay * ACCEL_SCALE_MPS2;
    az = (float)raw_az * ACCEL_SCALE_MPS2;

    // 2. Read BMI160 Silicon Temperature (Registers 0x20 & 0x21)
    Wire.beginTransmission(detectedBMI160Addr);
    Wire.write(0x20);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)detectedBMI160Addr, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t temp_lsb = Wire.read();
        uint8_t temp_msb = Wire.read();
        int16_t raw_temp = (int16_t)(temp_lsb | (temp_msb << 8));
        imuTemp = 23.0f + ((float)raw_temp / 512.0f);
    }

    return true;
}

// ==============================================================================
// 3. CORE 0 ASYNCHRONOUS MICROSD LOGGER TASK
// ==============================================================================
void core0LoggerTask(void *pvParameters) {
    IMURecord rec;
    uint32_t recordsLogged = 0;

    while (true) {
        if (xQueueReceive(sdQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdCardReady && logFile) {
                logFile.printf("%llu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.1f,%u\n",
                               (unsigned long long)rec.timeUs,
                               rec.gx, rec.gy, rec.gz,
                               rec.ax, rec.ay, rec.az,
                               rec.imuTemp, rec.espTemp, rec.status);
                recordsLogged++;
                if (recordsLogged % 100 == 0) {
                    logFile.flush();
                }
            }

            // Stream over Serial
            Serial.printf("%llu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.1f,%u\n",
                          (unsigned long long)rec.timeUs,
                          rec.gx, rec.gy, rec.gz,
                          rec.ax, rec.ay, rec.az,
                          rec.imuTemp, rec.espTemp, rec.status);
        }
    }
}

// ==============================================================================
// 4. SETUP & MAIN EXECUTION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

    // 1. Ensure Motors are Hard-Braked and Unpowered
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);

    // 2. Initialize Dedicated I2C Bus for this Robot
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(50);

    imuReady = initBMI160();

    // 3. Initialize Dedicated SPI3 MicroSD Card
    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 10000000)) {
        sdCardReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/imu_r%d_static.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("TimeUs,GyroX_dps,GyroY_dps,GyroZ_dps,AccelX_mps2,AccelY_mps2,AccelZ_mps2,ImuTempC,EspTempC,Status");
            logFile.flush();
        }
    }

    // 4. Create Queue and start Core 0 Logger
    sdQueue = xQueueCreate(256, sizeof(IMURecord));
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
    Serial.printf("   ANJOMAN FIRMWARE - 100 Hz STATIC IMU BENCHMARK (ROBOT %d)\n", Config::ID);
    Serial.printf("   Target Duration: %lu Seconds | Storage: MicroSD & 460800 Baud Serial\n", TEST_DURATION_MS / 1000);
    Serial.printf("   MicroSD Status: %s | BMI160 Status: %s\n", 
                  sdCardReady ? "MOUNTED (/imu_rX_static.csv)" : "NOT FOUND (Serial Only)",
                  imuReady ? "HEALTHY" : "FAILED");
    Serial.println("======================================================================");

    // Pure 10-Column Header
    Serial.println("TimeUs,GyroX_dps,GyroY_dps,GyroZ_dps,AccelX_mps2,AccelY_mps2,AccelZ_mps2,ImuTempC,EspTempC,Status");
}

void loop() {
    static uint32_t startTestTimeMs = millis();
    static uint64_t nextSampleUs = micros();
    static bool testCompleted = false;

    if (testCompleted) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint64_t nowUs = micros();
    uint32_t elapsedMs = millis() - startTestTimeMs;

    // Strict 100 Hz Sampling Loop (every 10,000 microseconds)
    if (nowUs >= nextSampleUs) {
        nextSampleUs = nowUs + SAMPLE_PERIOD_US;

        float gx, gy, gz, ax, ay, az, imuTemp;
        bool ok = readBMI160Full(gx, gy, gz, ax, ay, az, imuTemp);
        float espTemp = temperatureRead();

        IMURecord rec = {};
        rec.timeUs = nowUs;
        rec.gx = gx;
        rec.gy = gy;
        rec.gz = gz;
        rec.ax = ax;
        rec.ay = ay;
        rec.az = az;
        rec.imuTemp = imuTemp;
        rec.espTemp = espTemp;
        rec.status = (ok && imuReady) ? 1 : 0;

        if (sdQueue != nullptr) {
            xQueueSend(sdQueue, &rec, 0); // Non-blocking send
        }
    }

    // Automatic Shutdown exactly at 180 Seconds (3 Minutes)
    if (elapsedMs >= TEST_DURATION_MS) {
        testCompleted = true;

        // Allow Core 0 to flush remaining queue records
        vTaskDelay(pdMS_TO_TICKS(200));

        if (logFile) {
            logFile.flush();
            logFile.close();
        }

        Serial.println("\n======================================================================");
        Serial.printf("[TEST COMPLETE] 180s Static Benchmark Finished for Robot %d.\n", Config::ID);
        Serial.println("[STORAGE] MicroSD file safely flushed and closed. Ready for Python analysis.");
        Serial.println("======================================================================");
    }

    yield();
}
