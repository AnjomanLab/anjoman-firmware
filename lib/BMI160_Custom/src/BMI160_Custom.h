#ifndef BMI160_CUSTOM_H
#define BMI160_CUSTOM_H

#include <Arduino.h>
#include <Wire.h>
#include "bmi160.h"

// Conditionally include mutex headers depending on the target architecture
#if defined(ARDUINO_ARCH_RP2040)
#include "pico/mutex.h"
#else
#include <mutex>
#endif

class BMI160_Custom {
public:
    // Constructor. Default address is 0x68 (BMI160_I2C_ADDR_TYPE1)
    BMI160_Custom(TwoWire &wireInstance = Wire, uint8_t i2cAddress = 0x68);

    // Initializes the sensor and sets up the Bosch API structures
    bool begin();

    // Configures accelerometer and gyroscope with standard robotics settings
    bool configureDefault();

    // Reads raw sensor data and updates internal values (Thread-Safe)
    bool readSensorData();

    // Thread-safe Getters for float data
    float getAccX();
    float getAccY();
    float getAccZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();

private:
    TwoWire &_wire;
    uint8_t _address;
    struct bmi160_dev _sensorDev;

    // Platform-specific static Mutex declaration
#if defined(ARDUINO_ARCH_RP2040)
    static mutex_t _imuMutex;
#else
    static std::mutex _imuMutex;
#endif

    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;

    // Static callbacks required by the Bosch C-API
    static int8_t i2c_read_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static int8_t i2c_write_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static void delay_ms_cb(uint32_t period);

    // Pointer to hold active instance for callbacks
    static BMI160_Custom* _instance;

    // Nested RAII Helper Class to handle scoped locking seamlessly
    class IMULock {
    public:
        IMULock();
        ~IMULock();
    };
};

#endif // BMI160_CUSTOM_H
