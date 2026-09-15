#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <mutex>
#include "bmi160.h"

class BMI160_Custom {
public:
    BMI160_Custom(TwoWire &wireInstance = Wire, uint8_t i2cAddress = 0x69, uint8_t tcaChannel = 2);

    bool begin();
    bool configureDefault();
    bool readSensorData();

    // Raw acceleration converted to m/s^2
    float getAccX();
    float getAccY();
    float getAccZ();

    // Angular velocity in deg/s
    float getGyroX();
    float getGyroY();
    float getGyroZ();

    // Internal sensor temperature in Celsius
    float getTemperature();

private:
    TwoWire &_wire;
    uint8_t _address;
    uint8_t _tcaChannel;
    struct bmi160_dev _sensorDev;

    static std::mutex _imuMutex;

    float _accX, _accY, _accZ;
    float _gyroX, _gyroY, _gyroZ;
    float _temperatureC;

    static int8_t i2c_read_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static int8_t i2c_write_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static void delay_ms_cb(uint32_t period);

    static BMI160_Custom* _instance;

    class IMULock {
    public:
        IMULock() { _imuMutex.lock(); }
        ~IMULock() { _imuMutex.unlock(); }
    };
};
