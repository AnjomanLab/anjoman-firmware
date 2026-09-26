#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "bmi160.h"
#include "AnjomanI2C.h"

// ==============================================================================
// BMI160_Custom — Thread-safe BMI160 IMU driver
// ==============================================================================
// I2C thread safety is provided by AnjomanI2C::Guard. This class does NOT
// own its own mutex. All transactions are wrapped in AnjomanI2C::Guard
// so they are mutually exclusive with encoder and INA226 transactions.
// ==============================================================================

class BMI160_Custom {
public:
    BMI160_Custom(TwoWire &wireInstance = Wire,
                  uint8_t i2cAddress = 0x69,
                  uint8_t tcaChannel = 2);

    bool begin();
    bool configureDefault();
    bool readSensorData();

    // Physical quantities
    float getAccX() const { return _accX; }   // m/s^2
    float getAccY() const { return _accY; }
    float getAccZ() const { return _accZ; }

    float getGyroX() const { return _gyroX; } // deg/s
    float getGyroY() const { return _gyroY; }
    float getGyroZ() const { return _gyroZ; }

    float getTemperature() const { return _temperatureC; }

    bool isConnected() const { return _connected; }

private:
    TwoWire &_wire;
    uint8_t  _address;
    uint8_t  _tcaChannel;
    bool     _connected;

    struct bmi160_dev _sensorDev;

    float _accX, _accY, _accZ;
    float _gyroX, _gyroY, _gyroZ;
    float _temperatureC;

    static BMI160_Custom* _instance;

    static int8_t i2c_read_cb(uint8_t dev_id, uint8_t reg_addr,
                              uint8_t *data, uint16_t len);
    static int8_t i2c_write_cb(uint8_t dev_id, uint8_t reg_addr,
                               uint8_t *data, uint16_t len);
    static void   delay_ms_cb(uint32_t period);
};
