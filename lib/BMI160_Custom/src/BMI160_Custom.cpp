#include "BMI160_Custom.h"

BMI160_Custom* BMI160_Custom::_instance = nullptr;
std::mutex BMI160_Custom::_imuMutex;

BMI160_Custom::BMI160_Custom(TwoWire &wireInstance, uint8_t i2cAddress, uint8_t tcaChannel)
    : _wire(wireInstance),
      _address(i2cAddress),
      _tcaChannel(tcaChannel),
      _accX(0.0f), _accY(0.0f), _accZ(0.0f),
      _gyroX(0.0f), _gyroY(0.0f), _gyroZ(0.0f),
      _temperatureC(0.0f) {
    _instance = this;
}

int8_t BMI160_Custom::i2c_read_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (_instance == nullptr) return -1;

    IMULock lock;

    // Switch TCA9548A multiplexer to designated IMU channel (Default: Channel 2)
    _instance->_wire.beginTransmission(0x70);
    _instance->_wire.write(1 << _instance->_tcaChannel);
    if (_instance->_wire.endTransmission() != 0) {
        return -1;
    }

    _instance->_wire.beginTransmission(dev_id);
    _instance->_wire.write(reg_addr);
    if (_instance->_wire.endTransmission() != 0) {
        return -1;
    }

    _instance->_wire.requestFrom(dev_id, (uint8_t)len);
    for (uint16_t i = 0; i < len; i++) {
        if (_instance->_wire.available()) {
            data[i] = _instance->_wire.read();
        } else {
            return -1;
        }
    }
    return 0;
}

int8_t BMI160_Custom::i2c_write_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (_instance == nullptr) return -1;

    IMULock lock;

    // Switch TCA9548A multiplexer to designated IMU channel (Default: Channel 2)
    _instance->_wire.beginTransmission(0x70);
    _instance->_wire.write(1 << _instance->_tcaChannel);
    if (_instance->_wire.endTransmission() != 0) {
        return -1;
    }

    _instance->_wire.beginTransmission(dev_id);
    _instance->_wire.write(reg_addr);
    for (uint16_t i = 0; i < len; i++) {
        _instance->_wire.write(data[i]);
    }

    if (_instance->_wire.endTransmission() != 0) {
        return -1;
    }
    return 0;
}

void BMI160_Custom::delay_ms_cb(uint32_t period) {
    delay(period);
}

bool BMI160_Custom::begin() {
    _sensorDev.id = _address;
    _sensorDev.intf = BMI160_I2C_INTF;
    _sensorDev.read = i2c_read_cb;
    _sensorDev.write = i2c_write_cb;
    _sensorDev.delay_ms = delay_ms_cb;

    int8_t rslt = bmi160_init(&_sensorDev);
    if (rslt != BMI160_OK) {
        // Fallback: Check alternative address 0x68 if 0x69 failed
        uint8_t altAddr = (_address == 0x69) ? 0x68 : 0x69;
        _sensorDev.id = altAddr;
        rslt = bmi160_init(&_sensorDev);
        if (rslt == BMI160_OK) {
            _address = altAddr;
        } else {
            return false;
        }
    }

    return configureDefault();
}

bool BMI160_Custom::configureDefault() {
    _sensorDev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    _sensorDev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    _sensorDev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    _sensorDev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    _sensorDev.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    _sensorDev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    _sensorDev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    _sensorDev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    int8_t rslt = bmi160_set_sens_conf(&_sensorDev);
    return (rslt == BMI160_OK);
}

bool BMI160_Custom::readSensorData() {
    struct bmi160_sensor_data accelData;
    struct bmi160_sensor_data gyroData;

    int8_t rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accelData, &gyroData, &_sensorDev);
    if (rslt != BMI160_OK) {
        return false;
    }

    IMULock lock;

    // Convert 2g range to m/s^2 (16384 LSB/g)
    _accX = ((float)accelData.x * 9.80665f) / 16384.0f;
    _accY = ((float)accelData.y * 9.80665f) / 16384.0f;
    _accZ = ((float)accelData.z * 9.80665f) / 16384.0f;

    // Convert 2000 dps range (16.4 LSB/(deg/s))
    _gyroX = (float)gyroData.x / 16.4f;
    _gyroY = (float)gyroData.y / 16.4f;
    _gyroZ = (float)gyroData.z / 16.4f;

    return true;
}

float BMI160_Custom::getAccX() {
    IMULock lock;
    return _accX;
}

float BMI160_Custom::getAccY() {
    IMULock lock;
    return _accY;
}

float BMI160_Custom::getAccZ() {
    IMULock lock;
    return _accZ;
}

float BMI160_Custom::getGyroX() {
    IMULock lock;
    return _gyroX;
}

float BMI160_Custom::getGyroY() {
    IMULock lock;
    return _gyroY;
}

float BMI160_Custom::getGyroZ() {
    IMULock lock;
    return _gyroZ;
}

float BMI160_Custom::getTemperature() {
    IMULock lock;
    return _temperatureC;
}
