#include "BMI160_Custom.h"

BMI160_Custom* BMI160_Custom::_instance = nullptr;

// Allocate platform-specific static mutex
#if defined(ARDUINO_ARCH_RP2040)
mutex_t BMI160_Custom::_imuMutex;
#else
std::mutex BMI160_Custom::_imuMutex;
#endif

// Implement the Nested RAII Lock methods
BMI160_Custom::IMULock::IMULock() {
#if defined(ARDUINO_ARCH_RP2040)
    mutex_enter_blocking(&BMI160_Custom::_imuMutex);
#else
    BMI160_Custom::_imuMutex.lock();
#endif
}

BMI160_Custom::IMULock::~IMULock() {
#if defined(ARDUINO_ARCH_RP2040)
    mutex_exit(&BMI160_Custom::_imuMutex);
#else
    BMI160_Custom::_imuMutex.unlock();
#endif
}

BMI160_Custom::BMI160_Custom(TwoWire &wireInstance, uint8_t i2cAddress)
    : _wire(wireInstance), _address(i2cAddress), accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0) {
    _instance = this;
}

int8_t BMI160_Custom::i2c_read_cb(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (_instance == nullptr) return -1;

    // RAII Lock: strictly scoped lock that automatically releases at any return point
    IMULock lock;

    // Direct TCA9548A Multiplexer switch to Channel 3 (IMU) [SD3/SC3]
    _instance->_wire.beginTransmission(0x70);
    _instance->_wire.write(1 << 3); // Write bitmask for Channel 3 (0x08)
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

    // RAII Lock
    IMULock lock;

    // Direct TCA9548A Multiplexer switch to Channel 3 (IMU) [SD3/SC3]
    _instance->_wire.beginTransmission(0x70);
    _instance->_wire.write(1 << 3); 
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
    // Initialize native mutex dynamically at boot (only on RP2040 architecture)
    static bool mutexInitialized = false;
    if (!mutexInitialized) {
#if defined(ARDUINO_ARCH_RP2040)
        mutex_init(&_imuMutex);
#endif
        mutexInitialized = true;
    }

    _sensorDev.id = _address;
    _sensorDev.intf = BMI160_I2C_INTF;
    _sensorDev.read = i2c_read_cb;
    _sensorDev.write = i2c_write_cb;
    _sensorDev.delay_ms = delay_ms_cb;

    int8_t rslt = bmi160_init(&_sensorDev);
    return (rslt == BMI160_OK);
}

bool BMI160_Custom::configureDefault() {
    _sensorDev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    _sensorDev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
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

    // RAII Lock
    IMULock lock;

    accX = (float)accelData.x / 8192.0f;
    accY = (float)accelData.y / 8192.0f;
    accZ = (float)accelData.z / 8192.0f;

    gyroX = (float)gyroData.x / 16.4f;
    gyroY = (float)gyroData.y / 16.4f;
    gyroZ = (float)gyroData.z / 16.4f;

    return true;
}

// Thread-safe Getters
float BMI160_Custom::getAccX() {
    IMULock lock;
    return accX;
}
float BMI160_Custom::getAccY() {
    IMULock lock;
    return accY;
}
float BMI160_Custom::getAccZ() {
    IMULock lock;
    return accZ;
}
float BMI160_Custom::getGyroX() {
    IMULock lock;
    return gyroX;
}
float BMI160_Custom::getGyroY() {
    IMULock lock;
    return gyroY;
}
float BMI160_Custom::getGyroZ() {
    IMULock lock;
    return gyroZ;
}
