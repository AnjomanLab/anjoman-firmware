#include "MagneticEncoder.h"

// Define the static shared mutex
std::mutex MagneticEncoder::_i2cBusMutex;

MagneticEncoder::MagneticEncoder(TwoWire &wireInstance, uint8_t muxAddress, uint8_t muxChannel)
    : _wire(wireInstance), 
      _muxAddress(muxAddress), 
      _muxChannel(muxChannel), 
      _cumulativeSteps(0), 
      _lastRawAngle(0), 
      _isFirstRead(true) {}

bool MagneticEncoder::begin() {
    // Basic verification of the communication bus availability
    std::lock_guard<std::mutex> lock(_i2cBusMutex);
    _wire.beginTransmission(_muxAddress);
    return (_wire.endTransmission() == 0);
}

bool MagneticEncoder::selectMuxChannel() {
    if (_muxChannel > 7) return false;
    
    _wire.beginTransmission(_muxAddress);
    _wire.write(1 << _muxChannel);
    return (_wire.endTransmission() == 0);
}

uint16_t MagneticEncoder::readRawAngle() {
    // Strictly scoped RAII lock protecting the shared I2C1 bus
    std::lock_guard<std::mutex> lock(_i2cBusMutex);

    if (!selectMuxChannel()) {
        return 0xFFFF; // Return error state indicator
    }

    _wire.beginTransmission(AS5600_ADDR);
    _wire.write(ANGLE_REG_HIGH);
    if (_wire.endTransmission() != 0) {
        return 0xFFFF; 
    }

    _wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    if (_wire.available() >= 2) {
        uint8_t highByte = _wire.read();
        uint8_t lowByte = _wire.read();
        return ((uint16_t)highByte << 8) | lowByte;
    }

    return 0xFFFF; 
}

float MagneticEncoder::getAngleRadians() {
    uint16_t rawAngle = readRawAngle();
    if (rawAngle == 0xFFFF) {
        // Return accumulated physical representation even on failed reads to prevent control-loop crash
        return ((float)_cumulativeSteps / 4096.0f) * 2.0f * PI;
    }

    int16_t currentRaw = (int16_t)rawAngle;

    if (_isFirstRead) {
        _lastRawAngle = currentRaw;
        _isFirstRead = false;
    }

    // Roll-over/under delta detection (AS5600 12-bit max resolution is 4096)
    int16_t delta = currentRaw - _lastRawAngle;
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }

    _cumulativeSteps += delta;
    _lastRawAngle = currentRaw;

    // Return exact physical continuous state in radians
    return ((float)_cumulativeSteps / 4096.0f) * 2.0f * PI;
}

void MagneticEncoder::resetCumulativeAngle() {
    std::lock_guard<std::mutex> lock(_i2cBusMutex);
    _cumulativeSteps = 0;
    _isFirstRead = true;
}
