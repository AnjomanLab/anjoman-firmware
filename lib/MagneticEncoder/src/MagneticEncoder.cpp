#include "MagneticEncoder.h"

MagneticEncoder::MagneticEncoder(TwoWire &wireInstance, uint8_t muxAddress,
                                 uint8_t muxChannel, bool invert)
    : _wire(wireInstance),
      _muxAddress(muxAddress),
      _muxChannel(muxChannel),
      _invert(invert),
      _currentRawAngle(0),
      _lastRawAngle(0),
      _lastDelta(0),
      _cumulativeSteps(0),
      _isFirstRead(true),
      _currentRPM(0.0f),
      _currentRadPerSec(0.0f) {}

bool MagneticEncoder::begin() {
    AnjomanI2C::Guard g;
    if (!g.isValid()) return false;
    return selectMuxChannel();
}

// NOTE: caller must hold the I2C guard.
bool MagneticEncoder::selectMuxChannel() {
    if (_muxChannel > 7) return false;
    _wire.beginTransmission(_muxAddress);
    _wire.write(1 << _muxChannel);
    return (_wire.endTransmission() == 0);
}

bool MagneticEncoder::update(float dt) {
    // ---- Take I2C mutex for the full transaction sequence ----
    AnjomanI2C::Guard g;
    if (!g.isValid()) {
        return false;
    }

    if (!selectMuxChannel()) {
        return false;
    }

    _wire.beginTransmission(AS5600_ADDR);
    _wire.write(ANGLE_REG_HIGH);
    if (_wire.endTransmission() != 0) {
        return false;
    }

    _wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    if (_wire.available() < 2) {
        return false;
    }

    uint8_t highByte = _wire.read();
    uint8_t lowByte  = _wire.read();
    uint16_t raw = ((uint16_t)(highByte & 0x0F) << 8) | lowByte;
    _currentRawAngle = raw;

    int16_t currentRaw = (int16_t)raw;

    // ---- Angle unwrapping (no I2C) ----
    if (_isFirstRead) {
        _lastRawAngle = currentRaw;
        _isFirstRead = false;
        _lastDelta = 0;
        return true;
    }

    int16_t delta = currentRaw - _lastRawAngle;
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }

    if (_invert) {
        delta = -delta;
    }

    _lastDelta = delta;
    _cumulativeSteps += delta;
    _lastRawAngle = currentRaw;

    // ---- Velocity computation (no I2C) ----
    if (dt > 0.0001f) {
        float instantRPM = ((float)delta / ENCODER_CPR) * (60.0f / dt);
        _currentRPM = 0.30f * instantRPM + 0.70f * _currentRPM;
        _currentRadPerSec = _currentRPM * (TWO_PI_F / 60.0f);
    }

    return true;
}

uint16_t MagneticEncoder::getRawAngle() const {
    return _currentRawAngle;
}

int16_t MagneticEncoder::getDeltaSteps() const {
    return _lastDelta;
}

int32_t MagneticEncoder::getCumulativeSteps() const {
    return _cumulativeSteps;
}

float MagneticEncoder::getAngleRadians() const {
    return ((float)_cumulativeSteps / ENCODER_CPR) * TWO_PI_F;
}

float MagneticEncoder::getRPM() const {
    return _currentRPM;
}

float MagneticEncoder::getRadPerSec() const {
    return _currentRadPerSec;
}

void MagneticEncoder::resetCumulativeAngle() {
    _cumulativeSteps = 0;
    _lastDelta = 0;
    _isFirstRead = true;
    _currentRPM = 0.0f;
    _currentRadPerSec = 0.0f;
}
