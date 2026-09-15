#pragma once

#include <Arduino.h>
#include <Wire.h>

class MagneticEncoder {
public:
    MagneticEncoder(TwoWire &wireInstance, uint8_t muxAddress, uint8_t muxChannel, bool invert = false);

    bool begin();
    bool update(float dt);

    uint16_t getRawAngle() const;
    int16_t  getDeltaSteps() const;
    int32_t  getCumulativeSteps() const;
    float    getAngleRadians() const;
    float    getRPM() const;
    float    getRadPerSec() const;

    void resetCumulativeAngle();

private:
    TwoWire &_wire;
    uint8_t _muxAddress;
    uint8_t _muxChannel;
    bool    _invert;

    static constexpr uint8_t AS5600_ADDR      = 0x36;
    static constexpr uint8_t ANGLE_REG_HIGH   = 0x0E;
    static constexpr float   ENCODER_CPR      = 4096.0f;
    static constexpr float   TWO_PI_F         = 6.28318530718f;

    uint16_t _currentRawAngle;
    int16_t  _lastRawAngle;
    int16_t  _lastDelta;
    int32_t  _cumulativeSteps;
    bool     _isFirstRead;

    float    _currentRPM;
    float    _currentRadPerSec;

    bool selectMuxChannel();
};
