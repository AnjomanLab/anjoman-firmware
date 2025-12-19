#ifndef MAGNETIC_ENCODER_H
#define MAGNETIC_ENCODER_H

#include <Arduino.h>
#include <Wire.h>

class MagneticEncoder {
private:
    uint16_t _lastRawAngle;
    int64_t _totalRevolutions;
    const uint8_t _deviceAddress = 0x36;

    uint16_t readRawAngle();

public:
    MagneticEncoder();

    void begin();
    void update();
    
    int64_t getCumulativePosition();
    float getRevolutions();
    bool isMagnetDetected();
    bool isMagnetTooStrong();
    bool isMagnetTooWeak();
};

#endif
