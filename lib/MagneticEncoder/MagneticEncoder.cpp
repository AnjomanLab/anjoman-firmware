#include "MagneticEncoder.h"

#define REG_STATUS 0x0B
#define REG_RAW_ANGLE 0x0C

MagneticEncoder::MagneticEncoder() {
    _lastRawAngle = 0;
    _totalRevolutions = 0;
}

void MagneticEncoder::begin() {
    // Initial read to set the starting point
    _lastRawAngle = readRawAngle();
    _totalRevolutions = 0;
}

uint16_t MagneticEncoder::readRawAngle() {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(REG_RAW_ANGLE);
    Wire.endTransmission();
    
    // Request 2 bytes (High byte, Low byte)
    Wire.requestFrom(_deviceAddress, (uint8_t)2);
    
    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        // Mask with 0x0FFF because resolution is 12-bit
        return ((msb << 8) | lsb) & 0x0FFF;
    }
    
    return _lastRawAngle;
}

void MagneticEncoder::update() {
    uint16_t currentRaw = readRawAngle();
    int32_t delta = currentRaw - _lastRawAngle;

    // 12-bit range is 0-4095. Halfway is 2048.
    // If jump is larger than half range, it's a wrap-around.
    if (delta > 2048) {
        _totalRevolutions--;
    } else if (delta < -2048) {
        _totalRevolutions++;
    }

    _lastRawAngle = currentRaw;
}

int64_t MagneticEncoder::getCumulativePosition() {
    return (_totalRevolutions * 4096) + _lastRawAngle;
}

float MagneticEncoder::getRevolutions() {
    return (float)getCumulativePosition() / 4096.0f;
}

bool MagneticEncoder::isMagnetDetected() {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(REG_STATUS);
    Wire.endTransmission();
    
    Wire.requestFrom(_deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        // Bit 5: MD (Magnet Detected)
        return (status & 0x20); 
    }
    return false;
}

bool MagneticEncoder::isMagnetTooWeak() {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(REG_STATUS);
    Wire.endTransmission();
    
    Wire.requestFrom(_deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        // Bit 4: ML (Magnet Low/Weak)
        return (status & 0x10); 
    }
    return false;
}

bool MagneticEncoder::isMagnetTooStrong() {
    Wire.beginTransmission(_deviceAddress);
    Wire.write(REG_STATUS);
    Wire.endTransmission();
    
    Wire.requestFrom(_deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        // Bit 3: MH (Magnet High/Strong)
        return (status & 0x08); 
    }
    return false;
}
