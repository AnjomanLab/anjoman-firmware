#ifndef MAGNETIC_ENCODER_H
#define MAGNETIC_ENCODER_H

#include <Arduino.h>
#include <Wire.h>
#include <mutex>

class MagneticEncoder {
public:
    // Constructor associating the encoder with its dedicated TCA9548A channel
    MagneticEncoder(TwoWire &wireInstance, uint8_t muxAddress, uint8_t muxChannel);

    // Initializes the physical interface
    bool begin();

    // Selects the TCA9548A channel and reads the raw 12-bit angle (0 to 4095)
    uint16_t readRawAngle();

    // Reads raw angle and converts it to continuous radians taking roll-overs into account
    float getAngleRadians();

    // Resets cumulative odometry steps
    void resetCumulativeAngle();

private:
    TwoWire &_wire;
    uint8_t _muxAddress;
    uint8_t _muxChannel;
    
    // AS5600 constant registers
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t ANGLE_REG_HIGH = 0x0E;

    // Shared I2C Mutex across all encoders and devices on I2C1
    static std::mutex _i2cBusMutex;

    // Odometry tracking variables
    int32_t _cumulativeSteps;
    int16_t _lastRawAngle;
    bool _isFirstRead;

    // Internal helper to switch TCA9548A multiplexer channel
    bool selectMuxChannel();
};

#endif // MAGNETIC_ENCODER_H
