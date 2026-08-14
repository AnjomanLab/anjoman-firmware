#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
public:
    MotorController(uint8_t pinIn1, uint8_t pinIn2, uint8_t ledcChan1, uint8_t ledcChan2);

    // Initial configuration
    bool begin(uint32_t defaultFreq, uint8_t defaultResBits, float dutyLimit);

    // Dynamic runtime update of frequency and bit resolution
    void reconfigure(uint32_t newFreq, uint8_t newResBits);

    // Set speed normalized from -1.0f to 1.0f
    void setSpeed(float speed);

    // Hard electrical brake
    void brake();

private:
    uint8_t _pinIn1;
    uint8_t _pinIn2;
    uint8_t _chan1;
    uint8_t _chan2;
    uint32_t _freq;
    uint8_t _resBits;
    float _dutyLimit;
};

#endif // MOTOR_CONTROLLER_H
