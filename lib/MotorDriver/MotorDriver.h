#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
    uint8_t pinIN1;
    uint8_t pinIN2;
    uint8_t pinSTBY;

public:
    MotorDriver(uint8_t in1, uint8_t in2, uint8_t stby);

    void begin();
    void enable();
    void disable();
    void drive(int speed);
    void brake();
};

#endif
