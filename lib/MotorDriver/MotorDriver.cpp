#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t in1, uint8_t in2, uint8_t stby) 
    : pinIN1(in1), pinIN2(in2), pinSTBY(stby) {}

void MotorDriver::begin() {
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(pinSTBY, OUTPUT);
    
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    disable();
}

void MotorDriver::enable() {
    digitalWrite(pinSTBY, HIGH);
}

void MotorDriver::disable() {
    digitalWrite(pinSTBY, LOW);
}

void MotorDriver::drive(int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        analogWrite(pinIN1, speed);
        digitalWrite(pinIN2, LOW);
    } else if (speed < 0) {
        digitalWrite(pinIN1, LOW);
        analogWrite(pinIN2, -speed);
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
    }
}

void MotorDriver::brake() {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, HIGH);
}
