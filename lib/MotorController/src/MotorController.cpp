#include "MotorController.h"

MotorController::MotorController(uint8_t pinIn1, uint8_t pinIn2, uint8_t ledcChan1, uint8_t ledcChan2)
    : _pinIn1(pinIn1), _pinIn2(pinIn2), _chan1(ledcChan1), _chan2(ledcChan2), 
      _freq(5000), _resBits(10), _dutyLimit(1.0f) {}

bool MotorController::begin(uint32_t defaultFreq, uint8_t defaultResBits, float dutyLimit) {
    _freq = defaultFreq;
    _resBits = defaultResBits;
    _dutyLimit = dutyLimit;

    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);

    ledcSetup(_chan1, _freq, _resBits);
    ledcSetup(_chan2, _freq, _resBits);

    ledcAttachPin(_pinIn1, _chan1);
    ledcAttachPin(_pinIn2, _chan2);

    brake();
    return true;
}

void MotorController::reconfigure(uint32_t newFreq, uint8_t newResBits) {
    _freq = newFreq;
    _resBits = newResBits;
    
    // De-attach first to prevent pin allocation faults on core registers
    ledcDetachPin(_pinIn1);
    ledcDetachPin(_pinIn2);

    ledcSetup(_chan1, _freq, _resBits);
    ledcSetup(_chan2, _freq, _resBits);

    ledcAttachPin(_pinIn1, _chan1);
    ledcAttachPin(_pinIn2, _chan2);
}

void MotorController::setSpeed(float speed) {
    speed = constrain(speed, -1.0f, 1.0f);
    
    uint32_t max_val = (1 << _resBits) - 1;
    uint32_t maxAllowedDuty = (uint32_t)((float)max_val * _dutyLimit);

    if (speed > 0.01f) {
        uint32_t duty = (uint32_t)(speed * (float)maxAllowedDuty);
        ledcWrite(_chan1, duty);
        ledcWrite(_chan2, 0);
    } else if (speed < -0.01f) {
        uint32_t duty = (uint32_t)(-speed * (float)maxAllowedDuty);
        ledcWrite(_chan1, 0);
        ledcWrite(_chan2, duty);
    } else {
        brake();
    }
}

void MotorController::brake() {
    uint32_t max_val = (1 << _resBits) - 1;
    ledcWrite(_chan1, max_val);
    ledcWrite(_chan2, max_val);
}
