#include "MotorController.h"
#include <cmath>

MotorController::MotorController(uint8_t pinIn1, uint8_t pinIn2, bool invert)
    : _pinIn1(pinIn1),
      _pinIn2(pinIn2),
      _invert(invert),
      _pwmFreqHz(20000),
      _pwmResBits(10),
      _maxPwmTicks(1023),
      _integralError(0.0f),
      _lastDuty(0.0f) {
    _params = {0.40f, 0.40f, 200.0f, 200.0f, 7.40f};
    _gains  = {0.0025f, 0.015f, 0.30f};
}

bool MotorController::begin(uint32_t pwmFreqHz, uint8_t pwmResolutionBits) {
    _pwmFreqHz = pwmFreqHz;
    _pwmResBits = pwmResolutionBits;
    _maxPwmTicks = (1 << _pwmResBits) - 1;

    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);

    // Modern ESP32 Arduino Core 3.x API
    ledcAttach(_pinIn1, _pwmFreqHz, _pwmResBits);
    ledcAttach(_pinIn2, _pwmFreqHz, _pwmResBits);

    brake();
    return true;
}

void MotorController::setCalibration(const MotorSysIDParams &params) {
    _params = params;
}

void MotorController::setPIDGains(const PIDGains &gains) {
    _gains = gains;
}

void MotorController::resetPID() {
    _integralError = 0.0f;
    _lastDuty = 0.0f;
}

float MotorController::computeVelocityControl(float targetRPM, float measuredRPM, float vBatt, float dt, float crossCoupledTerm) {
    if (fabs(targetRPM) < 0.5f) {
        resetPID();
        brake();
        return 0.0f;
    }

    // Protect against invalid voltage readings
    if (vBatt < 5.5f) {
        vBatt = _params.vNominal;
    }

    float vRatio = _params.vNominal / vBatt;
    bool isFwd = (targetRPM >= 0.0f);

    float deadband = isFwd ? _params.deadbandFwd : _params.deadbandRev;
    float gain     = isFwd ? _params.gainRpmFwd  : _params.gainRpmRev;

    // 1. Voltage-Compensated Feedforward
    float u_ff = 0.0f;
    if (gain > 1.0f) {
        float baseEffort = (fabs(targetRPM) / gain) * vRatio;
        float deadbandComp = deadband * vRatio;
        u_ff = (isFwd ? 1.0f : -1.0f) * (deadbandComp + baseEffort);
    }

    // 2. Closed-Loop Feedback (PI)
    float error = targetRPM - measuredRPM;
    
    _integralError += error * dt;
    _integralError = constrain(_integralError, -_gains.integralLimit, _gains.integralLimit);

    float u_fb = (_gains.kp * error) + (_gains.ki * _integralError);

    // 3. Combine Feedforward + Feedback + Cross-Coupling Correction
    float u_total = u_ff + u_fb + crossCoupledTerm;
    u_total = constrain(u_total, -1.0f, 1.0f);

    _lastDuty = u_total;
    writeHBridge(u_total);

    return u_total;
}

void MotorController::setOpenLoopDuty(float duty) {
    duty = constrain(duty, -1.0f, 1.0f);
    _lastDuty = duty;
    writeHBridge(duty);
}

void MotorController::writeHBridge(float duty) {
    if (_invert) {
        duty = -duty;
    }

    if (duty > 0.01f) {
        uint32_t val = (uint32_t)(fabs(duty) * (float)_maxPwmTicks);
        ledcWrite(_pinIn1, val);
        ledcWrite(_pinIn2, 0);
    } else if (duty < -0.01f) {
        uint32_t val = (uint32_t)(fabs(duty) * (float)_maxPwmTicks);
        ledcWrite(_pinIn1, 0);
        ledcWrite(_pinIn2, val);
    } else {
        brake();
    }
}

void MotorController::brake() {
    ledcWrite(_pinIn1, _maxPwmTicks);
    ledcWrite(_pinIn2, _maxPwmTicks);
}

void MotorController::coast() {
    ledcWrite(_pinIn1, 0);
    ledcWrite(_pinIn2, 0);
}
