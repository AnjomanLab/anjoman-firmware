#include "MotorController.h"
#include <cmath>

// ---------------------------------------------------------------------------
// Static channel allocator — 4 channels total (2 per MotorController instance)
// Arduino Core 2.x assigns a channel per pin for LEDC.
// ---------------------------------------------------------------------------
static uint8_t s_nextChannel = 0;

MotorController::MotorController(uint8_t pinIn1, uint8_t pinIn2, bool invert)
    : _pinIn1(pinIn1),
      _pinIn2(pinIn2),
      _ch1(0),
      _ch2(0),
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
    _pwmFreqHz   = pwmFreqHz;
    _pwmResBits  = pwmResolutionBits;
    _maxPwmTicks = (1u << _pwmResBits) - 1u;

    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);

    // Allocate two LEDC channels from the static pool (0..15)
    if (s_nextChannel + 1 >= 16) {
        return false;   // out of channels
    }
    _ch1 = s_nextChannel++;
    _ch2 = s_nextChannel++;

    // Arduino Core 2.x API: setup channel first, then attach a pin to it.
    ledcSetup(_ch1, _pwmFreqHz, _pwmResBits);
    ledcSetup(_ch2, _pwmFreqHz, _pwmResBits);
    ledcAttachPin(_pinIn1, _ch1);
    ledcAttachPin(_pinIn2, _ch2);

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

float MotorController::computeVelocityControl(float targetRPM, float measuredRPM,
                                              float vBatt, float dt,
                                              float crossCoupledTerm) {
    if (fabsf(targetRPM) < 0.5f) {
        resetPID();
        brake();
        return 0.0f;
    }

    if (vBatt < 5.5f) {
        vBatt = _params.vNominal;
    }

    const float vRatio = _params.vNominal / vBatt;
    const bool  isFwd  = (targetRPM >= 0.0f);

    const float deadband = isFwd ? _params.deadbandFwd : _params.deadbandRev;
    const float gain     = isFwd ? _params.gainRpmFwd  : _params.gainRpmRev;

    // 1) Feedforward (voltage-compensated)
    float u_ff = 0.0f;
    if (gain > 1.0f) {
        const float baseEffort    = (fabsf(targetRPM) / gain) * vRatio;
        const float deadbandComp  = deadband * vRatio;
        u_ff = (isFwd ? 1.0f : -1.0f) * (deadbandComp + baseEffort);
    }

    // 2) PI feedback
    const float error = targetRPM - measuredRPM;
    _integralError += error * dt;
    _integralError = constrain(_integralError,
                               -_gains.integralLimit,
                                _gains.integralLimit);
    const float u_fb = _gains.kp * error + _gains.ki * _integralError;

    // 3) Combine
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
        const uint32_t val = (uint32_t)(fabsf(duty) * (float)_maxPwmTicks);
        ledcWrite(_ch1, val);
        ledcWrite(_ch2, 0);
    } else if (duty < -0.01f) {
        const uint32_t val = (uint32_t)(fabsf(duty) * (float)_maxPwmTicks);
        ledcWrite(_ch1, 0);
        ledcWrite(_ch2, val);
    } else {
        brake();
    }
}

void MotorController::brake() {
    ledcWrite(_ch1, _maxPwmTicks);
    ledcWrite(_ch2, _maxPwmTicks);
}

void MotorController::coast() {
    ledcWrite(_ch1, 0);
    ledcWrite(_ch2, 0);
}
