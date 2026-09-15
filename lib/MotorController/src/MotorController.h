#pragma once

#include <Arduino.h>

struct MotorSysIDParams {
    float deadbandFwd;
    float deadbandRev;
    float gainRpmFwd;
    float gainRpmRev;
    float vNominal;
};

struct PIDGains {
    float kp;
    float ki;
    float integralLimit;
};

class MotorController {
public:
    MotorController(uint8_t pinIn1, uint8_t pinIn2, bool invert = false);

    bool begin(uint32_t pwmFreqHz = 20000, uint8_t pwmResolutionBits = 10);

    void setCalibration(const MotorSysIDParams &params);
    void setPIDGains(const PIDGains &gains);

    // Closed-loop velocity control (returns computed PWM command in [-1.0, 1.0])
    float computeVelocityControl(float targetRPM, float measuredRPM, float vBatt, float dt, float crossCoupledTerm = 0.0f);

    // Direct open-loop duty control [-1.0, 1.0]
    void setOpenLoopDuty(float duty);

    void brake();
    void coast();
    void resetPID();

private:
    uint8_t _pinIn1;
    uint8_t _pinIn2;
    bool    _invert;

    uint32_t _pwmFreqHz;
    uint8_t  _pwmResBits;
    uint32_t _maxPwmTicks;

    MotorSysIDParams _params;
    PIDGains         _gains;

    float _integralError;
    float _lastDuty;

    void writeHBridge(float duty);
};
