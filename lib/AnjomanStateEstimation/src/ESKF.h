#pragma once

#include <Arduino.h>

// ==============================================================================
// Error-State Kalman Filter (local, per-robot)
// ==============================================================================
// Status: SKELETON — full implementation deferred.
//
// Error-state vector (5-dimensional):
//     δx = [δp_x, δp_y, δθ, δω, δb_g]
//
// Nominal state (integrated separately):
//     x  = [p_x, p_y, θ]
// ==============================================================================

class ESKF {
public:
    static constexpr int N_ERROR   = 5;
    static constexpr int N_NOMINAL = 3;

    ESKF();

    void init(float p0x, float p0y, float theta0, float gyroBias0);

    void predict(float vLinear, float omegaMeasured, float gyroZ,
                 float dt, float qPos, float qTheta, float qBias);

    void updateEncoder(float deltaPos, float deltaTheta,
                       float rPos, float rTheta);

    float getX() const { return _px; }
    float getY() const { return _py; }
    float getTheta() const { return _theta; }
    float getGyroBias() const { return _bg; }

private:
    float _px, _py, _theta;
    float _bg;

    // Renamed from `_P` — Arduino WCharacter.h defines `_P` as a macro.
    float _cov[N_ERROR][N_ERROR];

    bool _initialized;
};
