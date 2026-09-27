#pragma once

#include <Arduino.h>

// ==============================================================================
// Error-State Kalman Filter (local, per-robot)
// ==============================================================================
// Local error-state EKF for wheel-encoder + gyro fusion.
//
// Nominal state (4-dim):
//     x = [p_x, p_y, θ, b_g]
//
// Error state (4-dim):
//     δx = [δp_x, δp_y, δθ, δb_g]
//
// Continuous-time dynamics:
//     ṗ_x = v · cos(θ)
//     ṗ_y = v · sin(θ)
//     θ̇  = ω_m − b_g            (ω_m = raw gyro Z reading)
//     ḃ_g = 0                   (random walk)
//
// Error dynamics (first-order linearization):
//     δṗ_x = −v · sin(θ) · δθ
//     δṗ_y = +v · cos(θ) · δθ
//     δθ̇  = −δb_g
//     δḃ_g = 0
//
// Encoder measurement (angular increment):
//     z = Δθ_enc
//     h(x) = Δθ_nom
//     H = [0, 0, 1, 0]
//
// Sign conventions:
//     θ is CCW-positive, wrapped to (−π, π] after each step.
//     Gyro Z axis is CCW-positive.
// ==============================================================================

class ESKF {
public:
    static constexpr int N = 4;   // error-state dimension

    ESKF();

    // Initialize with known initial pose and initial gyro bias estimate.
    void init(float p0x, float p0y, float theta0, float gyroBias0);

    // Predict step — call at the control rate (e.g. 100 Hz).
    //   vLinear  : forward linear velocity from wheel encoders [m/s]
    //   gyroZ    : raw gyro Z reading [rad/s]  (NOT bias-corrected)
    //   dt       : time step [s]
    //   qPos     : position process noise density [m²/s]
    //   qTheta   : heading process noise density [rad²/s]
    //   qBias    : gyro bias random walk density [rad²/s³]
    void predict(float vLinear, float gyroZ, float dt,
                 float qPos, float qTheta, float qBias);

    // Measurement update using the encoder-derived angular increment.
    //   deltaThetaEnc : rotation increment from differential wheels [rad]
    //   gyroZ         : the SAME raw gyro reading used in predict [rad/s]
    //   dt            : the SAME time step used in predict [s]
    //   rTheta        : measurement noise variance [rad²]
    // Returns true if the update was accepted (not gated out).
    bool updateEncoder(float deltaThetaEnc, float gyroZ, float dt,
                       float rTheta);

    // Accessors
    float getX() const        { return _px;    }
    float getY() const        { return _py;    }
    float getTheta() const    { return _theta; }
    float getGyroBias() const { return _bg;    }

    // Diagnostics
    float getVarTheta() const { return _cov[2][2]; }
    float getVarBias()  const { return _cov[3][3]; }
    float getVarX() const { return _cov[0][0]; }
    float getVarY() const { return _cov[1][1]; }

    // Number of accepted/rejected updates (for diagnostics only)
    uint32_t getAcceptedUpdates() const { return _nAccepted; }
    uint32_t getRejectedUpdates() const { return _nRejected; }

private:
    // Nominal state
    float _px;
    float _py;
    float _theta;
    float _bg;

    // Error-state covariance (4×4)
    float _cov[N][N];

    // Counters
    uint32_t _nAccepted;
    uint32_t _nRejected;

    bool _initialized;
};
