#pragma once

#include <cmath>

// ==============================================================================
// Heading Kalman Filter — 2-state EKF (yaw, gyro bias)
// ==============================================================================
// Predict:  θ ← θ + (ω_gyro − b_g) · dt
// Update:   θ ← f(Δθ_encoder)   with χ²-slip gating
// ==============================================================================

class HeadingKalmanFilter {
public:
    HeadingKalmanFilter()
        : _theta(0.0f), _bias(0.0f),
          _P00(1.0e-4f), _P01(0.0f), _P10(0.0f), _P11(1.0e-5f),
          _qYaw(6.30e-9f), _qBias(1.00e-12f), _rEnc(1.50e-6f) {}

    void init(float initialHeadingRad, float initialBiasRadS,
              float qYaw, float qBias, float rEnc) {
        _theta = initialHeadingRad;
        _bias  = initialBiasRadS;
        _qYaw  = qYaw;
        _qBias = qBias;
        _rEnc  = rEnc;
        _P00   = 1.0e-4f;
        _P01   = 0.0f;
        _P10   = 0.0f;
        _P11   = 1.0e-5f;
    }

    void predict(float gyroZRadS, float dt) {
        float rate = gyroZRadS - _bias;
        _theta += rate * dt;

        float P00_temp = _P00 - dt * (_P01 + _P10) + dt * dt * _P11 + _qYaw;
        float P01_temp = _P01 - dt * _P11;
        float P10_temp = _P10 - dt * _P11;
        float P11_temp = _P11 + _qBias;

        _P00 = P00_temp;
        _P01 = P01_temp;
        _P10 = P10_temp;
        _P11 = P11_temp;
    }

    bool updateEncoder(float deltaThetaWheelRad, float gyroZRadS, float dt) {
        float predictedDelta = (gyroZRadS - _bias) * dt;
        float y_tilde = deltaThetaWheelRad - predictedDelta;

        float S = _P00 + _rEnc;
        float mahalanobisSq = (y_tilde * y_tilde) / S;
        float rateDivergence = fabsf(deltaThetaWheelRad / dt - gyroZRadS);

        if (mahalanobisSq > 6.635f || rateDivergence > 0.35f) {
            return false;
        }

        float K0 = _P00 / S;
        float K1 = _P10 / S;

        _theta += K0 * y_tilde;
        _bias  += K1 * y_tilde;

        float P00_old = _P00;
        float P01_old = _P01;

        _P00 = (1.0f - K0) * P00_old;
        _P01 = (1.0f - K0) * P01_old;
        _P10 = _P10 - K1 * P00_old;
        _P11 = _P11 - K1 * P01_old;

        float p_sym = (_P01 + _P10) * 0.5f;
        _P01 = _P10 = p_sym;

        return true;
    }

    float getHeadingRad() const { return _theta; }
    float getHeadingDeg() const { return _theta * (180.0f / 3.141592653589793f); }
    float getBiasRadS()   const { return _bias; }
    float getBiasDegS()   const { return _bias * (180.0f / 3.141592653589793f); }
    float getVarTheta()   const { return _P00; }

private:
    float _theta;
    float _bias;
    float _P00, _P01, _P10, _P11;
    float _qYaw, _qBias, _rEnc;
};
