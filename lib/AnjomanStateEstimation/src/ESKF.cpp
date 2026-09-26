#include "ESKF.h"
#include <math.h>
#include <string.h>

ESKF::ESKF()
    : _px(0.0f), _py(0.0f), _theta(0.0f), _bg(0.0f), _initialized(false) {
    memset(_cov, 0, sizeof(_cov));
}

void ESKF::init(float p0x, float p0y, float theta0, float gyroBias0) {
    _px    = p0x;
    _py    = p0y;
    _theta = theta0;
    _bg    = gyroBias0;

    memset(_cov, 0, sizeof(_cov));
    for (int i = 0; i < N_ERROR; i++) {
        _cov[i][i] = 1e-4f;
    }

    _initialized = true;
}

void ESKF::predict(float vLinear, float omegaMeasured, float gyroZ,
                   float dt, float qPos, float qTheta, float qBias) {
    if (!_initialized) return;

    // Nominal state propagation (midpoint integration)
    const float thetaMid = _theta + 0.5f * (omegaMeasured - _bg) * dt;
    _px    += vLinear * cosf(thetaMid) * dt;
    _py    += vLinear * sinf(thetaMid) * dt;
    _theta += (omegaMeasured - _bg) * dt;

    // Covariance diagonal inflation
    _cov[0][0] += qPos   * dt;
    _cov[1][1] += qPos   * dt;
    _cov[2][2] += qTheta * dt;
    _cov[3][3] += qTheta * dt;
    _cov[4][4] += qBias  * dt;
}

void ESKF::updateEncoder(float deltaPos, float deltaTheta,
                         float rPos, float rTheta) {
    if (!_initialized) return;

    // TODO: implement proper measurement update.
    (void)deltaPos; (void)deltaTheta;
    (void)rPos;     (void)rTheta;
}
