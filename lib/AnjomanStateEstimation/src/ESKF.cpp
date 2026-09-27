#include "ESKF.h"
#include <math.h>
#include <string.h>

namespace {

    constexpr float PI_F       = 3.14159265358979323846f;
    constexpr float TWO_PI_F   = 2.0f * PI_F;

    // Wrap an angle to (−π, π].
    inline float wrapPi(float a) {
        while (a >  PI_F) a -= TWO_PI_F;
        while (a < -PI_F) a += TWO_PI_F;
        return a;
    }

    // Chi-square 1-DOF 99% threshold for gating.
    constexpr float CHI2_1DOF_99 = 6.635f;

    // Rate divergence threshold for slip detection [rad/s].
    constexpr float RATE_DIVERGENCE_LIMIT = 0.35f;

} // namespace

// ============================================================================
// Construction / initialization
// ============================================================================

ESKF::ESKF()
    : _px(0.0f), _py(0.0f), _theta(0.0f), _bg(0.0f),
      _nAccepted(0), _nRejected(0), _initialized(false) {
    memset(_cov, 0, sizeof(_cov));
}

void ESKF::init(float p0x, float p0y, float theta0, float gyroBias0) {
    _px    = p0x;
    _py    = p0y;
    _theta = wrapPi(theta0);
    _bg    = gyroBias0;

    memset(_cov, 0, sizeof(_cov));

    // Initial uncertainties (tuned for hand-placed robot on a known square)
    //   position  : ~1 cm → variance 1e-4 m²
    //   heading   : ~0.6° → variance 1e-4 rad²
    //   gyro bias : ~0.01 rad/s → variance 1e-4 (rad/s)²
    _cov[0][0] = 1e-4f;   // p_x
    _cov[1][1] = 1e-4f;   // p_y
    _cov[2][2] = 1e-4f;   // θ
    _cov[3][3] = 1e-4f;   // b_g

    _nAccepted = 0;
    _nRejected = 0;

    _initialized = true;
}

// ============================================================================
// Predict — nominal-state propagation and covariance prediction
// ============================================================================

void ESKF::predict(float vLinear, float gyroZ, float dt,
                   float qPos, float qTheta, float qBias) {
    if (!_initialized) return;

    // ---------- Nominal state propagation (midpoint integration) ----------
    const float thetaPrior  = _theta;
    const float omegaCorr   = gyroZ - _bg;
    const float dTheta      = omegaCorr * dt;
    const float thetaMid    = thetaPrior + 0.5f * dTheta;

    _px    += vLinear * cosf(thetaMid) * dt;
    _py    += vLinear * sinf(thetaMid) * dt;
    _theta  = wrapPi(thetaPrior + dTheta);
    // _bg is unchanged (bias random walk has zero mean).

    // ---------- Covariance prediction: P = F · P · Fᵀ + Q ----------
    //
    // Linearization at (thetaPrior, vLinear):
    //     F = I + Fc · dt
    //     Fc[0][2] = -v · sin(θ)
    //     Fc[1][2] = +v · cos(θ)
    //     Fc[2][3] = -1
    //
    const float sinT = sinf(thetaPrior);
    const float cosT = cosf(thetaPrior);

    const float F[N][N] = {
        { 1.0f, 0.0f, -vLinear * sinT * dt,  0.0f },
        { 0.0f, 1.0f,  vLinear * cosT * dt,  0.0f },
        { 0.0f, 0.0f,  1.0f,                 -dt  },
        { 0.0f, 0.0f,  0.0f,                  1.0f }
    };

    // Compute FP = F · P (row-major, small fixed size)
    float FP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N; ++k) s += F[i][k] * _cov[k][j];
            FP[i][j] = s;
        }
    }

    // Compute newP = FP · Fᵀ
    float newP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N; ++k) s += FP[i][k] * F[j][k];   // F[j][k] = Fᵀ[k][j]
            newP[i][j] = s;
        }
    }

    // Add process noise (diagonal)
    newP[0][0] += qPos   * dt;
    newP[1][1] += qPos   * dt;
    newP[2][2] += qTheta * dt;
    newP[3][3] += qBias  * dt;

    memcpy(_cov, newP, sizeof(_cov));
}

// ============================================================================
// Update — encoder angular increment
// ============================================================================

bool ESKF::updateEncoder(float deltaThetaEnc, float gyroZ, float dt,
                         float rTheta) {
    if (!_initialized) return false;

    // ---------- Innovation ----------
    //   z = deltaThetaEnc
    //   h = dTheta_nom = (gyroZ - b_g) · dt
    //   y = z − h
    const float dThetaNom = (gyroZ - _bg) * dt;
    const float y         = wrapPi(deltaThetaEnc - dThetaNom);

    // ---------- Gating ----------
    // (a) chi-square on normalized innovation
    const float S = _cov[2][2] + rTheta;
    const float mahalanobisSq = (y * y) / S;
    if (mahalanobisSq > CHI2_1DOF_99) {
        ++_nRejected;
        return false;
    }

    // (b) rate divergence (slip heuristic)
    const float rateDivergence = fabsf(deltaThetaEnc / dt - gyroZ);
    if (rateDivergence > RATE_DIVERGENCE_LIMIT) {
        ++_nRejected;
        return false;
    }

    // ---------- Kalman gain K = P · Hᵀ / S = P[:,2] / S ----------
    float K[N];
    for (int i = 0; i < N; ++i) K[i] = _cov[i][2] / S;

    // ---------- State update (inject error correction) ----------
    _px    += K[0] * y;
    _py    += K[1] * y;
    _theta  = wrapPi(_theta + K[2] * y);
    _bg    += K[3] * y;

    // ---------- Covariance update: Joseph form for numerical stability ----------
    //
    //   IKH = I − K · H         with H = [0, 0, 1, 0]
    //   P_new = IKH · P · IKHᵀ + K · rTheta · Kᵀ
    //
    float IKH[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            const float Hj = (j == 2) ? 1.0f : 0.0f;
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * Hj;
        }
    }

    float IKH_P[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N; ++k) s += IKH[i][k] * _cov[k][j];
            IKH_P[i][j] = s;
        }
    }

    float newP[N][N];
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            float s = 0.0f;
            for (int k = 0; k < N; ++k) s += IKH_P[i][k] * IKH[j][k]; // IKHᵀ
            newP[i][j] = s + K[i] * rTheta * K[j];
        }
    }

    // Enforce symmetry
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            const float sym = 0.5f * (newP[i][j] + newP[j][i]);
            newP[i][j] = sym;
            newP[j][i] = sym;
        }
    }

    memcpy(_cov, newP, sizeof(_cov));

    ++_nAccepted;
    return true;
}
