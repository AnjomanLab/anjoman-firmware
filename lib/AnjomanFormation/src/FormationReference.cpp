#include "FormationReference.h"
#include <cmath>

void FormationReference::getUnitOffset(uint8_t robotId, float &rx, float &ry) {
    // Equilateral triangle, side = 2 m, centroid at origin.
    // r_i = (2/√3) * unit offset, so |r_i - r_j| = 1 (unit shape).
    // Vertices:
    //   R2: (0, +1/√3) * 2/√3 = (0, +0.6667)  → scaled to distance 1
    //   R3: (-0.5, -1/(2√3)) * 2/√3            → scaled to distance 1
    //   R4: (+0.5, -1/(2√3)) * 2/√3            → scaled to distance 1
    //
    // Unit offsets (after dividing by circumradius):
    constexpr float S3 = 1.73205080757f;   // sqrt(3)
    switch (robotId) {
        case 2: rx =  0.0f;         ry = +2.0f / (2.0f * S3); break;  // (0, 1/√3) ≈ (0, 0.577)
        case 3: rx = -1.0f / (2.0f); ry = -1.0f / (2.0f * S3); break;
        case 4: rx = +1.0f / (2.0f); ry = -1.0f / (2.0f * S3); break;
        default: rx = 0.0f; ry = 0.0f; break;
    }
}

// Minimum-jerk quintic: s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
void FormationReference::evalQuintic(float tau, float &s, float &s_dot) {
    if (tau <= 0.0f) { s = 0.0f; s_dot = 0.0f; return; }
    if (tau >= 1.0f) { s = 1.0f; s_dot = 0.0f; return; }

    float tau2 = tau * tau;
    float tau3 = tau2 * tau;
    float tau4 = tau3 * tau;

    s     = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau4 * tau;
    s_dot = 30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4;
}

FormationState2D FormationReference::evaluate(uint8_t robotId, float tElapsedSec) {
    FormationState2D ref = {};
    float rx, ry;
    getUnitOffset(robotId, rx, ry);

    float phi     = 0.0f;
    float phi_dot = 0.0f;
    float L       = L_INITIAL;
    float L_dot   = 0.0f;

    if (tElapsedSec < T1_ROT_SEC) {
        // Phase 1: pure rotation
        float tau = tElapsedSec / T1_ROT_SEC;
        float s, s_dot;
        evalQuintic(tau, s, s_dot);
        phi     = ROT_FINAL_RAD * s;
        phi_dot = (ROT_FINAL_RAD / T1_ROT_SEC) * s_dot;
        L       = L_INITIAL;
        L_dot   = 0.0f;
    } else if (tElapsedSec < T_TOTAL_SEC) {
        // Phase 2: pure expansion
        float tau = (tElapsedSec - T1_ROT_SEC) / T2_SCALE_SEC;
        float s, s_dot;
        evalQuintic(tau, s, s_dot);
        phi     = ROT_FINAL_RAD;
        phi_dot = 0.0f;
        L       = L_INITIAL + (L_FINAL - L_INITIAL) * s;
        L_dot   = ((L_FINAL - L_INITIAL) / T2_SCALE_SEC) * s_dot;
    } else {
        // Hold final pose
        phi     = ROT_FINAL_RAD;
        phi_dot = 0.0f;
        L       = L_FINAL;
        L_dot   = 0.0f;
    }

    float cosPhi = cosf(phi);
    float sinPhi = sinf(phi);

    float px_unrot = L * rx;
    float py_unrot = L * ry;
    ref.x = cosPhi * px_unrot - sinPhi * py_unrot;
    ref.y = sinPhi * px_unrot + cosPhi * py_unrot;

    float vx_unrot = L_dot * rx;
    float vy_unrot = L_dot * ry;
    float vx_rot   = -phi_dot * ref.y;
    float vy_rot   =  phi_dot * ref.x;
    float vx_scale = cosPhi * vx_unrot - sinPhi * vy_unrot;
    float vy_scale = sinPhi * vx_unrot + cosPhi * vy_unrot;

    ref.vx  = vx_rot + vx_scale;
    ref.vy  = vy_rot + vy_scale;
    ref.phi = phi;
    ref.L   = L;

    return ref;
}
