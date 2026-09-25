#pragma once

#include <Arduino.h>
#include <cmath>

struct FormationState2D {
    float x;      // Target X position (m)
    float y;      // Target Y position (m)
    float vx;     // Target X velocity (m/s)
    float vy;     // Target Y velocity (m/s)
    float phi;    // Formation orientation (rad)
    float L;      // Formation scale (m)
};

class FormationReference {
public:
    static constexpr float L_INITIAL    = 2.000f; // 2.0m initial square
    static constexpr float L_FINAL      = 3.000f; // 3.0m expanded square
    static constexpr float ROT_FINAL_RAD= 1.57079632679f; // +90 deg (pi/2)

    static constexpr float T1_ROT_SEC   = 20.0f; // 20s rotation (v_max = 13.8 cm/s)
    static constexpr float T2_SCALE_SEC = 10.0f; // 10s expansion (v_max = 9.4 cm/s)
    static constexpr float T_TOTAL_SEC  = T1_ROT_SEC + T2_SCALE_SEC; // 30.0s total

    static inline void getUnitOffset(uint8_t robotId, float &rx, float &ry) {
        switch (robotId) {
            case 1: rx = -0.5f; ry = -0.5f; break; // Bottom-Left
            case 2: rx = +0.5f; ry = -0.5f; break; // Bottom-Right
            case 3: rx = +0.5f; ry = +0.5f; break; // Top-Right
            case 4: rx = -0.5f; ry = +0.5f; break; // Top-Left
            default: rx = 0.0f; ry = 0.0f; break;
        }
    }

    // Minimum-Jerk Quintic Polynomial: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    static inline void evalQuintic(float tau, float &s, float &s_dot) {
        if (tau <= 0.0f) { s = 0.0f; s_dot = 0.0f; return; }
        if (tau >= 1.0f) { s = 1.0f; s_dot = 0.0f; return; }
        float tau2 = tau * tau;
        float tau3 = tau2 * tau;
        float tau4 = tau3 * tau;
        s = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau4 * tau;
        s_dot = 30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4; // ds / d_tau
    }

    static inline FormationState2D evaluate(uint8_t robotId, float tElapsedSec) {
        FormationState2D ref = {};
        float rx, ry;
        getUnitOffset(robotId, rx, ry);

        float phi = 0.0f;
        float phi_dot = 0.0f;
        float L = L_INITIAL;
        float L_dot = 0.0f;

        // PHASE 1: Pure Rotation around Center (0 to T1)
        if (tElapsedSec < T1_ROT_SEC) {
            float tau = tElapsedSec / T1_ROT_SEC;
            float s, s_dot;
            evalQuintic(tau, s, s_dot);
            phi = ROT_FINAL_RAD * s;
            phi_dot = (ROT_FINAL_RAD / T1_ROT_SEC) * s_dot;
            L = L_INITIAL;
            L_dot = 0.0f;
        }
        // PHASE 2: Pure Expansion (T1 to T1 + T2)
        else if (tElapsedSec < T_TOTAL_SEC) {
            float tau = (tElapsedSec - T1_ROT_SEC) / T2_SCALE_SEC;
            float s, s_dot;
            evalQuintic(tau, s, s_dot);
            phi = ROT_FINAL_RAD;
            phi_dot = 0.0f;
            L = L_INITIAL + (L_FINAL - L_INITIAL) * s;
            L_dot = ((L_FINAL - L_INITIAL) / T2_SCALE_SEC) * s_dot;
        }
        // MANEUVER COMPLETE: Hold Final Goal
        else {
            phi = ROT_FINAL_RAD;
            phi_dot = 0.0f;
            L = L_FINAL;
            L_dot = 0.0f;
        }

        float cosPhi = cosf(phi);
        float sinPhi = sinf(phi);

        // Desired Position: p*(t) = R(phi) * [L * rx, L * ry]^T
        float px_unrot = L * rx;
        float py_unrot = L * ry;
        ref.x = cosPhi * px_unrot - sinPhi * py_unrot;
        ref.y = sinPhi * px_unrot + cosPhi * py_unrot;

        // Desired Velocity: p_dot*(t)
        float vx_unrot = L_dot * rx;
        float vy_unrot = L_dot * ry;
        float vx_rot = -phi_dot * ref.y;
        float vy_rot =  phi_dot * ref.x;
        float vx_scale = cosPhi * vx_unrot - sinPhi * vy_unrot;
        float vy_scale = sinPhi * vx_unrot + cosPhi * vy_unrot;

        ref.vx  = vx_rot + vx_scale;
        ref.vy  = vy_rot + vy_scale;
        ref.phi = phi;
        ref.L   = L;

        return ref;
    }
};
