#pragma once

#include <Arduino.h>

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
    // ---- Maneuver configuration ----
    static constexpr float L_INITIAL     = 2.000f;
    static constexpr float L_FINAL       = 3.000f;
    static constexpr float ROT_FINAL_RAD = 1.57079632679f;   // +90 deg CCW

    static constexpr float T1_ROT_SEC    = 20.0f;            // Rotation phase
    static constexpr float T2_SCALE_SEC  = 10.0f;            // Expansion phase
    static constexpr float T_TOTAL_SEC   = T1_ROT_SEC + T2_SCALE_SEC;

    // ---- Static API ----
    static void  getUnitOffset(uint8_t robotId, float &rx, float &ry);
    static void  evalQuintic(float tau, float &s, float &s_dot);
    static FormationState2D evaluate(uint8_t robotId, float tElapsedSec);
};
