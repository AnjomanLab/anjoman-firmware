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
    // ---- Maneuver: equilateral triangle, 2 rotations, then scale 2→3 m ----
    //   • Initial: equilateral triangle, side = 2.0 m
    //   • All robots face +X at t=0
    //   • Phase 1: 2 full CCW rotations (20 s)
    //   • Phase 2: expansion from L=2 to L=3 (10 s)
    static constexpr float L_INITIAL     = 2.000f;
    static constexpr float L_FINAL       = 3.000f;
//    static constexpr float ROT_FINAL_RAD = 4.0f * 3.14159265359f;  // 2 rotations
    static constexpr float ROT_FINAL_RAD = 4.0f * 3.14159265359f + 2.0f * 3.14159265359f / 3.0f;  // 4π + 2π/3

    static constexpr float T1_ROT_SEC    = 20.0f;
    static constexpr float T2_SCALE_SEC  = 10.0f;
    static constexpr float T_TOTAL_SEC   = T1_ROT_SEC + T2_SCALE_SEC;

    // ---- Static API ----
    static void  getUnitOffset(uint8_t robotId, float &rx, float &ry);
    static void  evalQuintic(float tau, float &s, float &s_dot);
    static FormationState2D evaluate(uint8_t robotId, float tElapsedSec);
};
