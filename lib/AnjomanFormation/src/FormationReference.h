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
    // ---- Maneuver configuration: continuous rotation, fixed L ----
    // Test profile for long-duration drift characterization:
    //   • L constant at 2 m (no expansion)
    //   • 2 full rotations over 180 s
    //   • Antenna effect is not tested here — the relative
    //     antenna-to-LOS angle is constant during rigid rotation.
    //     This test isolates temporal drift.
    static constexpr float L_INITIAL     = 2.000f;
    static constexpr float L_FINAL       = 2.000f;
    static constexpr float ROT_FINAL_RAD = 4.0f * 3.14159265359f;   // 2 rotations

    static constexpr float T1_ROT_SEC    = 180.0f;    // 3 minutes
    static constexpr float T2_SCALE_SEC  = 0.001f;    // effectively disabled
    static constexpr float T_TOTAL_SEC   = T1_ROT_SEC + T2_SCALE_SEC;
    // ---- Static API ----
    static void  getUnitOffset(uint8_t robotId, float &rx, float &ry);
    static void  evalQuintic(float tau, float &s, float &s_dot);
    static FormationState2D evaluate(uint8_t robotId, float tElapsedSec);
};
