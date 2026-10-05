#pragma once

#include <Arduino.h>

struct FormationState2D {
    float x;
    float y;
    float vx;
    float vy;
    float phi;
    float L;
};

class FormationReference {
public:
    static constexpr float L_INITIAL     = 2.000f;
    static constexpr float L_FINAL       = 3.000f;
    
    // Feasible rotation: 90 degrees CCW (pi/2) to stay strictly within 0.21 m/s limit
    static constexpr float ROT_FINAL_RAD = 1.57079632679f;

    static constexpr float T1_ROT_SEC    = 20.0f;
    static constexpr float T2_SCALE_SEC  = 10.0f;
    static constexpr float T_TOTAL_SEC   = T1_ROT_SEC + T2_SCALE_SEC;

    static void  getUnitOffset(uint8_t robotId, float &rx, float &ry);
    static void  evalQuintic(float tau, float &s, float &s_dot);
    static FormationState2D evaluate(uint8_t robotId, float tElapsedSec);
};
