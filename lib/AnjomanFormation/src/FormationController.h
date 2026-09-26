#pragma once

#include <Arduino.h>
#include "FormationReference.h"

struct WheelVelocityCommand {
    float vLinear;    // Chassis linear velocity (m/s)
    float omegaRadS;  // Chassis angular velocity (rad/s)
};

class FormationController {
public:
    // Look-ahead distance from wheel axle center to virtual point (m)
    static constexpr float LP_OFFSET_M = 0.050f;

    // Position tracking gain (1/s)
    static constexpr float KP_POS      = 2.0f;

    // Saturation ceiling for angular velocity (rad/s)
    static constexpr float MAX_OMEGA   = 2.0f;

    static WheelVelocityCommand compute(
        uint8_t robotId,
        const FormationState2D &target,
        float currentX,
        float currentY,
        float currentThetaRad
    );
};
