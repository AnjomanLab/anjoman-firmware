#pragma once

#include <Arduino.h>
#include <cmath>
#include "RobotConfig.h"
#include "FormationReference.h"

struct WheelVelocityCommand {
    float vLinear;   // Chassis linear velocity (m/s)
    float omegaRadS; // Chassis angular velocity (rad/s)
};

class FormationController {
public:
    static constexpr float LP_OFFSET_M = 0.050f; // 50 mm look-ahead
    static constexpr float KP_POS      = 2.0f;   // Position tracking gain (s^-1)

    static inline WheelVelocityCommand compute(
        uint8_t robotId,
        const FormationState2D &target,
        float currentX,
        float currentY,
        float currentThetaRad
    ) {
        WheelVelocityCommand cmd = {};

        float cosTh = cosf(currentThetaRad);
        float sinTh = sinf(currentThetaRad);

        // Error between target position and robot axle center
        float errX = target.x - currentX;
        float errY = target.y - currentY;

        // Virtual Holonomic Control Input
        float u_x = target.vx + KP_POS * errX;
        float u_y = target.vy + KP_POS * errY;

        // Invert Decoupling Matrix J^-1
        float v_cmd     = cosTh * u_x + sinTh * u_y;
        float omega_cmd = (-sinTh * u_x + cosTh * u_y) / LP_OFFSET_M;

        // Kinematic Envelope Saturation
        if (v_cmd > Config::SWARM_MAX_VEL_M_S)  v_cmd = Config::SWARM_MAX_VEL_M_S;
        if (v_cmd < -Config::SWARM_MAX_VEL_M_S) v_cmd = -Config::SWARM_MAX_VEL_M_S;

        constexpr float MAX_OMEGA = 2.0f; // 2 rad/s max turning
        if (omega_cmd > MAX_OMEGA)  omega_cmd = MAX_OMEGA;
        if (omega_cmd < -MAX_OMEGA) omega_cmd = -MAX_OMEGA;

        cmd.vLinear   = v_cmd;
        cmd.omegaRadS = omega_cmd;
        return cmd;
    }
};
