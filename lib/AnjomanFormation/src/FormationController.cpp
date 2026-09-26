#include "FormationController.h"
#include "RobotConfig.h"
#include <cmath>

WheelVelocityCommand FormationController::compute(
    uint8_t robotId,
    const FormationState2D &target,
    float currentX,
    float currentY,
    float currentThetaRad
) {
    (void)robotId;   // reserved for future per-robot tuning

    WheelVelocityCommand cmd = {};

    float cosTh = cosf(currentThetaRad);
    float sinTh = sinf(currentThetaRad);

    // Position error between target and robot axle center
    float errX = target.x - currentX;
    float errY = target.y - currentY;

    // Virtual holonomic control input (world frame)
    float u_x = target.vx + KP_POS * errX;
    float u_y = target.vy + KP_POS * errY;

    // Inverse decoupling: transform world-frame virtual input to body frame
    // v_cmd     = cos(th)*u_x + sin(th)*u_y
    // omega_cmd = (-sin(th)*u_x + cos(th)*u_y) / Lp
    float v_cmd     = cosTh * u_x + sinTh * u_y;
    float omega_cmd = (-sinTh * u_x + cosTh * u_y) / LP_OFFSET_M;

    // ---- Kinematic envelope saturation ----
    if (v_cmd >  Config::SWARM_MAX_VEL_M_S) v_cmd =  Config::SWARM_MAX_VEL_M_S;
    if (v_cmd < -Config::SWARM_MAX_VEL_M_S) v_cmd = -Config::SWARM_MAX_VEL_M_S;

    if (omega_cmd >  MAX_OMEGA) omega_cmd =  MAX_OMEGA;
    if (omega_cmd < -MAX_OMEGA) omega_cmd = -MAX_OMEGA;

    cmd.vLinear   = v_cmd;
    cmd.omegaRadS = omega_cmd;
    return cmd;
}
