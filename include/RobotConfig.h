#pragma once

#include <Arduino.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

    // ==============================================================================
    // 1. FLEET IDENTIFIED PARAMETERS (MOTORS, KINEMATICS, IMU & UWB)
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID = 1;
    constexpr float GEAR_RATIO           = 48.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0500f;
    constexpr float TRACK_WIDTH_M        = 0.1350f;

    // Motor Inversions & Deadbands
    constexpr bool  INVERT_MOTOR_LEFT    = false;
    constexpr bool  INVERT_MOTOR_RIGHT   = true;
    constexpr float DEADBAND_PWM_LEFT    = 0.60f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.60f;
    constexpr float MOTOR_GAIN_L_RPM     = 596.7f;
    constexpr float MOTOR_GAIN_R_RPM     = 545.0f;

    // Static IMU Biases
    constexpr float GYRO_BIAS_X_DPS      = -0.3567f;
    constexpr float GYRO_BIAS_Y_DPS      = -0.6997f;
    constexpr float GYRO_BIAS_Z_DPS      = +0.1714f;
    constexpr float GYRO_BIAS_Z_RAD_S    = +0.002991f;

    constexpr float ACCEL_BIAS_X_MPS2    = -0.7481f;
    constexpr float ACCEL_BIAS_Y_MPS2    = -0.4747f;
    constexpr float ACCEL_BIAS_Z_MPS2    = -0.1125f;

    // EKF Process Noise Variances (100 Hz discrete: Q = continuous * dt)
    constexpr float VAR_GYRO_Z_RAD2_S    = 9.57e-9f;
    constexpr float VAR_ACCEL_MPS2       = 1.75e-6f;

    // Lever-Arm Offset [Lateral (Y), Longitudinal (X)]
    constexpr float IMU_OFFSET_LAT_M     = 0.0275f;
    constexpr float IMU_OFFSET_LON_M     = 0.0300f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID = 2;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool  INVERT_MOTOR_LEFT    = true;
    constexpr bool  INVERT_MOTOR_RIGHT   = true;
    constexpr float DEADBAND_PWM_LEFT    = 0.60f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.60f;
    constexpr float MOTOR_GAIN_L_RPM     = 164.8f;
    constexpr float MOTOR_GAIN_R_RPM     = 161.0f;

    constexpr float GYRO_BIAS_X_DPS      = -0.7773f;
    constexpr float GYRO_BIAS_Y_DPS      = +0.4408f;
    constexpr float GYRO_BIAS_Z_DPS      = +0.4341f;
    constexpr float GYRO_BIAS_Z_RAD_S    = +0.007576f;

    constexpr float ACCEL_BIAS_X_MPS2    = -0.7566f;
    constexpr float ACCEL_BIAS_Y_MPS2    = -1.4469f;
    constexpr float ACCEL_BIAS_Z_MPS2    = +0.2202f;

    constexpr float VAR_GYRO_Z_RAD2_S    = 6.39e-9f;
    constexpr float VAR_ACCEL_MPS2       = 1.42e-6f;

    constexpr float IMU_OFFSET_LAT_M     = 0.0000f;
    constexpr float IMU_OFFSET_LON_M     = 0.0550f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID = 3;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool  INVERT_MOTOR_LEFT    = false;
    constexpr bool  INVERT_MOTOR_RIGHT   = true;
    constexpr float DEADBAND_PWM_LEFT    = 0.40f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.40f;
    constexpr float MOTOR_GAIN_L_RPM     = 120.4f;
    constexpr float MOTOR_GAIN_R_RPM     = 139.4f;

    constexpr float GYRO_BIAS_X_DPS      = -0.3552f;
    constexpr float GYRO_BIAS_Y_DPS      = -0.3795f;
    constexpr float GYRO_BIAS_Z_DPS      = -0.0559f;
    constexpr float GYRO_BIAS_Z_RAD_S    = -0.000976f;

    constexpr float ACCEL_BIAS_X_MPS2    = +0.9135f;
    constexpr float ACCEL_BIAS_Y_MPS2    = -0.7392f;
    constexpr float ACCEL_BIAS_Z_MPS2    = -0.0782f;

    constexpr float VAR_GYRO_Z_RAD2_S    = 6.75e-9f;
    constexpr float VAR_ACCEL_MPS2       = 1.97e-6f;

    constexpr float IMU_OFFSET_LAT_M     = 0.0000f;
    constexpr float IMU_OFFSET_LON_M     = 0.0550f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID = 4;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool  INVERT_MOTOR_LEFT    = true;
    constexpr bool  INVERT_MOTOR_RIGHT   = false;
    constexpr float DEADBAND_PWM_LEFT    = 0.40f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.50f;
    constexpr float MOTOR_GAIN_L_RPM     = 144.5f;
    constexpr float MOTOR_GAIN_R_RPM     = 173.2f;

    constexpr float GYRO_BIAS_X_DPS      = -0.1826f;
    constexpr float GYRO_BIAS_Y_DPS      = -0.3406f;
    constexpr float GYRO_BIAS_Z_DPS      = +0.2633f;
    constexpr float GYRO_BIAS_Z_RAD_S    = +0.004595f;

    constexpr float ACCEL_BIAS_X_MPS2    = +0.7080f;
    constexpr float ACCEL_BIAS_Y_MPS2    = +0.0169f;
    constexpr float ACCEL_BIAS_Z_MPS2    = +0.1725f;

    constexpr float VAR_GYRO_Z_RAD2_S    = 7.14e-9f;
    constexpr float VAR_ACCEL_MPS2       = 1.40e-6f;

    constexpr float IMU_OFFSET_LAT_M     = 0.0000f;
    constexpr float IMU_OFFSET_LON_M     = 0.0550f;
#endif

    // Universal Sensor Rules
    constexpr bool  INVERT_ENCODER_LEFT  = true;
    constexpr bool  INVERT_ENCODER_RIGHT = false;

    // Swarm-Wide Cruising Ceiling
    constexpr float SWARM_MAX_CRUISE_VEL_M_S = 0.19f;

    // Control Frequencies
    constexpr float WHEEL_RADIUS_M       = WHEEL_DIAMETER_M / 2.0f;
    constexpr uint32_t CONTROL_RATE_HZ   = 100;
    constexpr uint32_t CONTROL_PERIOD_MS = 1000 / CONTROL_RATE_HZ;

    // Pairwise UWB Calibration Matrix (4x4)
    constexpr float UWB_PAIR_OFFSETS[4][4] = {
        {   0.0000f,   21.9198f,   23.1712f,   21.5458f },
        {  21.9198f,    0.0000f,   31.4960f,   40.8656f },
        {  23.1712f,   31.4960f,    0.0000f,   37.3417f },
        {  21.5458f,   40.8656f,   37.3417f,    0.0000f }
    };

    inline float getCalibratedDistance(uint8_t myId, uint8_t peerId, float rawDist) {
        if (myId < 1 || myId > 4 || peerId < 1 || peerId > 4 || myId == peerId) return rawDist;
        return rawDist - UWB_PAIR_OFFSETS[myId - 1][peerId - 1];
    }

} // namespace Config
