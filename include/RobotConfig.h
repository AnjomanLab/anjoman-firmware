#pragma once

#include <Arduino.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

    // ==============================================================================
    // 1. FLEET IDENTIFIED ACTUATOR & KINEMATICS PARAMETERS
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID = 1;
    constexpr float GEAR_RATIO           = 48.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0500f;
    constexpr float TRACK_WIDTH_M        = 0.1350f;

    // Motor Inversion Polarities (Forward = Positive)
    constexpr bool INVERT_MOTOR_LEFT     = false;
    constexpr bool INVERT_MOTOR_RIGHT    = true;

    // Identified Deadband Thresholds (PWM 0 to 1.0)
    constexpr float DEADBAND_PWM_LEFT    = 0.60f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.60f;

    // Actuator Gains & Time Constants
    constexpr float MOTOR_GAIN_LEFT_RPM  = 596.7f;
    constexpr float MOTOR_GAIN_RIGHT_RPM = 545.0f;
    constexpr float MOTOR_TAU_LEFT_S     = 0.141f;
    constexpr float MOTOR_TAU_RIGHT_S    = 0.133f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID = 2;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool INVERT_MOTOR_LEFT     = true;
    constexpr bool INVERT_MOTOR_RIGHT    = true;

    constexpr float DEADBAND_PWM_LEFT    = 0.60f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.60f;

    constexpr float MOTOR_GAIN_LEFT_RPM  = 164.8f;
    constexpr float MOTOR_GAIN_RIGHT_RPM = 161.0f;
    constexpr float MOTOR_TAU_LEFT_S     = 0.151f;
    constexpr float MOTOR_TAU_RIGHT_S    = 0.172f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID = 3;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool INVERT_MOTOR_LEFT     = false;
    constexpr bool INVERT_MOTOR_RIGHT    = true;

    constexpr float DEADBAND_PWM_LEFT    = 0.40f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.40f;

    constexpr float MOTOR_GAIN_LEFT_RPM  = 120.4f;
    constexpr float MOTOR_GAIN_RIGHT_RPM = 139.4f;
    constexpr float MOTOR_TAU_LEFT_S     = 0.226f;
    constexpr float MOTOR_TAU_RIGHT_S    = 0.163f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID = 4;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;

    constexpr bool INVERT_MOTOR_LEFT     = true;
    constexpr bool INVERT_MOTOR_RIGHT    = false;

    constexpr float DEADBAND_PWM_LEFT    = 0.40f;
    constexpr float DEADBAND_PWM_RIGHT   = 0.50f;

    constexpr float MOTOR_GAIN_LEFT_RPM  = 144.5f;
    constexpr float MOTOR_GAIN_RIGHT_RPM = 173.2f;
    constexpr float MOTOR_TAU_LEFT_S     = 0.199f;
    constexpr float MOTOR_TAU_RIGHT_S    = 0.165f;
#endif

    // Universal Encoder Polarities (Mirror-Inversion Corrected)
    constexpr bool INVERT_ENCODER_LEFT   = true;  // Universal on ALL robots
    constexpr bool INVERT_ENCODER_RIGHT  = false; // Universal on ALL robots

    // Swarm-Wide Synchronous Cruising Velocity Ceiling (Governed by slowest robot)
    constexpr float SWARM_MAX_CRUISE_VEL_M_S = 0.19f; // 19 cm/s (30% PID Headroom)

    constexpr float WHEEL_RADIUS_M       = WHEEL_DIAMETER_M / 2.0f;
    constexpr uint32_t CONTROL_RATE_HZ   = 100;
    constexpr uint32_t CONTROL_PERIOD_MS = 1000 / CONTROL_RATE_HZ;

} // namespace Config
