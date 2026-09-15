#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

    // ==============================================================================
    // 1. FLEET-WIDE HOMOGENEOUS PHYSICAL SPECIFICATIONS
    // ==============================================================================
    constexpr uint8_t FLEET_SIZE             = 4;
    constexpr float   GEAR_RATIO             = 120.0f;   // 1:120 homogeneous on all robots
    constexpr float   WHEEL_DIAMETER_M       = 0.0500f;  // 50 mm diameter on all robots
    constexpr float   WHEEL_RADIUS_M         = WHEEL_DIAMETER_M / 2.0f; // 25 mm radius
    constexpr float   WHEEL_WIDTH_M          = 0.0280f;  // 28 mm wheel width
    constexpr float   ENCODER_CPR            = 4096.0f;  // 12-bit AS5600 resolution
    constexpr float   SHUNT_RESISTOR_OHM     = 0.010f;   // 0.01 Ohm (R010) on all robots
    // Universal Sensor Polarities (Left is mirror-inverted on all chassis)
    constexpr bool INVERT_ENCODER_LEFT       = true;
    constexpr bool INVERT_ENCODER_RIGHT      = false;

    // Control Frequencies
    constexpr uint32_t CONTROL_RATE_HZ       = 100;
    constexpr uint32_t CONTROL_PERIOD_MS     = 1000 / CONTROL_RATE_HZ;
    constexpr float    CONTROL_PERIOD_S      = 0.010f; // 10 ms discrete time step (dt)

    // Swarm Kinematic Velocity Envelopes
    constexpr float SWARM_CRUISE_VEL_M_S     = 0.15f; // 15 cm/s safe cruising speed
    constexpr float SWARM_MAX_VEL_M_S        = 0.21f; // 21 cm/s saturation ceiling

    // ==============================================================================
    // 2. VARIANT-SPECIFIC KINEMATICS, POLARITIES & STATIC IMU BIASES
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID                     = 1;
    constexpr float   TRACK_WIDTH_M          = 0.1350f;  // 135 mm track width

    // Verified Motor Polarities (Forward Command = Forward Wheel Spin)
    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    // Identified Drivetrain Parameters
    constexpr float   DEADBAND_FWD_L         = 0.4535f;
    constexpr float   DEADBAND_FWD_R         = 0.4996f;
    constexpr float   DEADBAND_REV_L         = 0.4658f;
    constexpr float   DEADBAND_REV_R         = 0.4660f;
    constexpr float   GAIN_RPM_FWD_L         = 196.93f;
    constexpr float   GAIN_RPM_FWD_R         = 251.13f;
    constexpr float   GAIN_RPM_REV_L         = 194.76f;
    constexpr float   GAIN_RPM_REV_R         = 227.31f;
    constexpr float   ACTUATOR_TAU_S         = 0.2130f;

    // High-Confidence Static IMU Biases (from 3-minute 18,000-sample benchmark)
    constexpr float   GYRO_BIAS_X_DPS        = -0.3617f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.6928f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.1766f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.003082f;

    constexpr float   ACCEL_BIAS_X_MPS2      = -1.5146f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.7423f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = -0.1001f;

    // Heading Estimator Covariances (100 Hz Discrete Time)
    constexpr float   Q_YAW_DISCRETE         = 6.30e-9f;  // Process noise variance (rad^2)
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f; // Gyro bias drift variance (rad/s)^2
    constexpr float   R_YAW_ENCODER          = 1.45e-6f;  // Differential encoder measurement noise (rad^2)

    // Geometric Lever-Arm Offset [Lateral (Y), Longitudinal (X)]
    constexpr float   IMU_OFFSET_LAT_M       = 0.0275f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0300f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID                     = 2;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;  // 125 mm track width

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   DEADBAND_FWD_L         = 0.3719f;
    constexpr float   DEADBAND_FWD_R         = 0.3828f;
    constexpr float   DEADBAND_REV_L         = 0.4385f;
    constexpr float   DEADBAND_REV_R         = 0.3681f;
    constexpr float   GAIN_RPM_FWD_L         = 192.46f;
    constexpr float   GAIN_RPM_FWD_R         = 194.96f;
    constexpr float   GAIN_RPM_REV_L         = 229.81f;
    constexpr float   GAIN_RPM_REV_R         = 186.07f;
    constexpr float   ACTUATOR_TAU_S         = 0.1954f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.7720f;
    constexpr float   GYRO_BIAS_Y_DPS        = +0.4430f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.4421f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.007716f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.0279f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -1.4159f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +0.2139f;

    constexpr float   Q_YAW_DISCRETE         = 6.59e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID                     = 3;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   DEADBAND_FWD_L         = 0.4334f;
    constexpr float   DEADBAND_FWD_R         = 0.3899f;
    constexpr float   DEADBAND_REV_L         = 0.4316f;
    constexpr float   DEADBAND_REV_R         = 0.4422f;
    constexpr float   GAIN_RPM_FWD_L         = 213.21f;
    constexpr float   GAIN_RPM_FWD_R         = 174.18f;
    constexpr float   GAIN_RPM_REV_L         = 215.79f;
    constexpr float   GAIN_RPM_REV_R         = 195.42f;
    constexpr float   ACTUATOR_TAU_S         = 0.1967f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.3567f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.3729f;
    constexpr float   GYRO_BIAS_Z_DPS        = -0.0574f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = -0.001002f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.7860f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.6439f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = -0.0842f;

    constexpr float   Q_YAW_DISCRETE         = 6.71e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID                     = 4;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = false;

    constexpr float   DEADBAND_FWD_L         = 0.4284f;
    constexpr float   DEADBAND_FWD_R         = 0.3960f;
    constexpr float   DEADBAND_REV_L         = 0.4296f;
    constexpr float   DEADBAND_REV_R         = 0.3923f;
    constexpr float   GAIN_RPM_FWD_L         = 245.57f;
    constexpr float   GAIN_RPM_FWD_R         = 222.02f;
    constexpr float   GAIN_RPM_REV_L         = 243.65f;
    constexpr float   GAIN_RPM_REV_R         = 213.26f;
    constexpr float   ACTUATOR_TAU_S         = 0.2160f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.1860f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.3499f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.2565f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.004477f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.8025f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = +0.0297f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +0.1620f;

    constexpr float   Q_YAW_DISCRETE         = 6.92e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;
#endif

    // ==============================================================================
    // 3. UWB PAIRWISE OFFSETS (3.000m EMPIRICAL MATRIX)
    // ==============================================================================
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

    // ==============================================================================
    // 4. BASE NETWORK TELEMETRY PARAMETERS
    // ==============================================================================
    constexpr const char* WIFI_SSID          = "Oochoo";
    constexpr const char* WIFI_PASSWORD      = "ax200ax200";
    constexpr uint16_t    NETCAT_PORT        = 9000;

    const IPAddress STATIC_IP(192, 168, 1, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 1, 1);
    const IPAddress SUBNET(255, 255, 255, 0);

} // namespace Config
