#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

    // ==============================================================================
    // 1. FLEET HOMOGENEOUS SPECIFICATIONS
    // ==============================================================================
    constexpr uint8_t FLEET_SIZE             = 4;
    constexpr float   GEAR_RATIO             = 120.0f;   // 1:120 homogeneous on all robots
    constexpr float   WHEEL_DIAMETER_M       = 0.0500f;  // 50 mm diameter on all robots
    constexpr float   WHEEL_RADIUS_M         = WHEEL_DIAMETER_M / 2.0f; // 25 mm radius
    constexpr float   WHEEL_WIDTH_M          = 0.0280f;  // 28 mm wheel width
    constexpr float   ENCODER_CPR            = 4096.0f;  // 12-bit AS5600 resolution

    // Universal Sensor Polarities (Left is mirror-inverted on all chassis)
    constexpr bool INVERT_ENCODER_LEFT       = true;
    constexpr bool INVERT_ENCODER_RIGHT      = false;

    // ==============================================================================
    // 2. VARIANT-SPECIFIC KINEMATICS, POLARITIES & IDENTIFIED STATIC IMU BIASES
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID                     = 1;
    constexpr float   TRACK_WIDTH_M          = 0.1350f;

    // Verified Motor Polarities (Forward Command = Forward Wheel Spin)
    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    // Identified Static IMU Biases
    constexpr float   GYRO_BIAS_X_DPS        = -0.3617f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.6928f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.1766f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.003082f;

    constexpr float   ACCEL_BIAS_X_MPS2      = -1.5146f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.7423f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = -0.1001f;

    // EKF Process Noise (100 Hz Discrete: Q_discrete = continuous * dt)
    constexpr float   Q_GYRO_Z_DISCRETE      = 6.30e-9f;
    constexpr float   Q_ACCEL_DISCRETE       = 1.58e-6f;

    // Geometric Lever-Arm Offset [Lateral (Y), Longitudinal (X)]
    constexpr float   IMU_OFFSET_LAT_M       = 0.0275f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0300f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID                     = 2;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   GYRO_BIAS_X_DPS        = -0.7720f;
    constexpr float   GYRO_BIAS_Y_DPS        = +0.4430f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.4421f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.007716f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.0279f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -1.4159f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +0.2139f;

    constexpr float   Q_GYRO_Z_DISCRETE      = 6.59e-9f;
    constexpr float   Q_ACCEL_DISCRETE       = 1.37e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID                     = 3;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   GYRO_BIAS_X_DPS        = -0.3567f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.3729f;
    constexpr float   GYRO_BIAS_Z_DPS        = -0.0574f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = -0.001002f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.7860f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.6439f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = -0.0842f;

    constexpr float   Q_GYRO_Z_DISCRETE      = 6.71e-9f;
    constexpr float   Q_ACCEL_DISCRETE       = 1.83e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID                     = 4;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = false;

    constexpr float   GYRO_BIAS_X_DPS        = -0.1860f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.3499f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.2565f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.004477f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.8025f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = +0.0297f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +0.1620f;

    constexpr float   Q_GYRO_Z_DISCRETE      = 6.92e-9f;
    constexpr float   Q_ACCEL_DISCRETE       = 1.33e-6f;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;
#endif

    // ==============================================================================
    // 3. IDENTIFIED UWB PAIRWISE CALIBRATION MATRIX (3.000m BENCHMARK)
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
    // 4. NETWORKING PARAMETERS
    // ==============================================================================
    constexpr const char* WIFI_SSID          = "Oochoo";
    constexpr const char* WIFI_PASSWORD      = "ax200ax200";
    constexpr uint16_t    NETCAT_PORT        = 9000;

    const IPAddress STATIC_IP(192, 168, 1, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 1, 1);
    const IPAddress SUBNET(255, 255, 255, 0);

} // namespace Config
