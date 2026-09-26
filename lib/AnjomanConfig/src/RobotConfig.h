#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

    // ==============================================================================
    // 1. FLEET-WIDE PHYSICAL SPECIFICATIONS
    // ==============================================================================
    constexpr uint8_t  FLEET_SIZE            = 4;
    constexpr float    GEAR_RATIO            = 120.0f;
    constexpr float    WHEEL_DIAMETER_M      = 0.0537f;
    constexpr float    WHEEL_RADIUS_M        = WHEEL_DIAMETER_M / 2.0f;
    constexpr float    WHEEL_WIDTH_M         = 0.0280f;
    constexpr float    ENCODER_CPR           = 4096.0f;
    constexpr float    SHUNT_RESISTOR_OHM    = 0.010f;

    constexpr bool INVERT_ENCODER_LEFT       = true;
    constexpr bool INVERT_ENCODER_RIGHT      = false;

    // ==============================================================================
    // 2. CONTROL FREQUENCIES
    // ==============================================================================
    constexpr uint32_t CONTROL_RATE_HZ       = 100;
    constexpr uint32_t CONTROL_PERIOD_MS     = 1000 / CONTROL_RATE_HZ;
    constexpr float    CONTROL_PERIOD_S      = 0.010f;

    constexpr uint32_t TELEMETRY_RATE_HZ     = 5;
    constexpr uint32_t TELEMETRY_PERIOD_MS   = 1000 / TELEMETRY_RATE_HZ;

    // ==============================================================================
    // 3. SWARM KINEMATIC ENVELOPES
    // ==============================================================================
    constexpr float SWARM_CRUISE_VEL_M_S     = 0.15f;
    constexpr float SWARM_MAX_VEL_M_S        = 0.21f;

    // ==============================================================================
    // 4. PER-ROBOT VARIANT-SPECIFIC PARAMETERS
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID                     = 1;
    constexpr float   TRACK_WIDTH_NOM_M      = 0.1350f;
    constexpr float   TRACK_WIDTH_EFF_M      = 0.1237f;
    constexpr float   TRACK_WIDTH_M          = TRACK_WIDTH_EFF_M;
    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   DEADBAND_FWD_L         = 0.4560f;
    constexpr float   DEADBAND_FWD_R         = 0.4390f;
    constexpr float   DEADBAND_REV_L         = 0.4368f;
    constexpr float   DEADBAND_REV_R         = 0.4364f;
    constexpr float   GAIN_RPM_FWD_L         = 198.74f;
    constexpr float   GAIN_RPM_FWD_R         = 208.25f;
    constexpr float   GAIN_RPM_REV_L         = 181.79f;
    constexpr float   GAIN_RPM_REV_R         = 207.02f;
    constexpr float   ACTUATOR_TAU_S         = 0.1920f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.355f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.691f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.179f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.003124f;

    constexpr float   ACCEL_BIAS_X_MPS2      = -0.902f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.373f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +9.655f;

    constexpr float   Q_YAW_DISCRETE         = 6.30e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.45e-6f;

    constexpr float   IMU_OFFSET_LON_M       = 0.0358f;
    constexpr float   IMU_OFFSET_LAT_M       = -0.0284f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID                     = 2;
    constexpr float   TRACK_WIDTH_NOM_M      = 0.1250f;
    constexpr float   TRACK_WIDTH_EFF_M      = 0.1248f;
    constexpr float   TRACK_WIDTH_M          = TRACK_WIDTH_EFF_M;
    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   DEADBAND_FWD_L         = 0.3558f;
    constexpr float   DEADBAND_FWD_R         = 0.3099f;
    constexpr float   DEADBAND_REV_L         = 0.3703f;
    constexpr float   DEADBAND_REV_R         = 0.3523f;
    constexpr float   GAIN_RPM_FWD_L         = 182.89f;
    constexpr float   GAIN_RPM_FWD_R         = 162.03f;
    constexpr float   GAIN_RPM_REV_L         = 191.08f;
    constexpr float   GAIN_RPM_REV_R         = 177.86f;
    constexpr float   ACTUATOR_TAU_S         = 0.2390f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.770f;
    constexpr float   GYRO_BIAS_Y_DPS        = +0.447f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.439f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.007662f;

    constexpr float   ACCEL_BIAS_X_MPS2      = -0.124f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.670f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +9.993f;

    constexpr float   Q_YAW_DISCRETE         = 6.59e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LON_M       = 0.0516f;
    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID                     = 3;
    constexpr float   TRACK_WIDTH_NOM_M      = 0.1250f;
    constexpr float   TRACK_WIDTH_EFF_M      = 0.1163f;
    constexpr float   TRACK_WIDTH_M          = TRACK_WIDTH_EFF_M;
    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   DEADBAND_FWD_L         = 0.3468f;
    constexpr float   DEADBAND_FWD_R         = 0.3871f;
    constexpr float   DEADBAND_REV_L         = 0.3333f;
    constexpr float   DEADBAND_REV_R         = 0.3754f;
    constexpr float   GAIN_RPM_FWD_L         = 168.66f;
    constexpr float   GAIN_RPM_FWD_R         = 172.92f;
    constexpr float   GAIN_RPM_REV_L         = 167.05f;
    constexpr float   GAIN_RPM_REV_R         = 162.64f;
    constexpr float   ACTUATOR_TAU_S         = 0.2350f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.341f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.370f;
    constexpr float   GYRO_BIAS_Z_DPS        = -0.058f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = -0.001012f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.270f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = -0.292f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +9.710f;

    constexpr float   Q_YAW_DISCRETE         = 6.71e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LON_M       = 0.0590f;
    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID                     = 4;
    constexpr float   TRACK_WIDTH_NOM_M      = 0.1250f;
    constexpr float   TRACK_WIDTH_EFF_M      = 0.1209f;
    constexpr float   TRACK_WIDTH_M          = TRACK_WIDTH_EFF_M;
    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = false;

    constexpr float   DEADBAND_FWD_L         = 0.3243f;
    constexpr float   DEADBAND_FWD_R         = 0.3513f;
    constexpr float   DEADBAND_REV_L         = 0.3197f;
    constexpr float   DEADBAND_REV_R         = 0.3432f;
    constexpr float   GAIN_RPM_FWD_L         = 186.47f;
    constexpr float   GAIN_RPM_FWD_R         = 196.83f;
    constexpr float   GAIN_RPM_REV_L         = 183.89f;
    constexpr float   GAIN_RPM_REV_R         = 187.03f;
    constexpr float   ACTUATOR_TAU_S         = 0.2410f;

    constexpr float   GYRO_BIAS_X_DPS        = -0.167f;
    constexpr float   GYRO_BIAS_Y_DPS        = -0.344f;
    constexpr float   GYRO_BIAS_Z_DPS        = +0.256f;
    constexpr float   GYRO_BIAS_Z_RAD_S      = +0.004468f;

    constexpr float   ACCEL_BIAS_X_MPS2      = +0.280f;
    constexpr float   ACCEL_BIAS_Y_MPS2      = +0.089f;
    constexpr float   ACCEL_BIAS_Z_MPS2      = +9.963f;

    constexpr float   Q_YAW_DISCRETE         = 6.92e-9f;
    constexpr float   Q_GYRO_BIAS_WALK       = 1.00e-12f;
    constexpr float   R_YAW_ENCODER          = 1.68e-6f;

    constexpr float   IMU_OFFSET_LON_M       = 0.0578f;
    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;

#endif

    // ==============================================================================
    // 5. NETWORK / TELEMETRY
    // ==============================================================================
    constexpr const char* WIFI_SSID          = "H11T";
    constexpr const char* WIFI_PASSWORD      = "123456788";
    constexpr uint16_t    NETCAT_PORT        = 9000;

    const IPAddress STATIC_IP(192, 168, 242, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 242, 164);
    const IPAddress SUBNET(255, 255, 255, 0);

} // namespace Config
