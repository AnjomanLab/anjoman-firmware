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

    // Universal Encoder Polarities (Left is mirror-inverted on all chassis)
    constexpr bool INVERT_ENCODER_LEFT       = true;
    constexpr bool INVERT_ENCODER_RIGHT      = false;

    // ==============================================================================
    // 2. VARIANT-SPECIFIC PARAMETERS & MOTOR POLARITIES
    // ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t ID                     = 1;
    constexpr float   TRACK_WIDTH_M          = 0.1350f;  // 135 mm track width

    // Verified Motor Polarities (Forward Command = Forward Wheel Spin)
    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    // IMU Offset [Lateral (Y), Longitudinal (X)] in meters
    constexpr float   IMU_OFFSET_LAT_M       = 0.0275f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0300f;

#elif ROBOT_ID == 2
    constexpr uint8_t ID                     = 2;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;  // 125 mm track width

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 3
    constexpr uint8_t ID                     = 3;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = false;
    constexpr bool    INVERT_MOTOR_RIGHT     = true;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#elif ROBOT_ID == 4
    constexpr uint8_t ID                     = 4;
    constexpr float   TRACK_WIDTH_M          = 0.1250f;

    constexpr bool    INVERT_MOTOR_LEFT      = true;
    constexpr bool    INVERT_MOTOR_RIGHT     = false;

    constexpr float   IMU_OFFSET_LAT_M       = 0.0000f;
    constexpr float   IMU_OFFSET_LON_M       = 0.0550f;

#else
    #error "Invalid ROBOT_ID defined. Must be 1, 2, 3, or 4."
#endif

    // ==============================================================================
    // 3. BASE NETWORK TELEMETRY PARAMETERS
    // ==============================================================================
    constexpr const char* WIFI_SSID          = "Oochoo";
    constexpr const char* WIFI_PASSWORD      = "ax200ax200";
    constexpr uint16_t    NETCAT_PORT        = 9000;

    const IPAddress STATIC_IP(192, 168, 1, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 1, 1);
    const IPAddress SUBNET(255, 255, 255, 0);

} // namespace Config
