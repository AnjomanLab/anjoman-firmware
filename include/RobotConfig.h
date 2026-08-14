#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

#if ROBOT_ID == 1
    // Robot 1 Physical Configuration
    constexpr uint8_t ID = 1;
    constexpr float GEAR_RATIO           = 48.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0500f;
    constexpr float WHEEL_RADIUS_M       = WHEEL_DIAMETER_M / 2.0f;
    constexpr float TRACK_WIDTH_M        = 0.1350f;
    
    // IMU Offset from Axle Center [Lateral (Y), Longitudinal (X)] in meters
    constexpr float IMU_OFFSET_LAT_M     = 0.0275f;
    constexpr float IMU_OFFSET_LON_M     = 0.0300f;

#elif ROBOT_ID == 2 || ROBOT_ID == 3 || ROBOT_ID == 4
    // Robots 2, 3, 4 Physical Configuration
    constexpr uint8_t ID = ROBOT_ID;
    constexpr float GEAR_RATIO           = 120.0f;
    constexpr float WHEEL_DIAMETER_M     = 0.0550f;
    constexpr float WHEEL_RADIUS_M       = WHEEL_DIAMETER_M / 2.0f;
    constexpr float TRACK_WIDTH_M        = 0.1250f;
    
    // IMU Offset from Axle Center [Lateral (Y), Longitudinal (X)] in meters
    constexpr float IMU_OFFSET_LAT_M     = 0.0000f;
    constexpr float IMU_OFFSET_LON_M     = 0.0550f;

#else
    #error "Invalid ROBOT_ID defined. Must be 1, 2, 3, or 4."
#endif

    // Fleet Configuration
    constexpr uint8_t FLEET_SIZE         = 4;

    // WiFi & Netcat Telemetry Configuration
    constexpr const char* WIFI_SSID      = "Oochoo";
    constexpr const char* WIFI_PASSWORD  = "ax200ax200";
    constexpr uint16_t NETCAT_PORT       = 9000;

    const IPAddress STATIC_IP(192, 168, 1, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 1, 1);
    const IPAddress SUBNET(255, 255, 255, 0);

    // Default Rates
    constexpr uint32_t TELEMETRY_RATE_HZ   = 20;
    constexpr uint32_t TELEMETRY_PERIOD_MS = 1000 / TELEMETRY_RATE_HZ;

    constexpr uint32_t CONTROL_RATE_HZ     = 100;
    constexpr uint32_t CONTROL_PERIOD_MS   = 1000 / CONTROL_RATE_HZ;

} // namespace Config
