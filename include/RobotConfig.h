#pragma once

#include <Arduino.h>
#include <IPAddress.h>

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

namespace Config {

#if ROBOT_ID == 1
    constexpr uint8_t ID = 1;
    constexpr float GEAR_RATIO       = 48.0f;
    constexpr float WHEEL_DIAMETER_M = 0.0500f;
    constexpr float TRACK_WIDTH_M    = 0.1350f;
#elif ROBOT_ID == 2 || ROBOT_ID == 3 || ROBOT_ID == 4
    constexpr uint8_t ID = ROBOT_ID;
    constexpr float GEAR_RATIO       = 120.0f;
    constexpr float WHEEL_DIAMETER_M = 0.0550f;
    constexpr float TRACK_WIDTH_M    = 0.1250f;
#else
    #error "Invalid ROBOT_ID defined. Must be 1, 2, 3, or 4."
#endif

    // Fleet Networking Configuration
    constexpr const char* WIFI_SSID      = "Oochoo";
    constexpr const char* WIFI_PASSWORD  = "ax200ax200";
    constexpr uint16_t NETCAT_PORT       = 9000;
    const IPAddress STATIC_IP(192, 168, 1, 150 + (ID - 1));
    const IPAddress GATEWAY(192, 168, 1, 1);
    const IPAddress SUBNET(255, 255, 255, 0);

    // ==============================================================================
    // EXACT PAIRWISE CALIBRATION MATRIX (Empirically Identified from 60,400 samples)
    // ==============================================================================
    // Pairwise Hardware Offset Lookup Matrix (in Meters) for N=4 Swarm
    constexpr float UWB_PAIR_OFFSETS[4][4] = {
        //    R1           R2           R3           R4
        {   0.0000f,   21.9198f,   23.1712f,   21.5458f }, // R1
        {  21.9198f,    0.0000f,   31.4960f,   40.8656f }, // R2
        {  23.1712f,   31.4960f,    0.0000f,   37.3417f }, // R3
        {  21.5458f,   40.8656f,   37.3417f,    0.0000f }  // R4
    };

    // Helper to get calibrated distance between Robot i and Robot j
    inline float getCalibratedDistance(uint8_t myId, uint8_t peerId, float rawDist) {
        if (myId < 1 || myId > 4 || peerId < 1 || peerId > 4 || myId == peerId) return rawDist;
        return rawDist - UWB_PAIR_OFFSETS[myId - 1][peerId - 1];
    }

} // namespace Config
