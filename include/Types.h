#pragma once

#include <Arduino.h>

#pragma pack(push, 1)

// Clean real-time telemetry packet transmitted over ESP-NOW
struct TelemetryPacket {
    uint8_t  senderId;
    uint32_t timestampMs;
    uint32_t sequenceId;
    
    // Measured physical distance via UWB (in meters, -1.0 if not yet measured)
    float measuredDistanceM;
    
    // Received signal strength indicator (dBm)
    float signalRssi;
};

#pragma pack(pop)
