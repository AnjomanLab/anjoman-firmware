#pragma once

#include <Arduino.h>

#pragma pack(push, 1)

// Clean real-time telemetry packet transmitted over ESP-NOW
struct TelemetryPacket {
    uint8_t  senderId;
    uint32_t timestampMs;
    uint32_t sequenceId;
    float    measuredDistanceM;
    float    signalRssi;
};

// Sync beacon for TDMA timing
struct SyncBeaconPacket {
    uint32_t frameId;
    uint32_t timestampMs;
    uint8_t  maneuverActive;
    uint32_t maneuverStartMs;
};

// UWB poll packet
struct UWBPollPacket {
    char     header[4];     // "POLL"
    uint8_t  initiatorId;
    uint8_t  targetId;
    uint32_t sequence;
};

// UWB response packet
struct UWBResponsePacket {
    char     header[4];     // "RESP"
    uint8_t  responderId;
    uint8_t  targetId;
    uint32_t sequence;
    uint8_t  rxTimestamp[5];   // tRx2 (40-bit)
    uint8_t  txTimestamp[5];   // tTx2 (40-bit)
    float    tempUwb;
    float    tempEsp;
    float    vbatUwb;
};

#pragma pack(pop)
