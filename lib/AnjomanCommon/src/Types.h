#pragma once

#include <Arduino.h>

#pragma pack(push, 1)

struct TelemetryPacket {
    uint8_t  senderId;
    uint32_t timestampMs;
    uint32_t sequenceId;
    float    measuredDistanceM;
    float    signalRssi;
};

struct SyncBeaconPacket {
    uint8_t  senderId;              // 1..4
    uint8_t  senderSlot;            // 0, 4, 8, or 12
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint32_t maneuverStartFrame;    // consensus value (0 = not yet decided)
};

struct UWBPollPacket {
    char     header[4];             // "POLL"
    uint8_t  initiatorId;
    uint8_t  targetId;
    uint32_t sequence;
};

struct UWBResponsePacket {
    char     header[4];             // "RESP"
    uint8_t  responderId;
    uint8_t  targetId;
    uint32_t sequence;
    uint8_t  rxTimestamp[5];
    uint8_t  txTimestamp[5];
    float    tempUwb;
    float    tempEsp;
    float    vbatUwb;
};

#pragma pack(pop)
