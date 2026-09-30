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

// One UWB measurement carried in a sync beacon
struct UWBBeaconEntry {
    uint8_t peerId;
    uint8_t ldeErr;
    float   rawDist;
    float   cleanDist;
    float   rssi;
    float   fpPower;
    float   respTemp;
};  // 22 bytes

// Sync beacon — broadcast by each robot in its own slot
struct SyncBeaconPacket {
    uint8_t  senderId;
    uint8_t  senderSlot;
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint32_t maneuverStartFrame;

    uint8_t       nPeers;          // number of valid UWB entries
    UWBBeaconEntry peers[2];       // up to 2 peers per robot (3-robot fleet)

    float    vbat;
};

struct UWBPollPacket {
    char     header[4];            // "POLL"
    uint8_t  initiatorId;
    uint8_t  targetId;
    uint32_t sequence;
};

struct UWBResponsePacket {
    char     header[4];            // "RESP"
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
