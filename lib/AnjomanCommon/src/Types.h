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

struct UWBBeaconEntry {
    uint8_t peerId;
    uint8_t ldeErr;
    float   rawDist;
    float   cleanDist;
    float   rssi;
    float   fpPower;
    float   respTemp;
    float   cfoPpm;
};

struct SyncBeaconPacket {
    uint8_t  senderId;
    uint8_t  senderSlot;
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint32_t maneuverStartFrame;

    uint8_t        nPeers;
    UWBBeaconEntry peers[2];

    float    vbat;
    float    senderTempUwb;
    float    posX;
    float    posY;
    float    headingRad;
};

struct UWBPollPacket {
    char     header[4];
    uint8_t  initiatorId;
    uint8_t  targetId;
    uint32_t sequence;
};

struct UWBResponsePacket {
    char     header[4];
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
