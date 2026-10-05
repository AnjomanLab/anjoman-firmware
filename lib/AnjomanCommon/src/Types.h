#pragma once

#include <Arduino.h>

// ==============================================================================
// 1. OVER-THE-AIR UWB TWO-WAY RANGING PACKETS
// ==============================================================================

struct __attribute__((packed)) UWBPollPacket {
    uint8_t  initiatorId;
    uint8_t  targetId;
    uint32_t sequence;
};

struct __attribute__((packed)) UWBResponsePacket {
    uint8_t  responderId;
    uint8_t  targetId;
    uint8_t  rxTimestamp[5]; // 40-bit DW1000 arrival time
    uint8_t  txTimestamp[5]; // 40-bit DW1000 scheduled transmit time
    float    tempUwb;        // Responder live die temperature
};

// ==============================================================================
// 2. IN-MEMORY FILTER & ESTIMATION TELEMETRY METRICS
// ==============================================================================

struct UWBRangeMetric {
    bool     valid;
    uint8_t  peerId;
    uint8_t  ldeErr;
    uint16_t stdNoise;
    uint16_t fpAmpl1;
    uint16_t fpAmpl2;
    uint16_t cirPwr;
    uint16_t rxpacc;
    float    rawDist;
    float    cleanDist;
    float    rssi;
    float    fpPower;
    float    respTemp;
};

// Universal alias to guarantee zero type-name compilation mismatches
using UWBMetrics = UWBRangeMetric;

// ==============================================================================
// 3. OVER-THE-AIR ESP-NOW TELEMETRY & BEACON PACKETS
// ==============================================================================

struct __attribute__((packed)) UWBBeaconEntry {
    uint8_t  peerId;
    union {
        uint8_t ldeErr;
        uint8_t flags;
    };
    union {
        uint16_t rawDistMm;
        uint16_t distRawMm;
    };
    union {
        uint16_t cleanDistMm;
        uint16_t distanceMm;
        uint16_t distMm;
    };
    union {
        int8_t   rssi;
        int8_t   rssiDbm;
    };
    int8_t   fpPower;
    union {
        float    respTemp;
        float    tempUwb;
    };
    float    cfoPpm;
};

struct __attribute__((packed)) SyncBeaconPacket {
    uint8_t  senderId;
    uint8_t  senderSlot;
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint32_t maneuverStartFrame;
    union {
        uint8_t nPeers;
        uint8_t numRanges;
        uint8_t rangeCount;
    };
    float    vbat;
    float    senderTempUwb;
    float    posX;
    float    posY;
    float    headingRad;
    union {
        UWBBeaconEntry peers[2];
        UWBBeaconEntry ranges[2];
    };
};
