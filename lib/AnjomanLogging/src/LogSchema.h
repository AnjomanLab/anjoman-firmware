#pragma once

#include <Arduino.h>

// ==============================================================================
// Binary log format for Anjoman maneuver recordings — Version 2
// ==============================================================================
// Layout:
//   [LogHeader (14 B)] [LogRecord × N (184 B)] [LogFooter (8 B)]
//
// Version history:
//   1 — Hub-and-spoke, single UWB peer per record (68 B)
//   2 — Full mesh, three UWB peers per record (184 B)
//
// Endianness: little-endian on both writer (ESP32) and reader (Python)
// ==============================================================================

#pragma pack(push, 1)

struct LogHeader {
    char     magic[4];       // "ANJM"
    uint16_t version;        // 2
    uint8_t  robotId;        // Config::ID
    uint8_t  maneuverId;     // 1, 2, 3, ...
    uint32_t recordCount;    // N
    uint16_t recordSize;     // sizeof(LogRecord)
};

// ------------------------------------------------------------------------------
// One UWB peer block (three of these per record, one per other robot)
// ------------------------------------------------------------------------------
struct UWBPeerBlock {
    uint8_t  peerId;         // 1..4
    uint8_t  ldeErr;         // LDE error flag (0/1)
    uint16_t stdNoise;       // noise floor (EMI metric)
    uint16_t fpAmpl1;        // first-path amplitude 1
    uint16_t fpAmpl2;        // first-path amplitude 2
    uint16_t cirPwr;         // channel impulse response power
    float    rawDist;        // raw distance from this poll
    float    cleanDist;      // bias + thermal corrected distance
    float    rssi;           // received signal strength (dBm)
    float    fpPower;        // first-path power (dBm)
    float    respTemp;       // responder chip temperature (°C)
};  // 30 bytes

struct LogRecord {
    uint32_t t_ms;

    // ---- ESKF pose estimate ----
    float    x;
    float    y;
    float    heading;

    // ---- Commanded chassis velocities ----
    float    v_cmd;
    float    omega_cmd;

    // ---- Measured wheel speeds ----
    float    rpm_l;
    float    rpm_r;

    // ---- IMU ----
    float    gyro_z;
    float    imu_temp_c;
    float    imu_accel_x;

    // ---- Power ----
    float    vbat;
    float    current_a;

    // ---- Motor detail ----
    float    duty_l;
    float    duty_r;
    float    target_rpm_l;
    float    target_rpm_r;

    // ---- UWB: three peers ----
    UWBPeerBlock uwb[3];
    uint16_t     rxpacc;        // shared RX preamble accumulator count

    // ---- ESKF covariance (diagonal) ----
    float    eskf_var_x;
    float    eskf_var_y;
    float    eskf_var_theta;
    float    eskf_var_bias;

    // ---- TDMA state ----
    uint32_t tdma_frame_id;
    uint8_t  tdma_slot_index;
    uint8_t  tdma_sync_lost;
    uint8_t  maneuver_id;
    uint8_t  robot_state;       // 0=idle, 1=running, 2=finished
};

struct LogFooter {
    uint32_t crc32;
    char     endMagic[4];       // "MJNA"
};

#pragma pack(pop)

static_assert(sizeof(UWBPeerBlock) == 30,
              "UWBPeerBlock size mismatch");
static_assert(sizeof(LogRecord) == 184,
              "LogRecord size mismatch — check struct packing");
