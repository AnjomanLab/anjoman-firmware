#pragma once

#include <Arduino.h>

// ==============================================================================
// Binary log file format for Anjoman maneuver recordings
// ==============================================================================
// Layout:
//   [LogHeader] [LogRecord × N] [LogFooter]
//   Header: 14 bytes
//   Record: 68 bytes (see struct)
//   Footer: 8 bytes (CRC32 + end magic)
//
// Endianness: little-endian on both writer (ESP32) and reader (Python)
// ==============================================================================

#pragma pack(push, 1)

struct LogHeader {
    char     magic[4];       // "ANJM"
    uint16_t version;        // 1
    uint8_t  robotId;        // Config::ID
    uint8_t  maneuverId;     // 1, 2, 3, ...
    uint32_t recordCount;    // N
    uint16_t recordSize;     // sizeof(LogRecord)
};

struct LogRecord {
    uint32_t t_ms;           // Timestamp (ms since boot)

    // ---- Local odometry state ----
    float    x;              // Odometry X (m)
    float    y;              // Odometry Y (m)
    float    heading;        // Heading theta (rad)

    // ---- Commanded velocities ----
    float    v_cmd;          // Command linear velocity (m/s)
    float    omega_cmd;      // Command angular velocity (rad/s)

    // ---- Measured wheel speeds ----
    float    rpm_l;          // Measured Left RPM
    float    rpm_r;          // Measured Right RPM

    // ---- IMU ----
    float    gyro_z;         // Gyroscope rate (rad/s)

    // ---- Power ----
    float    vbat;           // Battery voltage (V)
    float    current_a;      // Motor current (A)

    // ---- UWB ----
    float    uwb_raw;        // Raw UWB range (m, after CFO compensation)
    float    uwb_clean;      // Calibrated UWB range (m, after UWBPreprocessor)
    float    uwb_rssi;       // RSSI (dBm)
    float    uwb_fp_power;   // First path power (dBm)
    uint16_t uwb_std_noise;  // Noise floor (EMI metric)
    uint8_t  uwb_peer_id;    // Target peer (1..4)
    uint8_t  uwb_lde_err;    // Leading edge error flag (0 or 1)
    float    uwb_temp;       // Transceiver temperature (°C)
};

struct LogFooter {
    uint32_t crc32;          // CRC32 of payload (records only)
    char     endMagic[4];    // "MJNA"
};

#pragma pack(pop)

// Static assertion: LogRecord must be exactly 68 bytes
static_assert(sizeof(LogRecord) == 68,
              "LogRecord size mismatch — check struct packing");
