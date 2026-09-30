#pragma once

#include <Arduino.h>

// ==============================================================================
// UWB Preprocessor — deterministic static bias correction only
// ==============================================================================
// Thermal correction has been REMOVED because:
//   • SAR register on these DWM1000 modules returns 0 (firmware bug)
//   • Proxy temperature from ESP32 worsened bias in tests
//   • In the 20-35 °C range, thermal effect is < 5 cm — below noise floor
//
// Input convention:
//   d_raw = ToF_raw_ticks × TIME_UNIT_SEC × c
//   where ToF_raw_ticks = (tRound − tReply) / 2, WITHOUT CFO.
//
// Calibration provenance:
//   • 12 directed links (2 m square, static, 10 min).
//   • Additive model: d_meas = d_true + A_i + B_j + c.
//   • RMSE of static bias model: 13.83 cm.
// ==============================================================================

class UWBPreprocessor {
public:
    // Static bias per directed link [m]
    // Row = initiator (0..3 → R1..R4)
    // Col = responder (0..3 → R1..R4)
    static constexpr float BIAS_RAW[4][4] = {
        {   0.00000f,   +19.08774f,   +21.59553f,   +19.74432f },  // R1
        { +58.35179f,     0.00000f,   +41.25733f,   +39.40612f },  // R2
        { +55.81512f,   +36.21287f,     0.00000f,   +36.86945f },  // R3
        { +57.79520f,   +38.19295f,   +40.70074f,     0.00000f }   // R4
    };

    // Post-correction residual variance per directed link [m²]
    static constexpr float VARIANCE_POST_THERMAL[4][4] = {
        {   0.000000f,    0.068956f,    0.065759f,    0.072837f },  // R1
        {   0.035141f,    0.000000f,    0.002154f,    0.001717f },  // R2
        {   0.011790f,    0.001583f,    0.000000f,    0.000904f },  // R3
        {   0.051209f,    0.001944f,    0.001129f,    0.000000f }   // R4
    };

    static float correctRawDistance(uint8_t initId, uint8_t respId,
                                    float dRawM,
                                    float tempInit, float tempResp);

    static float getVariance(uint8_t initId, uint8_t respId);

    struct CorrectionTerms {
        float biasStatic;
    };
    static CorrectionTerms computeTerms(uint8_t initId, uint8_t respId,
                                        float tempInit, float tempResp);
};
