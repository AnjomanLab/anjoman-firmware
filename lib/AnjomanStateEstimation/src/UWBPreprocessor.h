#pragma once

#include <Arduino.h>

// ==============================================================================
// UWB Preprocessor — Deterministic bias + thermal correction
// ==============================================================================
// This class removes the deterministic bias and temperature-driven drift
// from the raw UWB range BEFORE it enters the Kalman filter.
//
// IMPORTANT: The bias and thermal tables below apply to the **RAW** distance:
//
//     d_raw = ToF_raw_ticks × TIME_UNIT_SEC × c
//
// where ToF_raw_ticks = (tRound − tReply) / 2, WITHOUT CFO compensation.
//
// Rationale:
//   • The 2 m static calibration yielded these biases on dist_raw_m.
//   • CFO compensation has shown a sign inconsistency during motion.
//   • Using raw distance + explicit bias is more predictable.
//
// For diagnostics, both d_raw and d_cfo can be logged separately.
// ==============================================================================

class UWBPreprocessor {
public:
    // -------------------------------------------------------------------------
    // Static bias per directed link [m], applied to d_raw
    // -------------------------------------------------------------------------
    // Row = initiator device index (0-based)
    // Col = responder device index (0-based)
    // Measured from the 2 m static square benchmark (12 directed links).
    static constexpr float BIAS_RAW[4][4] = {
        //   →R1         →R2         →R3         →R4
        {   0.0000f,   19.2418f,   21.6532f,   19.5407f },  // R1 outgoing
        {  58.5328f,    0.0000f,   41.0874f,   39.3955f },  // R2 outgoing
        {  55.7571f,   36.0566f,    0.0000f,   37.0840f },  // R3 outgoing
        {  57.6726f,   38.2033f,   40.8134f,    0.0000f }   // R4 outgoing
    };

    // -------------------------------------------------------------------------
    // Thermal coefficient on the mean temperature: ΔT_avg = (T_i + T_j)/2 − T_ref
    // Units: m / °C
    // -------------------------------------------------------------------------
    static constexpr float THERMAL_B1[4][4] = {
        {   0.0000f,  -0.2466f,  -0.1787f,  -0.2430f },
        {  +0.2990f,   0.0000f,  +0.1227f,  -0.0005f },
        {  +0.0774f,  -0.1263f,   0.0000f,  -0.1332f },
        {  +0.2767f,  -0.0087f,  +0.1091f,   0.0000f }
    };

    // -------------------------------------------------------------------------
    // Thermal coefficient on the differential temperature: ΔT_diff = T_i − T_j
    // Units: m / °C
    // -------------------------------------------------------------------------
    static constexpr float THERMAL_B2[4][4] = {
        {   0.0000f,  -0.1664f,  -0.1072f,  -0.1723f },
        {  -0.4949f,   0.0000f,  -0.0170f,  -0.0128f },
        {  -0.3601f,  -0.0517f,   0.0000f,  -0.0709f },
        {  -0.4920f,  -0.0094f,  -0.0225f,   0.0000f }
    };

    // Reference temperatures used during calibration
    static constexpr float T_REF_AVG  = 48.6f;   // Mean fleet temperature during calibration
    static constexpr float T_REF_DIFF = 0.0f;

    // -------------------------------------------------------------------------
    // Apply the deterministic correction
    // -------------------------------------------------------------------------
    // @param initId     Device ID of the initiator (1..4)
    // @param respId     Device ID of the responder (1..4)
    // @param dRawM      RAW distance in meters (see file header for definition)
    // @param tempInit   Initiator chip temperature [°C]
    // @param tempResp   Responder chip temperature [°C]
    //
    // @return           Bias-corrected distance in meters (may be negative if
    //                   d_raw is smaller than the bias — caller should handle)
    // -------------------------------------------------------------------------
    static float correctRawDistance(uint8_t initId, uint8_t respId,
                                    float dRawM,
                                    float tempInit, float tempResp);

    // -------------------------------------------------------------------------
    // Diagnostics: returns the individual bias components for logging
    // -------------------------------------------------------------------------
    struct CorrectionTerms {
        float biasStatic;   // [m]
        float biasThermal;  // [m]
    };
    static CorrectionTerms computeTerms(uint8_t initId, uint8_t respId,
                                        float tempInit, float tempResp);
};
