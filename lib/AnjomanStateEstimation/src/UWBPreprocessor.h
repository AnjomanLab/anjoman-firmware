#pragma once

#include <Arduino.h>

// ==============================================================================
// UWB Preprocessor — deterministic bias + thermal correction
// ==============================================================================
// Removes the deterministic bias and temperature-driven drift from the raw
// UWB range BEFORE the measurement enters any Kalman filter.
//
// Input convention:
//   The tables below apply to the RAW distance:
//       d_raw = ToF_raw_ticks × TIME_UNIT_SEC × c
//   where ToF_raw_ticks = (tRound − tReply) / 2, WITHOUT CFO compensation.
//
// Calibration provenance:
//   • All 12 directed links (2 m square, static, 10 min).
//   • One numerical outlier was removed (dist_raw_m = −64650 m, R4→R3 link).
//   • Additive model: d_meas = d_true + A_i + B_j + c, with sum(A)=sum(B)=0.
//   • RMSE of static bias model: 13.83 cm.
//   • THERMAL_B1/B2 are per-directed-link linear coefficients of the residual
//     versus mean and differential chip temperature.
//   • VARIANCE_POST_THERMAL is the residual variance after all corrections;
//     used directly as the diagonal element of R in the EKF.
// ==============================================================================

class UWBPreprocessor {
public:
    // -------------------------------------------------------------------------
    // Static bias per directed link [m]
    //   Row = initiator (0..3 → R1..R4)
    //   Col = responder (0..3 → R1..R4)
    // -------------------------------------------------------------------------
    static constexpr float BIAS_RAW[4][4] = {
        //       → R1          → R2          → R3          → R4
        {   0.00000f,   +19.08774f,   +21.59553f,   +19.74432f },  // R1
        { +58.35179f,     0.00000f,   +41.25733f,   +39.40612f },  // R2
        { +55.81512f,   +36.21287f,     0.00000f,   +36.86945f },  // R3
        { +57.79520f,   +38.19295f,   +40.70074f,     0.00000f }   // R4
    };

    // -------------------------------------------------------------------------
    // Thermal coefficient on mean temperature (m / °C)
    //     ΔT_avg = (T_i + T_j)/2 − T_REF_AVG
    // -------------------------------------------------------------------------
    static constexpr float THERMAL_B1[4][4] = {
        {   0.00000f,    -0.12035f,    -0.08478f,    -0.11822f },  // R1
        {  +0.15341f,     0.00000f,    +0.06586f,    +0.00275f },  // R2
        {  +0.04406f,    -0.05913f,     0.00000f,    -0.06319f },  // R3
        {  +0.14169f,    -0.00125f,    +0.05830f,     0.00000f }   // R4
    };

    // -------------------------------------------------------------------------
    // Thermal coefficient on differential temperature (m / °C)
    //     ΔT_diff = T_i − T_j
    // -------------------------------------------------------------------------
    static constexpr float THERMAL_B2[4][4] = {
        {   0.00000f,    -0.08322f,    -0.05545f,    -0.08763f },  // R1
        {  -0.24521f,     0.00000f,    -0.00896f,    -0.00665f },  // R2
        {  -0.17807f,    -0.02419f,     0.00000f,    -0.03515f },  // R3
        {  -0.24585f,    -0.00549f,    -0.01183f,     0.00000f }   // R4
    };

    // -------------------------------------------------------------------------
    // Post-thermal measurement variance per directed link [m²]
    //   Used as the diagonal element of R in the EKF.
    //   Values reflect the observed residual σ after all corrections.
    // -------------------------------------------------------------------------
    static constexpr float VARIANCE_POST_THERMAL[4][4] = {
        //       → R1          → R2          → R3          → R4
        {   0.000000f,    0.068956f,    0.065759f,    0.072837f },  // R1
        {   0.035141f,    0.000000f,    0.002154f,    0.001717f },  // R2
        {   0.011790f,    0.001583f,    0.000000f,    0.000904f },  // R3
        {   0.051209f,    0.001944f,    0.001129f,    0.000000f }   // R4
    };

    // -------------------------------------------------------------------------
    // Reference temperatures used during calibration
    // -------------------------------------------------------------------------
    static constexpr float T_REF_AVG  = 46.7178f;
    static constexpr float T_REF_DIFF =  0.0000f;

    // -------------------------------------------------------------------------
    // Apply the deterministic correction
    // -------------------------------------------------------------------------
    static float correctRawDistance(uint8_t initId, uint8_t respId,
                                    float dRawM,
                                    float tempInit, float tempResp);

    // -------------------------------------------------------------------------
    // Return the post-thermal variance for a given link [m²]
    // -------------------------------------------------------------------------
    static float getVariance(uint8_t initId, uint8_t respId);

    // -------------------------------------------------------------------------
    // Diagnostic: return the individual bias components for logging
    // -------------------------------------------------------------------------
    struct CorrectionTerms {
        float biasStatic;   // [m]
        float biasThermal;  // [m]
    };
    static CorrectionTerms computeTerms(uint8_t initId, uint8_t respId,
                                        float tempInit, float tempResp);
};
