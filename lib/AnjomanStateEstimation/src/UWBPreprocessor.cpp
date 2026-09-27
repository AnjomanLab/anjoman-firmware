#include "UWBPreprocessor.h"

// -----------------------------------------------------------------------------
// Out-of-line definitions for static constexpr arrays (C++11/C++14 requirement)
// -----------------------------------------------------------------------------
constexpr float UWBPreprocessor::BIAS_RAW[4][4];
constexpr float UWBPreprocessor::THERMAL_B1[4][4];
constexpr float UWBPreprocessor::THERMAL_B2[4][4];
constexpr float UWBPreprocessor::VARIANCE_POST_THERMAL[4][4];

namespace {
    inline bool isValidLink(uint8_t initId, uint8_t respId) {
        if (initId < 1 || initId > 4) return false;
        if (respId < 1 || respId > 4) return false;
        if (initId == respId)         return false;
        return true;
    }
}

float UWBPreprocessor::correctRawDistance(uint8_t initId, uint8_t respId,
                                          float dRawM,
                                          float tempInit, float tempResp) {
    if (!isValidLink(initId, respId)) return dRawM;

    const uint8_t i = initId - 1;
    const uint8_t j = respId - 1;

    const float tAvg     = 0.5f * (tempInit + tempResp);
    const float tDiff    = tempInit - tempResp;
    const float deltaAvg = tAvg  - T_REF_AVG;
    const float deltaDif = tDiff - T_REF_DIFF;

    const float biasStatic  = BIAS_RAW[i][j];
    const float biasThermal = THERMAL_B1[i][j] * deltaAvg
                            + THERMAL_B2[i][j] * deltaDif;

    return dRawM - biasStatic - biasThermal;
}

float UWBPreprocessor::getVariance(uint8_t initId, uint8_t respId) {
    if (!isValidLink(initId, respId)) return 0.05f;
    const float v = VARIANCE_POST_THERMAL[initId - 1][respId - 1];
    return (v > 0.0f) ? v : 0.05f;
}

UWBPreprocessor::CorrectionTerms UWBPreprocessor::computeTerms(
        uint8_t initId, uint8_t respId,
        float tempInit, float tempResp) {
    CorrectionTerms terms = { 0.0f, 0.0f };
    if (!isValidLink(initId, respId)) return terms;

    const uint8_t i = initId - 1;
    const uint8_t j = respId - 1;

    const float tAvg     = 0.5f * (tempInit + tempResp);
    const float tDiff    = tempInit - tempResp;
    const float deltaAvg = tAvg  - T_REF_AVG;
    const float deltaDif = tDiff - T_REF_DIFF;

    terms.biasStatic  = BIAS_RAW[i][j];
    terms.biasThermal = THERMAL_B1[i][j] * deltaAvg
                      + THERMAL_B2[i][j] * deltaDif;

    return terms;
}
