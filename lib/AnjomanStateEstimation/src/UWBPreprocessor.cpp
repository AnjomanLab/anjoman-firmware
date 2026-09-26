#include "UWBPreprocessor.h"

float UWBPreprocessor::correctRawDistance(uint8_t initId, uint8_t respId,
                                          float dRawM,
                                          float tempInit, float tempResp) {
    // Guard: valid device IDs and no self-loop
    if (initId < 1 || initId > 4 || respId < 1 || respId > 4 ||
        initId == respId) {
        return dRawM;
    }

    const uint8_t i = initId - 1;
    const uint8_t j = respId - 1;

    // Temperature deviations from reference
    const float tAvg     = 0.5f * (tempInit + tempResp);
    const float tDiff    = tempInit - tempResp;
    const float deltaAvg = tAvg  - T_REF_AVG;
    const float deltaDif = tDiff - T_REF_DIFF;

    // Static + thermal components
    const float biasStatic  = BIAS_RAW[i][j];
    const float biasThermal = THERMAL_B1[i][j] * deltaAvg
                            + THERMAL_B2[i][j] * deltaDif;

    return dRawM - biasStatic - biasThermal;
}

UWBPreprocessor::CorrectionTerms UWBPreprocessor::computeTerms(
        uint8_t initId, uint8_t respId,
        float tempInit, float tempResp) {
    CorrectionTerms terms = { 0.0f, 0.0f };
    if (initId < 1 || initId > 4 || respId < 1 || respId > 4 ||
        initId == respId) {
        return terms;
    }

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

constexpr float UWBPreprocessor::BIAS_RAW[4][4];
constexpr float UWBPreprocessor::THERMAL_B1[4][4];
constexpr float UWBPreprocessor::THERMAL_B2[4][4];
