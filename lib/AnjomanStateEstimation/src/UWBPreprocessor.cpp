#include "UWBPreprocessor.h"

constexpr float UWBPreprocessor::BIAS_RAW[4][4];
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
    (void)tempInit;
    (void)tempResp;
    if (!isValidLink(initId, respId)) return dRawM;
    return dRawM - BIAS_RAW[initId - 1][respId - 1];
}

float UWBPreprocessor::getVariance(uint8_t initId, uint8_t respId) {
    if (!isValidLink(initId, respId)) return 0.05f;
    const float v = VARIANCE_POST_THERMAL[initId - 1][respId - 1];
    return (v > 0.0f) ? v : 0.05f;
}

UWBPreprocessor::CorrectionTerms UWBPreprocessor::computeTerms(
        uint8_t initId, uint8_t respId,
        float tempInit, float tempResp) {
    (void)tempInit;
    (void)tempResp;
    CorrectionTerms terms = { 0.0f };
    if (!isValidLink(initId, respId)) return terms;
    terms.biasStatic = BIAS_RAW[initId - 1][respId - 1];
    return terms;
}
