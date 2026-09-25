#pragma once

#include <Arduino.h>

class UWBPreprocessor {
public:
    // 12-Directed Antenna Biases (m) from empirical report
    static constexpr float DIRECTIONAL_BIAS[4][4] = {
        {   0.0000f,  19.2418f,  21.6532f,  19.5407f }, // R1 outgoing (1->j)
        {  58.5328f,   0.0000f,  41.0874f,  39.3955f }, // R2 outgoing (2->j)
        {  55.7571f,  36.0566f,   0.0000f,  37.0840f }, // R3 outgoing (3->j)
        {  57.6726f,  38.2033f,  40.8134f,   0.0000f }  // R4 outgoing (4->j)
    };

    // Common-Mode Thermal Drift Coefficients b1 (m/°C)
    static constexpr float B1_COMMON[4][4] = {
        {   0.0000f, -0.2466f, -0.1787f, -0.2430f },
        {  +0.2990f,  0.0000f, +0.1227f, -0.0005f },
        {  +0.0774f, -0.1263f,  0.0000f, -0.1332f },
        {  +0.2767f, -0.0087f, +0.1091f,  0.0000f }
    };

    // Differential Thermal Drift Coefficients b2 (m/°C)
    static constexpr float B2_DIFF[4][4] = {
        {   0.0000f, -0.1664f, -0.1072f, -0.1723f },
        {  -0.4949f,  0.0000f, -0.0170f, -0.0128f },
        {  -0.3601f, -0.0517f,  0.0000f, -0.0709f },
        {  -0.4920f, -0.0094f, -0.0225f,  0.0000f }
    };

    static constexpr float T_REF_AVG  = 46.5f; // Fleet equilibrium mean
    static constexpr float T_REF_DIFF = 0.0f;

    static inline float correctDistance(uint8_t initId, uint8_t respId, float distCfoM, float tempInit, float tempResp) {
        if (initId < 1 || initId > 4 || respId < 1 || respId > 4 || initId == respId) {
            return distCfoM;
        }

        uint8_t i = initId - 1;
        uint8_t j = respId - 1;

        float tAvg  = 0.5f * (tempInit + tempResp);
        float tDiff = tempInit - tempResp;

        float deltaTAvg  = tAvg - T_REF_AVG;
        float deltaTDiff = tDiff - T_REF_DIFF;

        float biasStatic = DIRECTIONAL_BIAS[i][j];
        float biasTherm  = (B1_COMMON[i][j] * deltaTAvg) + (B2_DIFF[i][j] * deltaTDiff);

        return distCfoM - biasStatic - biasTherm;
    }
};
