#pragma once

#include "../../include/config.hpp"
#include "../../include/common.hpp"

struct FFTAnalysisResult {
    uint32_t bufferId = 0;
    float fsEff = 0.0f; // Effective sampling frequency for the acquired window
    float mean = 0.0f;
    float minVal = 0.0f;
    float maxVal = 0.0f;
    float dominantFreq = 0.0f; // Frequency corresponding to the bin with the highest magnitude
    uint16_t dominantBin = 1;
    float dominantBinMag = 0.0f; // Magnitude of the dominant bin
    float threshold = 0.0f;
    uint16_t lastSignificantBin = 0; // Last bin index that is above the significance threshold
    float fMaxSignificant = 0.0f; // Frequency corresponding to the last significant bin
    float fRef = 0.0f;
    float fsAdapted = 0.0f; // Suggested adapted sampling frequency based on the analysis
    float adaptedPeriodUs = 0.0f;
};

FFTAnalysisResult analyzeFFTWindow(FFTSampleWindow* window);

void printFFTAnalysis(
    const FFTSampleWindow* window,
    const FFTAnalysisResult& result,
    uint32_t currentSamplePeriodUs,
    bool verbose
);
