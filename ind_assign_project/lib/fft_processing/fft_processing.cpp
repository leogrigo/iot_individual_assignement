#include "fft_processing.hpp"
#include <Arduino.h>
#include <arduinoFFT.h>

static FFTValue vImag[SAMPLES];

static void resetFFTImaginaryPart() {
    for (uint16_t i = 0; i < SAMPLES; i++) {
        vImag[i] = 0.0f;
    }
}

static void removeFFTMean(FFTSampleWindow* window, FFTAnalysisResult& result) {
    for (uint16_t i = 0; i < SAMPLES; i++) {
        result.mean += window->samples[i];
    }
    result.mean /= static_cast<float>(SAMPLES);

    for (uint16_t i = 0; i < SAMPLES; i++) {
        window->samples[i] -= result.mean;
        if (i == 0) {
            result.minVal = window->samples[i];
            result.maxVal = window->samples[i];
        } else {
            if (window->samples[i] < result.minVal) result.minVal = window->samples[i];
            if (window->samples[i] > result.maxVal) result.maxVal = window->samples[i];
        }
    }
}

static void computeFFT(FFTSampleWindow* window, FFTAnalysisResult& result) {
    ArduinoFFT<FFTValue> fft(window->samples, vImag, SAMPLES, result.fsEff);
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    result.dominantFreq = fft.majorPeak();
}

static void findFFTSignificantBins(FFTSampleWindow* window, FFTAnalysisResult& result) {
    const uint16_t max_bin_to_scan = (SAMPLES / 2) - 1;

    float max_mag = 0.0f;
    for (uint16_t i = 1; i <= max_bin_to_scan; i++) {
        if (window->samples[i] > max_mag) {
            max_mag = window->samples[i];
        }
    }

    result.threshold = SIGNIFICANCE_RATIO * max_mag;
    result.dominantBin = 1;
    result.dominantBinMag = window->samples[1];
    result.lastSignificantBin = 0;
    result.fMaxSignificant = 0.0f;

    for (uint16_t i = 1; i <= max_bin_to_scan; i++) {
        const float mag = window->samples[i];

        if (mag > result.dominantBinMag) {
            result.dominantBinMag = mag;
            result.dominantBin = i;
        }

        if (mag >= result.threshold) {
            result.lastSignificantBin = i;
            result.fMaxSignificant =
                (static_cast<float>(i) * result.fsEff) / static_cast<float>(SAMPLES);
        }
    }

    if (result.lastSignificantBin == 0) {
        result.lastSignificantBin = result.dominantBin;
        result.fMaxSignificant =
            (static_cast<float>(result.dominantBin) * result.fsEff) / static_cast<float>(SAMPLES);
    }
}

static void computeAdaptiveSampling(FFTAnalysisResult& result) {
    result.fRef = (result.fMaxSignificant > 0.0f) ? result.fMaxSignificant : result.dominantFreq;
    result.fsAdapted = 2.0f * result.fRef * ADAPTIVE_MARGIN;
    if (result.fsAdapted < MIN_ADAPTED_FS) {
        result.fsAdapted = MIN_ADAPTED_FS;
    }

    result.adaptedPeriodUs = 1000000.0f / result.fsAdapted;
}

FFTAnalysisResult analyzeFFTWindow(FFTSampleWindow* window) {
    FFTAnalysisResult result;
    result.bufferId = window->window_id;
    result.fsEff = window->fs_eff;

    resetFFTImaginaryPart();
    removeFFTMean(window, result);
    computeFFT(window, result);
    findFFTSignificantBins(window, result);
    computeAdaptiveSampling(result);

    return result;
}

static void printFFTVerbose(
    const FFTSampleWindow* window,
    const FFTAnalysisResult& result,
    uint32_t currentSamplePeriodUs
) {
    Serial.println();
    Serial.println("========== FFT ANALYSIS ==========");
    Serial.printf("Buffer id            : %lu\n", static_cast<unsigned long>(result.bufferId));
    Serial.printf("Samples              : %u\n", SAMPLES);
    Serial.printf("Effective fs         : %.3f Hz\n", result.fsEff);
    Serial.printf("Current Ts           : %lu us\n", static_cast<unsigned long>(currentSamplePeriodUs));
    Serial.printf("Removed mean         : %.3f\n", result.mean);
    Serial.printf("Centered min         : %.3f\n", result.minVal);
    Serial.printf("Centered max         : %.3f\n", result.maxVal);
    Serial.printf("Dominant freq        : %.3f Hz\n", result.dominantFreq);
    Serial.printf("Dominant bin         : %u\n", result.dominantBin);
    Serial.printf("Dominant bin freq    : %.3f Hz\n",
                  (static_cast<float>(result.dominantBin) * result.fsEff) / static_cast<float>(SAMPLES));
    Serial.printf("Dominant bin mag     : %.3f\n", result.dominantBinMag);
    Serial.printf("Threshold            : %.3f\n", result.threshold);
    Serial.printf("Estimated f_max      : %.3f Hz\n", result.fMaxSignificant);
    Serial.printf("Last significant bin : %u\n", result.lastSignificantBin);
    Serial.printf("Last significant freq: %.3f Hz\n",
                  (static_cast<float>(result.lastSignificantBin) * result.fsEff) / static_cast<float>(SAMPLES));
    Serial.println("----------------------------------");
    Serial.println("Bins (freq Hz -> magnitude):");

    const uint16_t max_bins_to_print = 50;
    const uint16_t bins_to_print =
        (result.lastSignificantBin < max_bins_to_print) ? result.lastSignificantBin : max_bins_to_print;

    for (uint16_t i = 1; i <= bins_to_print; i++) {
        const float bin_freq = (static_cast<float>(i) * result.fsEff) / static_cast<float>(SAMPLES);
        const float mag = window->samples[i];

        Serial.printf("%.3f Hz -> %.3f", bin_freq, mag);

        if (i == result.dominantBin) {
            Serial.print("   <-- dominant bin");
        }
        if (mag >= result.threshold) {
            Serial.print("   <-- significant");
        }

        Serial.println();
    }

    Serial.println("----------------------------------");
    Serial.println("Adaptive sampling suggestion:");
    Serial.printf("Reference freq       : %.3f Hz\n", result.fRef);
    Serial.printf("Adapted fs           : %.3f Hz\n", result.fsAdapted);
    Serial.printf("Adapted Ts           : %.1f us\n", result.adaptedPeriodUs);
    Serial.println("==================================");
}

static void printFFTSummary(const FFTAnalysisResult& result) {
    Serial.printf("[FFT] Buffer %lu: fmax %.3f Hz, adapt fs %.3f Hz\n",
                  static_cast<unsigned long>(result.bufferId),
                  result.fRef,
                  result.fsAdapted);
}

void printFFTAnalysis(
    const FFTSampleWindow* window,
    const FFTAnalysisResult& result,
    uint32_t currentSamplePeriodUs,
    bool verbose
) {
    if (verbose) {
        printFFTVerbose(window, result, currentSamplePeriodUs);
    } else {
        printFFTSummary(result);
    }
}
