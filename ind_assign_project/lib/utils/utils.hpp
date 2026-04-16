#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Data structures

using FFTValue = float;
constexpr uint16_t SAMPLES = 16384;                  // must be power of 2 (needed for FFT processing)

// Represent a sampling window to be processed by the FFT task
struct FFTSampleWindow {
    uint32_t window_id;
    float fs_eff;
    uint64_t window_start_us;
    uint64_t window_end_us;
    FFTValue samples[SAMPLES];
};

// Represent an aggregated value to be sent
struct AggregatedValue {
    uint32_t window_id;
    float mean;
    float duration_ms;
};

// Represent a single sample to be aggregated
struct Sample {
    float value;
    uint32_t timestamp_us;
};