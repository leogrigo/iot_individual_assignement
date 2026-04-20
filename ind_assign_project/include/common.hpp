#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include "config.hpp"

using FFTValue = float;

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
    uint32_t sample_count;
    uint32_t window_start_us;
    uint32_t aggregate_ready_us;
};

// Represent a single sample to be aggregated
struct Sample {
    float value;
    uint32_t timestamp_us;
};
