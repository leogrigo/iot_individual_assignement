#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

constexpr int ADC_PIN = 2; // Heltec v3 ADC pin
constexpr uint32_t INITIAL_SAMPLE_PERIOD_US = 61; // ~16.4 kHz (max sampling frequency identified)
constexpr float AGGREGATION_WINDOW_MS = 5000.0f; // 5 seconds for mean aggregation

constexpr float SIGNIFICANCE_RATIO = 0.018f; // Threshold for significant peaks in the FFT, as a ratio of the maximum magnitude
constexpr float ADAPTIVE_MARGIN = 1.5f; // Margin for adaptive thresholding
constexpr float MIN_ADAPTED_FS = 10.0f; // Minimum adapted sampling frequency
constexpr uint16_t SAMPLES = 16384; // must be power of 2 (needed for FFT processing)

constexpr bool ADAPTIVE_SAMPLING_FREQUENCY_ENABLED = true;
constexpr uint32_t ADAPTIVE_SAMPLING_ROUNDS = 30;
static_assert(ADAPTIVE_SAMPLING_ROUNDS > 0, "ADAPTIVE_SAMPLING_ROUNDS must be greater than 0");

constexpr size_t FFT_BUFFER_COUNT = 2;
constexpr size_t FFT_READY_QUEUE_LENGTH = FFT_BUFFER_COUNT;
constexpr size_t FFT_FREE_QUEUE_LENGTH = FFT_BUFFER_COUNT;
constexpr size_t SAMPLE_QUEUE_LENGTH = 256;
constexpr size_t COMM_QUEUE_LENGTH = 4;

constexpr bool FFT_DEBUG_VERBOSE = false;
