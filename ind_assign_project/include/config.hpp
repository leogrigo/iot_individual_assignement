#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Feature toggles
constexpr bool ADAPTIVE_SAMPLING_FREQUENCY_ENABLED = true; // Enables FFT-based sampling-rate adaptation
constexpr uint32_t ADAPTIVE_SAMPLING_ROUNDS = 25; // FFT windows averaged before locking the adapted rate
static_assert(ADAPTIVE_SAMPLING_ROUNDS > 0, "ADAPTIVE_SAMPLING_ROUNDS must be greater than 0");
constexpr bool LIGHT_SLEEP_TEST_MODE_ENABLED = false; // Enables manual light sleep benchmarking and disables communication
constexpr bool FFT_DEBUG_VERBOSE = false; // Enables detailed FFT bin-by-bin debug output

// Hardware configuration
constexpr int ADC_PIN = 2; // Heltec v3 ADC pin

// Sampling and aggregation timing
constexpr uint32_t INITIAL_SAMPLE_PERIOD_US = 61; // ~16.4 kHz (max sampling frequency identified)
constexpr float AGGREGATION_WINDOW_MS = 5000.0f; // 5 seconds for mean aggregation
constexpr uint32_t EDGE_RTT_FIXED_US = 60000; // Fixed MQTT RTT measured experimentally (~60 ms)

// Light sleep / timing compensation
constexpr uint32_t LIGHT_SLEEP_MEASURED_OVERHEAD_US = 284; // 2x measured max overhead (~142 us) on target board
constexpr uint32_t SAMPLING_BUSY_WAIT_MARGIN_US = 250; // Final precise wait before sampling

// FFT and adaptive sampling parameters
constexpr float SIGNIFICANCE_RATIO = 0.018f; // Threshold for significant peaks in the FFT, as a ratio of the maximum magnitude
constexpr float ADAPTIVE_MARGIN = 1.5f; // Margin for adaptive thresholding
constexpr float MIN_ADAPTED_FS = 10.0f; // Minimum adapted sampling frequency
constexpr uint16_t SAMPLES = 16384; // must be power of 2 (needed for FFT processing)

// Buffering and queue sizing
constexpr size_t FFT_BUFFER_COUNT = 2;
constexpr size_t FFT_READY_QUEUE_LENGTH = FFT_BUFFER_COUNT;
constexpr size_t FFT_FREE_QUEUE_LENGTH = FFT_BUFFER_COUNT;
constexpr size_t SAMPLE_QUEUE_LENGTH = 256;
constexpr size_t COMM_QUEUE_LENGTH = 4;
