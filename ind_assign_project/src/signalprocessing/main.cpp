#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>
#include "comm_edge.hpp"
#include "utils.hpp"

// Configuration
constexpr int ADC_PIN = 1; // Heltec v3 ADC pin
constexpr uint32_t INITIAL_SAMPLE_PERIOD_US = 61;    // ~16.4 kHz target
constexpr uint32_t AGGREGATION_WINDOW_MS = 5000; // 5 seconds for mean aggregation

constexpr float SIGNIFICANCE_RATIO = 0.018f; // Threshold for significant peaks in the FFT, as a ratio of the maximum magnitude
constexpr float ADAPTIVE_MARGIN = 1.5f;       // Margin for adaptive thresholding
constexpr float MIN_ADAPTED_FS = 10.0f;      // Minimum adapted sampling frequency

constexpr size_t FFT_BUFFER_COUNT = 2;
constexpr size_t FFT_READY_QUEUE_LENGTH = FFT_BUFFER_COUNT;
constexpr size_t FFT_FREE_QUEUE_LENGTH  = FFT_BUFFER_COUNT;
constexpr size_t SAMPLE_QUEUE_LENGTH = 256;
constexpr size_t COMM_QUEUE_LENGTH = 4;

constexpr bool FFT_DEBUG_VERBOSE = false;

// Queues
QueueHandle_t fftReadyQueue = nullptr;       // contains FFTSampleWindow*
QueueHandle_t fftFreeQueue = nullptr;        // contains FFTSampleWindow*
QueueHandle_t sampleQueue = nullptr;         // contains Sample
QueueHandle_t communicationQueue = nullptr;  // contains AggregatedValue

// Global/shared state
volatile uint32_t sample_period_us_global = INITIAL_SAMPLE_PERIOD_US;
volatile uint32_t aggregation_window_ms = AGGREGATION_WINDOW_MS;

static FFTValue vImag[SAMPLES]; // Imaginary part for FFT computation
static FFTSampleWindow fftBuffers[FFT_BUFFER_COUNT]; // Pre-allocated buffers for FFT processing, managed via queues

// ===== Helpers =====

// Safe queue creation check
static bool createQueues() {
    fftReadyQueue = xQueueCreate(FFT_READY_QUEUE_LENGTH, sizeof(FFTSampleWindow*));
    fftFreeQueue  = xQueueCreate(FFT_FREE_QUEUE_LENGTH,  sizeof(FFTSampleWindow*));
    sampleQueue   = xQueueCreate(SAMPLE_QUEUE_LENGTH,    sizeof(Sample));
    communicationQueue = xQueueCreate(COMM_QUEUE_LENGTH, sizeof(AggregatedValue));

    return (fftReadyQueue != nullptr &&
            fftFreeQueue  != nullptr &&
            sampleQueue   != nullptr &&
            communicationQueue != nullptr);
}

// ===== Tasks =====

// Task for sampling data
void TaskSample(void* pvParameters) {
    FFTSampleWindow* fillBuffer = nullptr;

    // Wait for first free buffer
    xQueueReceive(fftFreeQueue, &fillBuffer, portMAX_DELAY);

    uint16_t fftIndex = 0;
    uint32_t fft_window_counter = 0;

    uint64_t last_sample_us = micros();
    uint64_t current_window_start_us = last_sample_us;
    uint64_t acquisition_start_us = 0;

    for (;;) {
        const uint32_t local_period_us = sample_period_us_global;

        while ((micros() - last_sample_us) < local_period_us) {
            // busy wait for regular cadence
        }

        last_sample_us += local_period_us;

        const FFTValue sampleValue = static_cast<FFTValue>(analogRead(ADC_PIN));
        const uint32_t now_us_32 = static_cast<uint32_t>(micros());

        // Send sample to aggregation pipeline (best effort)
        Sample aggSample;
        aggSample.value = sampleValue;
        aggSample.timestamp_us = now_us_32;
        (void)xQueueSend(sampleQueue, &aggSample, 0);

        if (fftIndex == 0) {
            acquisition_start_us = micros();
        }

        fillBuffer->samples[fftIndex] = sampleValue;
        fftIndex++;

        if (fftIndex >= SAMPLES) {
            const uint64_t acquisition_end_us = micros();

            float elapsed_us = static_cast<float>(acquisition_end_us - acquisition_start_us);
            if (elapsed_us <= 0.0f) {
                elapsed_us = static_cast<float>(SAMPLES) * static_cast<float>(local_period_us);
            }

            fillBuffer->window_id = ++fft_window_counter;
            fillBuffer->window_start_us = acquisition_start_us;
            fillBuffer->window_end_us = acquisition_end_us;
            fillBuffer->fs_eff = (static_cast<float>(SAMPLES) * 1000000.0f) / elapsed_us;

            // Publish full buffer
            xQueueSend(fftReadyQueue, &fillBuffer, portMAX_DELAY);

            // Immediately ask for another free buffer.
            // Only here TaskSample can block, and only if all buffers are busy.
            xQueueReceive(fftFreeQueue, &fillBuffer, portMAX_DELAY);

            fftIndex = 0;
        }
    }
}

// Task for computing FFT and analyzing results
void TaskFFT(void* pvParameters) {
    for (;;) {
        FFTSampleWindow* window = nullptr;
        xQueueReceive(fftReadyQueue, &window, portMAX_DELAY);

        const uint32_t local_buffer_id = window->window_id;
        const float fs_eff = window->fs_eff;

        // Reset imaginary part
        for (uint16_t i = 0; i < SAMPLES; i++) {
            vImag[i] = 0.0f;
        }

        // Compute mean
        float mean = 0.0f;
        for (uint16_t i = 0; i < SAMPLES; i++) {
            mean += window->samples[i];
        }
        mean /= static_cast<float>(SAMPLES);

        // Remove DC + compute min/max of centered signal
        float min_val = 0.0f;
        float max_val = 0.0f;
        for (uint16_t i = 0; i < SAMPLES; i++) {
            window->samples[i] -= mean;
            if (i == 0) {
                min_val = window->samples[i];
                max_val = window->samples[i];
            } else {
                if (window->samples[i] < min_val) min_val = window->samples[i];
                if (window->samples[i] > max_val) max_val = window->samples[i];
            }
        }

        ArduinoFFT<FFTValue> fft(window->samples, vImag, SAMPLES, fs_eff);
        fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        fft.compute(FFTDirection::Forward);
        fft.complexToMagnitude();

        const float dominant_freq = fft.majorPeak();

        const uint16_t max_bin_to_scan = (SAMPLES / 2) - 1;

        float max_mag = 0.0f;
        for (uint16_t i = 1; i <= max_bin_to_scan; i++) {
            if (window->samples[i] > max_mag) {
                max_mag = window->samples[i];
            }
        }

        const float threshold = SIGNIFICANCE_RATIO * max_mag;

        uint16_t dominant_bin = 1;
        float dominant_bin_mag = window->samples[1];

        uint16_t last_significant_bin = 0;
        float f_max_significant = 0.0f;

        for (uint16_t i = 1; i <= max_bin_to_scan; i++) {
            const float mag = window->samples[i];

            if (mag > dominant_bin_mag) {
                dominant_bin_mag = mag;
                dominant_bin = i;
            }

            if (mag >= threshold) {
                last_significant_bin = i;
                f_max_significant =
                    (static_cast<float>(i) * fs_eff) / static_cast<float>(SAMPLES);
            }
        }

        if (last_significant_bin == 0) {
            last_significant_bin = dominant_bin;
            f_max_significant =
                (static_cast<float>(dominant_bin) * fs_eff) / static_cast<float>(SAMPLES);
        }

        float f_ref = (f_max_significant > 0.0f) ? f_max_significant : dominant_freq;
        float fs_adapted = 2.0f * f_ref * ADAPTIVE_MARGIN;
        if (fs_adapted < MIN_ADAPTED_FS) {
            fs_adapted = MIN_ADAPTED_FS;
        }

        const float adapted_period_us = 1000000.0f / fs_adapted;

        // =========================
        // Print
        // =========================
        if(FFT_DEBUG_VERBOSE) {
            Serial.println();
            Serial.println("========== FFT ANALYSIS ==========");
            Serial.printf("Buffer id            : %lu\n", static_cast<unsigned long>(local_buffer_id));
            Serial.printf("Samples              : %u\n", SAMPLES);
            Serial.printf("Effective fs         : %.3f Hz\n", fs_eff);
            Serial.printf("Current Ts           : %lu us\n", static_cast<unsigned long>(sample_period_us_global));
            Serial.printf("Removed mean         : %.3f\n", mean);
            Serial.printf("Centered min         : %.3f\n", min_val);
            Serial.printf("Centered max         : %.3f\n", max_val);
            Serial.printf("Dominant freq        : %.3f Hz\n", dominant_freq);
            Serial.printf("Dominant bin         : %u\n", dominant_bin);
            Serial.printf("Dominant bin freq    : %.3f Hz\n",
                        (static_cast<float>(dominant_bin) * fs_eff) / static_cast<float>(SAMPLES));
            Serial.printf("Dominant bin mag     : %.3f\n", dominant_bin_mag);
            Serial.printf("Threshold            : %.3f\n", threshold);
            Serial.printf("Estimated f_max      : %.3f Hz\n", f_max_significant);
            Serial.printf("Last significant bin : %u\n", last_significant_bin);
            Serial.printf("Last significant freq: %.3f Hz\n",
                        (static_cast<float>(last_significant_bin) * fs_eff) / static_cast<float>(SAMPLES));
            Serial.println("----------------------------------");
            Serial.println("Bins (freq Hz -> magnitude):");

            const uint16_t max_bins_to_print = 50;
            const uint16_t bins_to_print = (last_significant_bin < max_bins_to_print) ? last_significant_bin : max_bins_to_print;

            for (uint16_t i = 1; i <= bins_to_print; i++) {
                const float bin_freq =
                    (static_cast<float>(i) * fs_eff) / static_cast<float>(SAMPLES);
                const float mag = window->samples[i];

                Serial.printf("%.3f Hz -> %.3f", bin_freq, mag);

                if (i == dominant_bin) {
                    Serial.print("   <-- dominant bin");
                }
                if (mag >= threshold) {
                    Serial.print("   <-- significant");
                }

                Serial.println();
            }

            Serial.println("----------------------------------");
            Serial.println("Adaptive sampling suggestion:");
            Serial.printf("Reference freq       : %.3f Hz\n", f_ref);
            Serial.printf("Adapted fs           : %.3f Hz\n", fs_adapted);
            Serial.printf("Adapted Ts           : %.1f us\n", adapted_period_us);
            Serial.println("==================================");
        }
        else {
            Serial.printf("FFT Buffer %lu: fmax %.3f Hz, adapt fs %.3f Hz\n",
                          static_cast<unsigned long>(local_buffer_id),
                          f_ref,
                          fs_adapted);
        }
        // If you want to actually apply adaptive sampling:
        // sample_period_us_global = static_cast<uint32_t>(adapted_period_us);

        // Return buffer to free pool
        xQueueSend(fftFreeQueue, &window, portMAX_DELAY);
    }
}

// Task for computing the aggregate value (mean)
void TaskAggregateValue(void* pvParameters) {
    float sum = 0.0f;
    uint32_t sample_count = 0;
    uint64_t window_start_us = 0;
    uint32_t aggregate_window_id = 0;

    for (;;) {
        Sample sample{};
        xQueueReceive(sampleQueue, &sample, portMAX_DELAY);

        if (window_start_us == 0) {
            window_start_us = sample.timestamp_us;
            aggregate_window_id++;
        }

        sum += sample.value;
        sample_count++;

        const float elapsed_ms =
            static_cast<float>(sample.timestamp_us - window_start_us) / 1000.0f;

        if (elapsed_ms >= static_cast<float>(aggregation_window_ms) && sample_count > 0) {
            AggregatedValue agg;
            agg.window_id = aggregate_window_id;
            agg.mean = sum / static_cast<float>(sample_count);
            agg.duration_ms = elapsed_ms;

            xQueueSend(communicationQueue, &agg, portMAX_DELAY);

            sum = 0.0f;
            sample_count = 0;
            window_start_us = 0;
        }
    }
}

// Task for handling communication with the edge server and cloud
void TaskCommunication(void* pvParameters) {
    edge_comm_init();
    for (;;) {
        edge_comm_loop();
        AggregatedValue agg{};
        if (xQueueReceive(communicationQueue, &agg, portMAX_DELAY) == pdTRUE) {
            bool edge_sent = edge_comm_send(agg);
            if(edge_sent) { Serial.printf("Sent aggregate value to edge: window_id=%lu, mean=%.3f, duration=%.1f ms\n",
                                  static_cast<unsigned long>(agg.window_id),
                                  agg.mean,
                                  agg.duration_ms);
            } else {
                Serial.println("Failed to send aggregate value to edge.");
            }
        }
    }
}

// ===== Setup / Loop =====

void setup() {
    Serial.begin(115200);
    delay(1000);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PIN, INPUT);

    Serial.println("Signal Processing boot");
    Serial.printf("ADC pin: GPIO%d\n", ADC_PIN);
    Serial.printf("Initial target fs: %.2f Hz\n", 1000000.0f / static_cast<float>(INITIAL_SAMPLE_PERIOD_US));
    Serial.printf("Aggregation window: %lu ms\n", static_cast<unsigned long>(AGGREGATION_WINDOW_MS));

    if (!createQueues()) {
        Serial.println("Queue creation failed.");
        while (true) {
            delay(1000);
        }
    }

    // Initially all FFT buffers are free
    for (size_t i = 0; i < FFT_BUFFER_COUNT; i++) {
        FFTSampleWindow* ptr = &fftBuffers[i];
        xQueueSend(fftFreeQueue, &ptr, portMAX_DELAY);
    }

    xTaskCreatePinnedToCore(
        TaskSample,
        "TaskSample",
        8192,
        nullptr,
        3,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        TaskFFT,
        "TaskFFT",
        12288,
        nullptr,
        2,
        nullptr,
        0
    );

    xTaskCreatePinnedToCore(
        TaskAggregateValue,
        "TaskAggregate",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    xTaskCreatePinnedToCore(
        TaskCommunication,
        "TaskComm",
        6114,
        nullptr,
        1,
        nullptr,
        0
    );
}

void loop() {
    vTaskDelete(nullptr);
}
