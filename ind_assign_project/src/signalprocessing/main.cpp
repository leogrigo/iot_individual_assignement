#include <Arduino.h>
#include "config.hpp"
#include "common.hpp"
#include "comm_edge.hpp"
#include "comm_cloud.hpp"
#include "aggregation.hpp"
#include "fft_processing.hpp"
#include "sampling.hpp"

#include <esp_pm.h>

// Queues
QueueHandle_t fftReadyQueue = nullptr;       // contains FFTSampleWindow*
QueueHandle_t fftFreeQueue = nullptr;        // contains FFTSampleWindow*
QueueHandle_t sampleQueue = nullptr;         // contains Sample
QueueHandle_t communicationQueue = nullptr;  // contains AggregatedValue

// Global variables
volatile uint32_t sample_period_us_global = INITIAL_SAMPLE_PERIOD_US;
volatile bool adaptive_sampling_applied_global = !ADAPTIVE_SAMPLING_FREQUENCY_ENABLED;
static FFTSampleWindow fftBuffers[FFT_BUFFER_COUNT]; // Pre-allocated buffers for FFT processing, managed via queues

// ===== Helpers =====
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

static void configureAutoLightSleep() {
    esp_pm_config_esp32s3_t pmConfig;
    pmConfig.max_freq_mhz = static_cast<int>(getCpuFrequencyMhz());
    pmConfig.min_freq_mhz = 80;
    pmConfig.light_sleep_enable = true;

    const esp_err_t err = esp_pm_configure(&pmConfig);
    if (err == ESP_OK) {
        Serial.printf("[PM] Auto light sleep enabled, CPU %d-%d MHz\n",
                      pmConfig.min_freq_mhz,
                      pmConfig.max_freq_mhz);
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        Serial.println("[PM] Auto light sleep not supported by this Arduino core build.");
        Serial.println("[PM] CONFIG_PM_ENABLE must be enabled in the ESP-IDF sdkconfig.");
    } else {
        Serial.printf("[PM] Auto light sleep configuration failed: %s (%d)\n",
                      esp_err_to_name(err),
                      err);
    }
}

static FFTSampleWindow* receiveFFTWindow() {
    FFTSampleWindow* window = nullptr;
    xQueueReceive(fftReadyQueue, &window, portMAX_DELAY);
    return window;
}

static void releaseFFTWindow(FFTSampleWindow* window) {
    xQueueSend(fftFreeQueue, &window, portMAX_DELAY);
}

static Sample receiveAggregationSample() {
    Sample sample{};
    xQueueReceive(sampleQueue, &sample, portMAX_DELAY);
    return sample;
}

static void publishAggregatedValue(const AggregatedValue& agg) {
    if (LIGHT_SLEEP_TEST_MODE_ENABLED) {
        return;
    }

    xQueueSend(communicationQueue, &agg, portMAX_DELAY);
}

static bool receiveAggregatedValueForCommunication(AggregatedValue& agg) {
    return xQueueReceive(communicationQueue, &agg, pdMS_TO_TICKS(50)) == pdTRUE;
}

static void sendAggregatedValue(const AggregatedValue& agg) {
    edge_comm_send(agg);
    if (!cloud_comm_is_ready()) {
        Serial.println("[CLOUD] Cloud communication not ready, skipping sending aggregate value to cloud.");
        return;
    }
    cloud_comm_send(agg);
}

// ===== Tasks =====

// Task for sampling data
void TaskSample(void* pvParameters) {
    SamplingState state;
    initSamplingState(state, fftFreeQueue);

    for (;;) {
        const uint32_t local_period_us = sample_period_us_global;

        waitNextSample(state, local_period_us);

        const FFTValue sampleValue = readADCSample();
        const uint32_t sample_timestamp_us = static_cast<uint32_t>(micros());

        publishSampleForAggregation(sampleQueue, sampleValue, sample_timestamp_us);

        if (!adaptive_sampling_applied_global &&
            addSampleToFFTWindow(state, sampleValue, sample_timestamp_us)) {
            publishFFTWindow(fftReadyQueue, state, sample_timestamp_us, local_period_us);
            acquireNextFFTBuffer(state, fftFreeQueue);
        }
    }
}

// Task for computing FFT and analyzing results
void TaskFFT(void* pvParameters) {
    float fsAdaptedSum = 0.0f;
    uint32_t adaptiveRoundsCompleted = 0;

    for (;;) {
        FFTSampleWindow* window = receiveFFTWindow();
        const FFTAnalysisResult result = analyzeFFTWindow(window);

        printFFTAnalysis(window, result, sample_period_us_global, FFT_DEBUG_VERBOSE);

        bool stopFFTAfterRelease = false;

        if (ADAPTIVE_SAMPLING_FREQUENCY_ENABLED && !adaptive_sampling_applied_global) {
            fsAdaptedSum += result.fsAdapted;
            adaptiveRoundsCompleted++;

            if (adaptiveRoundsCompleted >= ADAPTIVE_SAMPLING_ROUNDS) {
                const float averageFsAdapted =
                    fsAdaptedSum / static_cast<float>(ADAPTIVE_SAMPLING_ROUNDS);
                uint32_t adaptedPeriodUs =
                    static_cast<uint32_t>((1000000.0f / averageFsAdapted) + 0.5f);
                if (adaptedPeriodUs == 0) {
                    adaptedPeriodUs = 1;
                }

                sample_period_us_global = adaptedPeriodUs;
                adaptive_sampling_applied_global = true;
                stopFFTAfterRelease = true;

                Serial.println("[FFT] Official adapted sampling frequency calculated.");
                Serial.printf("[FFT] fs_adapted: %.3f Hz\n", averageFsAdapted);
                Serial.printf("[FFT] New global sample period: %lu us\n",
                              static_cast<unsigned long>(sample_period_us_global));
                Serial.println("[FFT] FFT processing stopped after adaptive calculation.");

            }
        }

        releaseFFTWindow(window);

        if (stopFFTAfterRelease) {
            vTaskSuspend(nullptr);
        }
    }
}

// Task for computing the aggregate value (mean)
void TaskAggregateValue(void* pvParameters) {
    AggregationState state;

    for (;;) {
        const Sample sample = receiveAggregationSample();

        const uint64_t t0 = micros();
        updateAggregationWindow(state, sample);
        const float elapsedMs = getAggregationElapsedMs(state, sample);
        const uint64_t t1 = micros();
        state.processingOverheadUs += (t1 - t0);

        if (isAggregationWindowReady(state, elapsedMs)) {
            const uint64_t t2 = micros();
            const AggregatedValue agg = buildAggregatedValue(state, elapsedMs);
            publishAggregatedValue(agg);
            const uint64_t t3 = micros();
            state.processingOverheadUs += (t3 - t2);
            logWindowProcessingMetric(agg, state.processingOverheadUs);
            resetAggregationWindow(state);
        }
    }
}

// Task for handling communication with the edge server and cloud
void TaskCommunication(void* pvParameters) {
    edge_comm_init();
    cloud_comm_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        edge_comm_loop();
        cloud_comm_loop();

        AggregatedValue agg{};
        if (receiveAggregatedValueForCommunication(agg)) {
            sendAggregatedValue(agg);
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

    Serial.println("===== Signal Processing boot =====");
    Serial.printf("ADC pin: GPIO%d\n", ADC_PIN);
    Serial.printf("Initial target fs: %.2f Hz\n", 1000000.0f / static_cast<float>(INITIAL_SAMPLE_PERIOD_US));
    Serial.printf("Aggregation window: %lu ms\n", static_cast<unsigned long>(AGGREGATION_WINDOW_MS));
    Serial.printf("FFT Verbose debug: %s\n", FFT_DEBUG_VERBOSE ? "ON" : "OFF");
    Serial.printf("Adaptive sampling frequency: %s\n",
                  ADAPTIVE_SAMPLING_FREQUENCY_ENABLED ? "ON" : "OFF");
    Serial.printf("Adaptive sampling rounds: %lu\n",
                  static_cast<unsigned long>(ADAPTIVE_SAMPLING_ROUNDS));
    Serial.printf("Light sleep test mode: %s\n",
                  LIGHT_SLEEP_TEST_MODE_ENABLED ? "ON" : "OFF");

    // configureAutoLightSleep();

    if (!createQueues()) {
        Serial.println("Queue creation failed.");
        while (true) {
            delay(1000);
        }
    } else {
        Serial.println("Queues created successfully.");
    }

    // Initially all FFT buffers are free
    for (size_t i = 0; i < FFT_BUFFER_COUNT; i++) {
        FFTSampleWindow* ptr = &fftBuffers[i];
        xQueueSend(fftFreeQueue, &ptr, portMAX_DELAY);
    }

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

    if (!LIGHT_SLEEP_TEST_MODE_ENABLED) {
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

    Serial.println("===================================");

    xTaskCreatePinnedToCore(
        TaskSample,
        "TaskSample",
        8192,
        nullptr,
        3,
        nullptr,
        1
    );

}

void loop() {
}
