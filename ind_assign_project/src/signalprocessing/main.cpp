#include <Arduino.h>
#include <arduinoFFT.h>

// =========================
// Configuration
// =========================
constexpr int ADC_PIN = 1;                 // Heltec V3 ADC pin
constexpr uint16_t SAMPLES = 16384;          // MUST be power of 2
constexpr uint32_t INITIAL_SAMPLE_PERIOD_US = 61; // initial target: ~16.4 kHz

constexpr double SIGNIFICANCE_RATIO = 0.20; // 20% of max magnitude
constexpr double ADAPTIVE_MARGIN = 1.5;     // safety margin over Nyquist
constexpr double MIN_ADAPTED_FS = 10.0;     // lower bound for adapted sampling
constexpr double MAX_BIN_FREQ_TO_PRINT = 20.0; // for debug output
using FFTValue = float;

// =========================
// Shared buffers
// =========================
FFTValue bufferA[SAMPLES];
FFTValue bufferB[SAMPLES];
FFTValue vImag[SAMPLES];

// currentBuffer -> written by sampling task
// processingBuffer -> read by FFT task
FFTValue* currentBuffer = bufferA;
FFTValue* processingBuffer = bufferB;

// =========================
// Shared state between tasks
// =========================
TaskHandle_t FFTTaskHandle = nullptr;

// Effective sampling frequency measured by TaskSample
volatile double fs_eff_global = 1000.0;

// Current sample period used by TaskSample
volatile uint32_t sample_period_us_global = INITIAL_SAMPLE_PERIOD_US;

// Simple buffer counter for logs
volatile uint32_t buffer_counter = 0;

// =========================
// Task: Sampling
// =========================
void TaskSample(void* pvParameters) {
    uint64_t last_sample_us = micros();

    while (true) {
        uint64_t start_us = micros();

        for (uint16_t i = 0; i < SAMPLES; i++) {
            const uint32_t local_period_us = sample_period_us_global;

            while ((micros() - last_sample_us) < local_period_us) {
                // busy wait for regular sampling
            }

            last_sample_us += local_period_us;
            currentBuffer[i] = static_cast<FFTValue>(analogRead(ADC_PIN));
        }

        uint64_t end_us = micros();
        double elapsed_us = static_cast<double>(end_us - start_us);
        double fs_eff = (static_cast<double>(SAMPLES) * 1000000.0) / elapsed_us;

        fs_eff_global = fs_eff;
        buffer_counter++;

        // Swap buffers
        FFTValue* temp = currentBuffer;
        currentBuffer = processingBuffer;
        processingBuffer = temp;

        // Notify FFT task that a new buffer is ready
        xTaskNotifyGive(FFTTaskHandle);
    }
}

// =========================
// Task: FFT + Analysis
// =========================
void TaskFFT(void* pvParameters) {
    while (true) {
        // Wait until a full buffer is ready
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const double fs_eff = fs_eff_global;
        const uint32_t local_buffer_id = buffer_counter;

        // 1) Initialize imaginary part
        for (uint16_t i = 0; i < SAMPLES; i++) {
            vImag[i] = 0.0;
        }

        // 2) Compute mean
        double mean = 0.0;
        for (uint16_t i = 0; i < SAMPLES; i++) {
            mean += processingBuffer[i];
        }
        mean /= static_cast<double>(SAMPLES);

        // 3) Remove DC offset
        for (uint16_t i = 0; i < SAMPLES; i++) {
            processingBuffer[i] -= mean;
        }

        // Optional stats on centered signal
        double min_val = processingBuffer[0];
        double max_val = processingBuffer[0];
        for (uint16_t i = 1; i < SAMPLES; i++) {
            if (processingBuffer[i] < min_val) min_val = processingBuffer[i];
            if (processingBuffer[i] > max_val) max_val = processingBuffer[i];
        }

        // 4) FFT
        ArduinoFFT<FFTValue> fft(processingBuffer, vImag, SAMPLES, fs_eff);

        fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        fft.compute(FFTDirection::Forward);
        fft.complexToMagnitude();

        // 5) Dominant peak (interpolated estimate)
        double dominant_freq = fft.majorPeak();

        // 6) Inspect bins up to selected frequency
        uint16_t max_bin_to_print =
            static_cast<uint16_t>((MAX_BIN_FREQ_TO_PRINT * static_cast<double>(SAMPLES)) / fs_eff);

        if (max_bin_to_print >= (SAMPLES / 2)) {
            max_bin_to_print = (SAMPLES / 2) - 1;
        }

        // Find strongest magnitude in printed region (excluding DC)
        double max_mag = 0.0;
        for (uint16_t i = 1; i <= max_bin_to_print; i++) {
            if (processingBuffer[i] > max_mag) {
                max_mag = processingBuffer[i];
            }
        }

        // Threshold for "significant" frequencies
        double threshold = SIGNIFICANCE_RATIO * max_mag;

        // Find highest significant frequency
        double f_max_significant = 0.0;
        uint16_t dominant_bin = 0;
        double dominant_bin_mag = 0.0;

        for (uint16_t i = 1; i <= max_bin_to_print; i++) {
            if (processingBuffer[i] > dominant_bin_mag) {
                dominant_bin_mag = processingBuffer[i];
                dominant_bin = i;
            }

            if (processingBuffer[i] >= threshold) {
                double bin_freq = (static_cast<double>(i) * fs_eff) / static_cast<double>(SAMPLES);
                f_max_significant = bin_freq;
            }
        }

        // 7) Adaptive sampling suggestion
        double f_ref = (f_max_significant > 0.0) ? f_max_significant : dominant_freq;
        double fs_adapted = 2.0 * f_ref * ADAPTIVE_MARGIN;

        if (fs_adapted < MIN_ADAPTED_FS) {
            fs_adapted = MIN_ADAPTED_FS;
        }

        double adapted_period_us = 1000000.0 / fs_adapted;

        // 8) Print results
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
                      (static_cast<double>(dominant_bin) * fs_eff) / static_cast<double>(SAMPLES));
        Serial.printf("Dominant bin mag     : %.3f\n", dominant_bin_mag);
        Serial.printf("Threshold            : %.3f\n", threshold);
        Serial.printf("Estimated f_max      : %.3f Hz\n", f_max_significant);
        Serial.println("----------------------------------");
        Serial.println("Bins (freq Hz -> magnitude):");

        for (uint16_t i = 1; i <= max_bin_to_print; i++) {
            double bin_freq = (static_cast<double>(i) * fs_eff) / static_cast<double>(SAMPLES);
            double mag = processingBuffer[i];

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

        // If you want to actually enable adaptive sampling
        // sample_period_us_global = static_cast<uint32_t>(adapted_period_us);
    }
}

// =========================
// Arduino setup/loop
// =========================
void setup() {
    Serial.begin(115200);
    delay(1000);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PIN, INPUT);

    Serial.println("Signal Processing FFT boot");
    Serial.printf("ADC pin: GPIO%d\n", ADC_PIN);
    Serial.printf("Samples: %u\n", SAMPLES);
    Serial.printf("Initial target fs: %.2f Hz\n", 1000000.0 / INITIAL_SAMPLE_PERIOD_US);

    xTaskCreatePinnedToCore(
        TaskSample,
        "TaskSample",
        4096,
        nullptr,
        1,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        TaskFFT,
        "TaskFFT",
        8192,
        nullptr,
        1,
        &FFTTaskHandle,
        0
    );
}

void loop() {
    vTaskDelete(nullptr);
}
