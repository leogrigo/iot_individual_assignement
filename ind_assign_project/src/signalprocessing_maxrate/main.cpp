#include <Arduino.h>

constexpr int ADC_PIN = 2;
constexpr uint32_t TEST_DURATION_MS = 1000;
constexpr uint32_t NUM_RUNS = 15;

void setup() {
    Serial.begin(115200);
    delay(1000);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(ADC_PIN, INPUT);

    Serial.println("Signal Processing MaxRate benchmark boot");
    Serial.printf("ADC pin           : GPIO%d\n", ADC_PIN);
    Serial.printf("Test duration     : %lu ms\n", (unsigned long)TEST_DURATION_MS);
    Serial.printf("Number of runs    : %lu\n", (unsigned long)NUM_RUNS);
    Serial.println();
}

void loop() {
    double fs_sum = 0.0;
    double fs_min = 1e12;
    double fs_max = 0.0;

    for (uint32_t run = 1; run <= NUM_RUNS; run++) {
        volatile uint32_t dummy_sink = 0;  // prevents the compiler from optimizing reads away
        uint32_t samples = 0;

        uint64_t start_us = micros();
        uint64_t end_target_us = start_us + (uint64_t)TEST_DURATION_MS * 1000ULL;

        while (micros() < end_target_us) {
            dummy_sink += (uint32_t)analogRead(ADC_PIN);
            samples++;
        }

        uint64_t end_us = micros();
        double elapsed_us = (double)(end_us - start_us);
        double fs = ((double)samples * 1000000.0) / elapsed_us;

        fs_sum += fs;
        if (fs < fs_min) fs_min = fs;
        if (fs > fs_max) fs_max = fs;

        Serial.printf("Run %lu\n", (unsigned long)run);
        Serial.printf("  Samples acquired : %lu\n", (unsigned long)samples);
        Serial.printf("  Elapsed time     : %.0f us\n", elapsed_us);
        Serial.printf("  Throughput       : %.2f samples/s\n", fs);
        Serial.printf("  Dummy sink       : %lu\n", (unsigned long)dummy_sink);
        Serial.println();
    }

    double fs_avg = fs_sum / (double)NUM_RUNS;
    double period_us = 1000000.0 / fs_avg;

    Serial.println("==== Summary ====");
    Serial.printf("Average throughput : %.2f samples/s\n", fs_avg);
    Serial.printf("Minimum throughput : %.2f samples/s\n", fs_min);
    Serial.printf("Maximum throughput : %.2f samples/s\n", fs_max);
    Serial.println("=================");
    Serial.printf("Suggested period   : %.2f us\n", period_us);
    Serial.println();

    delay(5000);
}