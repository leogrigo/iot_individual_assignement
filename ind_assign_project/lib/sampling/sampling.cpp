#include "sampling.hpp"

#include "../../include/config.hpp"

#include <esp_sleep.h>

void acquireNextFFTBuffer(SamplingState& state, QueueHandle_t fftFreeQueue) {
    xQueueReceive(fftFreeQueue, &state.fillBuffer, portMAX_DELAY);
}

void initSamplingState(SamplingState& state, QueueHandle_t fftFreeQueue) {
    acquireNextFFTBuffer(state, fftFreeQueue);
    state.lastSampleUs = micros();
}

void waitNextSample(SamplingState& state, uint32_t periodUs) {
    const uint64_t targetUs = state.lastSampleUs + periodUs;
    const uint32_t tickUs = 1000000UL / static_cast<uint32_t>(configTICK_RATE_HZ);

    for (;;) {
        const uint64_t nowUs = micros();
        if (nowUs >= targetUs) {
            break;
        }

        const uint64_t remainingUs = targetUs - nowUs;

        if (LIGHT_SLEEP_TEST_MODE_ENABLED &&
            remainingUs > LIGHT_SLEEP_MEASURED_OVERHEAD_US + SAMPLING_BUSY_WAIT_MARGIN_US) {
            const uint64_t sleepUs =
                remainingUs - LIGHT_SLEEP_MEASURED_OVERHEAD_US - SAMPLING_BUSY_WAIT_MARGIN_US;

            if (esp_sleep_enable_timer_wakeup(sleepUs) == ESP_OK &&
                esp_light_sleep_start() == ESP_OK) {
                continue;
            }
        }

        if (remainingUs <= SAMPLING_BUSY_WAIT_MARGIN_US + (2U * tickUs)) {
            // if time remaining is less than margin + 2 ticks, do a busy wait until targetUs for better precision
            continue;
        }
        TickType_t delayTicks =
            static_cast<TickType_t>((remainingUs - SAMPLING_BUSY_WAIT_MARGIN_US) / tickUs);
        vTaskDelay(delayTicks);
    }

    state.lastSampleUs = targetUs;
}

FFTValue readADCSample() {
    return static_cast<FFTValue>(analogRead(ADC_PIN));
}

void publishSampleForAggregation(QueueHandle_t sampleQueue, FFTValue value, uint32_t timestampUs) {
    Sample sample;
    sample.value = value;
    sample.timestamp_us = timestampUs;
    (void)xQueueSend(sampleQueue, &sample, 0);
}

bool addSampleToFFTWindow(SamplingState& state, FFTValue value, uint64_t nowUs) {
    if (state.fftIndex == 0) {
        state.fftAcquisitionStartUs = nowUs;
    }

    state.fillBuffer->samples[state.fftIndex] = value;
    state.fftIndex++;

    return state.fftIndex >= SAMPLES;
}

void publishFFTWindow(
    QueueHandle_t fftReadyQueue,
    SamplingState& state,
    uint64_t fftAcquisitionEndUs,
    uint32_t periodUs
) {
    float elapsedUs = static_cast<float>(fftAcquisitionEndUs - state.fftAcquisitionStartUs);
    if (elapsedUs <= 0.0f) {
        elapsedUs = static_cast<float>(SAMPLES) * static_cast<float>(periodUs);
    }

    state.fillBuffer->window_id = ++state.fftWindowCounter;
    state.fillBuffer->window_start_us = state.fftAcquisitionStartUs;
    state.fillBuffer->window_end_us = fftAcquisitionEndUs;
    state.fillBuffer->fs_eff = (static_cast<float>(SAMPLES) * 1000000.0f) / elapsedUs;

    xQueueSend(fftReadyQueue, &state.fillBuffer, portMAX_DELAY);
    state.fftIndex = 0;
}
