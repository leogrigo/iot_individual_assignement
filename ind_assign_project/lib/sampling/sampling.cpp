#include "sampling.hpp"

#include "../../include/config.hpp"

void acquireNextFFTBuffer(SamplingState& state, QueueHandle_t fftFreeQueue) {
    xQueueReceive(fftFreeQueue, &state.fillBuffer, portMAX_DELAY);
}

void initSamplingState(SamplingState& state, QueueHandle_t fftFreeQueue) {
    acquireNextFFTBuffer(state, fftFreeQueue);
    state.lastSampleUs = micros();
}

void waitNextSample(SamplingState& state, uint32_t periodUs) {
    while ((micros() - state.lastSampleUs) < periodUs) {
        // busy wait for regular cadence
    }

    state.lastSampleUs += periodUs;
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
