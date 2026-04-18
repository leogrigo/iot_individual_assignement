#pragma once

#include <Arduino.h>
#include "../../include/common.hpp"

struct SamplingState {
    FFTSampleWindow* fillBuffer = nullptr;
    uint16_t fftIndex = 0;
    uint32_t fftWindowCounter = 0;
    uint64_t lastSampleUs = 0;
    uint64_t fftAcquisitionStartUs = 0;
};

void acquireNextFFTBuffer(SamplingState& state, QueueHandle_t fftFreeQueue);
void initSamplingState(SamplingState& state, QueueHandle_t fftFreeQueue);
void waitNextSample(SamplingState& state, uint32_t periodUs);
FFTValue readADCSample();
void publishSampleForAggregation(QueueHandle_t sampleQueue, FFTValue value, uint32_t timestampUs);
bool addSampleToFFTWindow(SamplingState& state, FFTValue value, uint64_t nowUs);
void publishFFTWindow(
    QueueHandle_t fftReadyQueue,
    SamplingState& state,
    uint64_t fftAcquisitionEndUs,
    uint32_t periodUs
);
