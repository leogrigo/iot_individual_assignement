#pragma once

#include "../../include/common.hpp"

struct AggregationState {
    float sum = 0.0f;
    uint32_t sampleCount = 0;
    uint64_t windowStartUs = 0;
    uint32_t windowId = 0;
};

void updateAggregationWindow(AggregationState& state, const Sample& sample);
float getAggregationElapsedMs(const AggregationState& state, const Sample& sample);
bool isAggregationWindowReady(const AggregationState& state, float elapsedMs);
AggregatedValue buildAggregatedValue(const AggregationState& state, float elapsedMs);
void resetAggregationWindow(AggregationState& state);
