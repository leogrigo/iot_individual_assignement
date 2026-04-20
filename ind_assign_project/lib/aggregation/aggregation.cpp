#include "aggregation.hpp"

void updateAggregationWindow(AggregationState& state, const Sample& sample) {
    if (state.windowStartUs == 0) {
        state.windowStartUs = sample.timestamp_us;
        state.windowId++;
    }

    state.sum += sample.value;
    state.sampleCount++;
}

float getAggregationElapsedMs(const AggregationState& state, const Sample& sample) {
    return static_cast<float>(sample.timestamp_us - state.windowStartUs) / 1000.0f;
}

bool isAggregationWindowReady(const AggregationState& state, float elapsedMs) {
    return elapsedMs >= AGGREGATION_WINDOW_MS && state.sampleCount > 0;
}

AggregatedValue buildAggregatedValue(const AggregationState& state, float elapsedMs) {
    AggregatedValue agg;
    agg.window_id = state.windowId;
    agg.mean = state.sum / static_cast<float>(state.sampleCount);
    agg.duration_ms = elapsedMs;
    agg.sample_count = state.sampleCount;

    Serial.printf("[AV] Ready: window_id=%u, mean=%.2f, duration=%.2f ms, samples=%lu\n",
                  agg.window_id,
                  agg.mean,
                  agg.duration_ms,
                  static_cast<unsigned long>(agg.sample_count));
    return agg;
}

void logWindowProcessingMetric(const AggregatedValue& agg, uint32_t processingTimeUs) {
    Serial.printf("[METRIC][W.E.T.] id=%u samples=%lu duration_ms=%.2f processing_us=%lu\n",
                  agg.window_id,
                  static_cast<unsigned long>(agg.sample_count),
                  agg.duration_ms,
                  static_cast<unsigned long>(processingTimeUs));
}

void resetAggregationWindow(AggregationState& state) {
    state.sum = 0.0f;
    state.sampleCount = 0;
    state.windowStartUs = 0;
    state.processingOverheadUs = 0;
}
