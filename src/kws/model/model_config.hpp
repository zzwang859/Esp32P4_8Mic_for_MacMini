#pragma once

#include "kws_port.h"
#include "sdkconfig.h"

namespace kws_model {
constexpr int sample_rate = 16000;
constexpr unsigned warmup_chunks = 100;
constexpr unsigned stable_chunks = CONFIG_KWS_FILTER_STABLE_FRAMES;
#if CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES
constexpr bool need_vad_checked = true;
#else
constexpr bool need_vad_checked = false;
#endif
constexpr float keyword_min_probability = CONFIG_KWS_FILTER_KEYWORD_THRESHOLD_PERCENT / 100.0f;
constexpr int64_t cooldown_us = int64_t(CONFIG_KWS_FILTER_COOLDOWN_MS) * 1000;
struct FilterConfig {
    unsigned threshold_percent;
    unsigned cooldown_ms;
    unsigned stable_frames;
    bool need_vad_checked;
};
constexpr FilterConfig default_filter = {
    CONFIG_KWS_FILTER_KEYWORD_THRESHOLD_PERCENT, CONFIG_KWS_FILTER_COOLDOWN_MS,
    CONFIG_KWS_FILTER_STABLE_FRAMES, need_vad_checked};
constexpr const char *feature_input = "streaming/speech_features/normalizer_1/truediv:0";
constexpr const char *logits_output = "onnx_logits:0";
constexpr const char *state_inputs[] = {
    "streaming/stream/stream/ExternalState:0",
    "streaming/stream_1/stream_1/ExternalState:0",
    "streaming/stream_2/stream_2/ExternalState:0",
    "streaming/stream_3/stream_3/ExternalState:0",
    "streaming/stream_4/stream_4/ExternalState:0",
    "streaming/stream_6/stream_6/ExternalState:0",
};
constexpr const char *state_outputs[] = {
    "onnx_next_state_1:0", "onnx_next_state_2:0", "onnx_next_state_3:0",
    "onnx_next_state_4:0", "onnx_next_state_5:0", "onnx_next_state_7:0",
};
}  // namespace kws_model
