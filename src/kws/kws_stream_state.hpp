#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include "model_config.hpp"

namespace kws_stream {
struct Block {
    uint32_t sequence;
    int16_t pcm[KWS_CHUNK_SAMPLES];
    bool vad_high;
};

// Owned only by the configured input producer. Rejected blocks consume a sequence,
// so the consumer resets exactly at a gap, after processing older queued audio.
class Framer {
public:
    template <typename Send> void feed(const int16_t *pcm, size_t count, Send send, bool vad_high = false) {
        while (count) {
            if (used_ == 0) block_.vad_high = true;
            block_.vad_high = block_.vad_high && vad_high;
            const size_t n = std::min(count, KWS_CHUNK_SAMPLES - used_);
            std::memcpy(block_.pcm + used_, pcm, n * sizeof(*pcm));
            used_ += n;
            pcm += n;
            count -= n;
            if (used_ == KWS_CHUNK_SAMPLES) {
                send(block_);
                ++block_.sequence;
                used_ = 0;
            }
        }
    }
    void discontinuity() { used_ = 0; ++block_.sequence; }
private:
    Block block_{};
    size_t used_ = 0;
};

class Debouncer {
public:
    void reset() {
        // Audio gaps restart warmup, but must not bypass a running cooldown.
        processed_ = stable_ = 0;
        candidate_ = last_reported_ = -1;
    }
    void config_changed() { reject_candidate(); }
    int update(int index, const float logits[KWS_CLASS_COUNT], int64_t now_us,
               int class_count = KWS_CLASS_COUNT, bool vad_high = false,
               kws_model::FilterConfig filter = kws_model::default_filter) {
        filter_ = filter;
        if (class_count < 2 || class_count > KWS_MAX_CLASS_COUNT) return reject_candidate();
        if (!logits || index < 0 || index >= class_count || !std::isfinite(logits[index]))
            return reject_candidate();
        const float peak = logits[index];
        for (int i = 0; i < class_count; ++i)
            if (!std::isfinite(logits[i]) || logits[i] > peak) return reject_candidate();
        if (processed_ < kws_model::warmup_chunks) ++processed_;
        if (index >= 2) {
            if (filter.need_vad_checked && !vad_high)
                return reject_candidate();
            if (now_us < cooldown_until_us_ || index == last_reported_)
                return reject_candidate();
            // Stable softmax; only the top-1 probability is needed. Keep raw
            // logits unchanged for diagnostics, including when tracing is off.
            float sum = 0;
            for (int i = 0; i < class_count; ++i) sum += std::exp(logits[i] - peak);
            if (1.0f / sum < filter.threshold_percent / 100.0f) return reject_candidate();
        }
        if (candidate_ == index) {
            if (stable_ < filter.stable_frames) ++stable_;
        } else {
            candidate_ = index;
            stable_ = 1;
        }
        if (processed_ >= kws_model::warmup_chunks && stable_ >= filter.stable_frames &&
            candidate_ != last_reported_) return candidate_;
        return -1;
    }
    // A failed result enqueue must neither start cooldown nor commit dedup.
    void acknowledge(bool accepted, int64_t now_us) {
        if (accepted) {
            last_reported_ = candidate_;
            if (candidate_ >= 2) {
                cooldown_until_us_ = now_us + int64_t(filter_.cooldown_ms) * 1000;
                reject_candidate();
            }
        }
        else stable_ = filter_.stable_frames - 1;
    }
private:
    int reject_candidate() { candidate_ = -1; stable_ = 0; return -1; }
    static_assert(kws_model::stable_chunks > 0);
    static_assert(kws_model::keyword_min_probability >= 0 && kws_model::keyword_min_probability <= 1);
    static_assert(kws_model::cooldown_us >= 0);
    unsigned processed_ = 0, stable_ = 0;
    int candidate_ = -1, last_reported_ = -1;
    int64_t cooldown_until_us_ = 0;
    kws_model::FilterConfig filter_ = kws_model::default_filter;
};
}  // namespace kws_stream
