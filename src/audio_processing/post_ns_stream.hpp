#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace array_dsp {
struct PostNsMetadata {
    int16_t doa = -18000;
    int8_t vad = -1;
};
struct PostNsInput {
    uint32_t sequence;
    uint16_t samples;
    PostNsMetadata metadata;
    int16_t pcm[1024];
};

// AFE fetch lengths need not be multiples of 160 (currently 512 samples).
// Keep PCM and per-sample metadata together across framing and the NS core's
// 256-160=96 sample / 6-ms overlap-add delay. One owner: core1 NS task.
class PostNsFramer {
public:
    static constexpr size_t frame_samples = 160;
    static constexpr size_t delay_samples = 96;

    explicit PostNsFramer(bool bypass) : bypass_(bypass) { reset(); }
    void reset() {
        used_ = delay_pos_ = 0;
        history_.fill(PostNsMetadata{});
    }

    template<class Process, class Emit>
    void feed(const int16_t *pcm, size_t count, PostNsMetadata metadata,
              Process process, Emit emit) {
        while (count) {
            const size_t take = std::min(count, frame_samples - used_);
            std::memcpy(input_.data() + used_, pcm, take * sizeof(*pcm));
            std::fill_n(current_.data() + used_, take, metadata);
            used_ += take; pcm += take; count -= take;
            if (used_ != frame_samples) continue;
            process(input_.data(), output_.data());
            for (size_t i = 0; i < frame_samples; ++i) {
                if (bypass_) delayed_[i] = current_[i];
                else {
                    delayed_[i] = history_[delay_pos_];
                    history_[delay_pos_] = current_[i];
                    delay_pos_ = (delay_pos_ + 1) % delay_samples;
                }
            }
            emit(output_.data(), delayed_.data(), frame_samples);
            used_ = 0;
        }
    }

private:
    bool bypass_;
    size_t used_ = 0, delay_pos_ = 0;
    std::array<int16_t, frame_samples> input_{}, output_{};
    std::array<PostNsMetadata, frame_samples> current_{}, delayed_{};
    std::array<PostNsMetadata, delay_samples> history_{};
};
} // namespace array_dsp
