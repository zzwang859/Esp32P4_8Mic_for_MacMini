#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace array_dsp {
// Owned by the DSP task. VAD gates motion, including motion toward a previous
// target. No elapsed time is saved up during silence or a processing stall.
class DoaTracking {
public:
    DoaTracking(float degrees_per_second, float max_step_seconds, float initial_degrees = 0)
        : speed_(degrees_per_second), max_step_seconds_(max_step_seconds),
          degrees_(normalize(initial_degrees)) {}

    float update(float target, bool speech, int64_t now_us) {
        const int64_t elapsed_us = now_us - last_update_us_;
        last_update_us_ = now_us;
        const bool advance = speech && was_speech_;
        was_speech_ = speech;
        if (!advance || elapsed_us <= 0 || !std::isfinite(target)) return degrees_;
        target = normalize(target);
        float delta = target - degrees_;
        if (delta > 180) delta -= 360;
        if (delta <= -180) delta += 360; // Equal paths: use increasing angles.
        const float seconds = std::min(float(elapsed_us) / 1000000.0f, max_step_seconds_);
        const float step = speed_ * seconds;
        if (std::abs(delta) <= step) degrees_ = target;
        else degrees_ = normalize(degrees_ + std::copysign(step, delta));
        return degrees_;
    }

private:
    static float normalize(float degrees) {
        float value = std::fmod(degrees, 360.0f);
        return value < 0 ? value + 360.0f : value;
    }
    float speed_, max_step_seconds_, degrees_;
    int64_t last_update_us_ = 0;
    bool was_speech_ = false;
};
} // namespace array_dsp
