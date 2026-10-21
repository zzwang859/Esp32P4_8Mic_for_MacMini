#include <cassert>
#include <cmath>
#include "kws_stream_state.hpp"

int main() {
    static_assert(kws_model::stable_chunks == 5);
    static_assert(kws_model::keyword_min_probability == 0.5f);
    kws_stream::Debouncer filter;
    float logits[KWS_CLASS_COUNT]{};
    logits[0] = 4;
    for (unsigned i = 0; i < 99; ++i) assert(filter.update(0, logits, 0) == -1);
    assert(filter.update(0, logits, 0) == 0);
    filter.acknowledge(true, 1000);

    // An exact 50% probability meets the configured threshold inclusively.
    for (float &v : logits) v = -1000;
    logits[2] = logits[7] = 0;
    for (unsigned i = 0; i < 4; ++i) assert(filter.update(2, logits, 1000) == -1);
    assert(filter.update(2, logits, 1000) == 2);
    filter.acknowledge(true, 1000);

    logits[2] = -1000;
    logits[3] = 0;
    const int64_t deadline = 1000 + kws_model::cooldown_us;
    if (kws_model::cooldown_us > 0)
        for (unsigned i = 0; i < 20; ++i) assert(filter.update(3, logits, deadline - 1) == -1);
    // Cooldown uses microseconds, not rounded 10-ms frame counts. Zero permits
    // immediate new confirmation; 35 ms expires exactly on its own boundary.
    for (unsigned i = 0; i < 4; ++i) assert(filter.update(3, logits, deadline) == -1);
    assert(filter.update(3, logits, deadline) == 3);

    kws_stream::Debouncer runtime;
    kws_model::FilterConfig live{0, 10, 1, true};
    float background[KWS_CLASS_COUNT]{};
    background[0] = 10;
    for (unsigned i = 0; i < 100; ++i) runtime.update(0, background, 0, 8, false, live);
    runtime.config_changed(); // A live edit does not start another 1-second warmup.
    float equal_logits[KWS_CLASS_COUNT]{};
    assert(runtime.update(2, equal_logits, 100000, 8, false, live) == -1);
    assert(runtime.update(2, equal_logits, 100000, 8, true, live) == 2);
    runtime.acknowledge(true, 100000);
    runtime.config_changed(); // A live edit cannot bypass the active cooldown.
    assert(runtime.update(3, equal_logits, 109999, 8, true, live) == -1);
    assert(runtime.update(3, equal_logits, 110000, 8, true, live) == 3);
    filter.acknowledge(false, deadline);
    assert(filter.update(3, logits, deadline) == 3);
}
