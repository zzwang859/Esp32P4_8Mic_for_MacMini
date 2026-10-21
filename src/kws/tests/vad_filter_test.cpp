#include <cassert>
#include <vector>
#include "kws_stream_state.hpp"

namespace {
int64_t now_us = 0;
int step(kws_stream::Debouncer &filter, int index, bool vad_high) {
    float logits[KWS_MAX_CLASS_COUNT]{};
    logits[index] = 10;
    now_us += 10000;
    int result = filter.update(index, logits, now_us, index == 33 ? 34 : KWS_CLASS_COUNT, vad_high);
    if (result >= 0) filter.acknowledge(true, now_us);
    return result;
}
void prime(kws_stream::Debouncer &filter) {
    for (unsigned i = 1; i < kws_model::warmup_chunks; ++i) assert(step(filter, 0, false) == -1);
    assert(step(filter, 0, false) == 0); // Background confirmation never requires VAD.
}
}

int main() {
    static_assert(kws_model::stable_chunks == 3);
    static_assert(kws_model::need_vad_checked == bool(CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES));
    kws_stream::Framer framer;
    int16_t pcm[160]{};
    std::vector<kws_stream::Block> blocks;
    auto send = [&](const kws_stream::Block &b) { blocks.push_back(b); };
    framer.feed(pcm, 159, send, true);
    framer.feed(pcm, 1, send, false);
    assert(blocks.size() == 1 && !blocks.back().vad_high);
    framer.feed(pcm, 1, send, false);
    framer.feed(pcm, 159, send, true);
    assert(!blocks.back().vad_high); // A partial block cannot lose its low-VAD prefix.
    framer.feed(pcm, 80, send, true);
    framer.feed(pcm, 80, send, true);
    assert(blocks.back().vad_high);
    framer.feed(pcm, 159, send, false);
    framer.discontinuity();
    framer.feed(pcm, 160, send, true);
    assert(blocks.back().vad_high && blocks.back().sequence == 4);
    framer.feed(pcm, 160, send);
    assert(!blocks.back().vad_high); // Missing VAD fails closed only in Yes mode.

    kws_stream::Debouncer filter;
    prime(filter);
    std::vector<int> fired_at;
    const bool vad[] = {true, true, false, true, true, true};
    for (int i = 0; i < 6; ++i)
        if (step(filter, 2, vad[i]) == 2) fired_at.push_back(i);
    if (kws_model::need_vad_checked) assert((fired_at == std::vector<int>{5}));
    else assert((fired_at == std::vector<int>{2}));

    // VAD low alone cannot rearm a keyword that already fired.
    for (int i = 0; i < 60; ++i) assert(step(filter, 2, false) == -1);
    for (int i = 0; i < 5; ++i) assert(step(filter, 2, true) == -1);
    for (int i = 0; i < 2; ++i) assert(step(filter, 1, false) == -1);
    assert(step(filter, 1, false) == 1);
    for (int i = 0; i < 2; ++i) assert(step(filter, 2, true) == -1);
    assert(step(filter, 2, true) == 2);

    // Silence VAD during startup does not prolong the fixed warmup.
    kws_stream::Debouncer startup;
    int low_vad_fires = 0;
    for (int i = 0; i < 120; ++i) low_vad_fires += step(startup, 3, false) == 3;
    assert(low_vad_fires == (kws_model::need_vad_checked ? 0 : 1));
    if (kws_model::need_vad_checked) {
        assert(step(startup, 3, true) == -1);
        assert(step(startup, 3, true) == -1);
        assert(step(startup, 3, true) == 3);
    }

    // The same policy applies to downloaded models with more than eight classes.
    kws_stream::Debouncer dynamic;
    prime(dynamic);
    assert(step(dynamic, 33, false) == -1);
    assert(step(dynamic, 33, false) == -1);
    assert(step(dynamic, 33, false) == (kws_model::need_vad_checked ? -1 : 33));
    if (kws_model::need_vad_checked) {
        assert(step(dynamic, 33, true) == -1);
        assert(step(dynamic, 33, true) == -1);
        assert(step(dynamic, 33, true) == 33);
    }

    // A failed action enqueue must still require VAD on the retry frame.
    kws_stream::Debouncer retry;
    prime(retry);
    float logits[KWS_CLASS_COUNT]{};
    logits[4] = 10;
    for (int i = 0; i < 2; ++i) assert(retry.update(4, logits, now_us, 8, true) == -1);
    assert(retry.update(4, logits, now_us, 8, true) == 4);
    retry.acknowledge(false, now_us);
    assert(retry.update(4, logits, now_us, 8, false) == (kws_model::need_vad_checked ? -1 : 4));
    if (kws_model::need_vad_checked) {
        assert(retry.update(4, logits, now_us, 8, true) == -1);
        assert(retry.update(4, logits, now_us, 8, true) == -1);
        assert(retry.update(4, logits, now_us, 8, true) == 4);
    }
}
