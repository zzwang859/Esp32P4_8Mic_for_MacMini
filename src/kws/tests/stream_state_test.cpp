#include <cassert>
#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>
#include "kws_stream_state.hpp"
#include "kws_transpose_copy.h"

namespace {
int64_t now_us = 0;
int step(kws_stream::Debouncer &filter, int label, float strength = 4.0f) {
    float logits[KWS_CLASS_COUNT]{};
    if (label >= 0 && label < KWS_CLASS_COUNT) logits[label] = strength;
    now_us += 10000;
    return filter.update(label, logits, now_us);
}
int predict(kws_stream::Debouncer &filter, int label, float strength = 4.0f) {
    const int detected = step(filter, label, strength);
    if (detected >= 0) filter.acknowledge(true, now_us);
    return detected;
}
void expect_none(kws_stream::Debouncer &filter, int label, unsigned chunks, float strength = 4.0f) {
    for (unsigned i = 0; i < chunks; ++i) assert(predict(filter, label, strength) == -1);
}
void prime(kws_stream::Debouncer &filter) {
    expect_none(filter, 0, 99);
    assert(predict(filter, 0) == 0);
}
void fire(kws_stream::Debouncer &filter, int label) {
    expect_none(filter, label, kws_model::stable_chunks - 1);
    assert(predict(filter, label) == label);
}
}

int main() {
    using namespace kws_stream;
    Framer framer;
    std::vector<Block> blocks;
    std::vector<int16_t> pcm(512 * 10);
    for (size_t i = 0; i < pcm.size(); ++i) pcm[i] = int16_t(i - 2560);
    auto send = [&](const Block &b) { blocks.push_back(b); };
    for (size_t i = 0; i < pcm.size(); i += 512) framer.feed(pcm.data() + i, 512, send);
    assert(blocks.size() == 32);
    for (size_t i = 0; i < blocks.size(); ++i) {
        assert(blocks[i].sequence == i);
        for (size_t j = 0; j < KWS_CHUNK_SAMPLES; ++j)
            assert(blocks[i].pcm[j] == pcm[i * KWS_CHUNK_SAMPLES + j]);
    }
    // A full consumer queue may reject one block. The next accepted block
    // exposes exactly one missing sequence without modifying the older queue.
    framer.feed(pcm.data(), 160, [](const Block &) {});
    framer.feed(pcm.data() + 160, 160, send);
    assert(blocks.back().sequence == 33 && blocks.back().pcm[0] == pcm[160]);
    // Discard a partial chunk at an AFE gap; never join samples across it.
    framer.feed(pcm.data(), 159, send);
    framer.discontinuity();
    framer.feed(pcm.data() + 160, 160, send);
    assert(blocks.size() == 34 && blocks.back().sequence == 35);
    for (size_t j = 0; j < 160; ++j) assert(blocks.back().pcm[j] == pcm[160 + j]);

    static_assert(kws_model::stable_chunks == 3 && kws_model::cooldown_us == 500000);
    // A qualified word at startup confirms at the 100th valid frame.
    Debouncer startup;
    expect_none(startup, 2, 99);
    assert(predict(startup, 2) == 2);
    expect_none(startup, 2, 1000); // A held word is still deduplicated.

    // All six keywords remain enabled; stable background permits the same
    // word again, and background confirmations never start a cooldown.
    Debouncer all;
    prime(all);
    for (int word : {2, 3, 4, 5, 6, 7, 7}) {
        fire(all, word);
        expect_none(all, 0, 2);
        assert(predict(all, 0) == 0);
        expect_none(all, 0, 50);
    }

    // Global cooldown suppresses DIFFERENT words as well. No candidate frames
    // may accumulate inside it: the exact expiry frame starts a fresh run.
    Debouncer cooldown;
    prime(cooldown);
    fire(cooldown, 2);
    expect_none(cooldown, 3, 49);
    expect_none(cooldown, 3, 2); // expiry at frame 50, then second confirmation
    assert(predict(cooldown, 3) == 3);

    // Same-word rearm through background is allowed during cooldown, but
    // cannot shorten the cooldown itself.
    const int64_t deadline = now_us + kws_model::cooldown_us;
    expect_none(cooldown, 1, 2);
    assert(predict(cooldown, 1) == 1);
    while (now_us + 10000 < deadline) expect_none(cooldown, 3, 1);
    fire(cooldown, 3);

    // Below-threshold or changed top-1 frames break a partial confirmation.
    Debouncer confidence;
    prime(confidence);
    expect_none(confidence, 6, 2);
    expect_none(confidence, 6, 1, 0.001f);
    expect_none(confidence, 6, 2);
    expect_none(confidence, 7, 1);
    fire(confidence, 6);

    // Independent boundary checks around p=0.80, with a large common logit
    // offset: softmax must remain shift invariant and never overflow.
    for (float offset : {0.0f, 1000.0f, -1000.0f}) {
        Debouncer threshold;
        prime(threshold);
        float logits[KWS_CLASS_COUNT];
        for (float &v : logits) v = offset;
        logits[2] = offset + std::log(27.9f); // 27.9/(27.9+7) < .80
        for (int i = 0; i < 10; ++i) assert(threshold.update(2, logits, now_us) == -1);
        logits[2] = offset + std::log(28.1f); // 28.1/(28.1+7) > .80
        for (int i = 0; i < 2; ++i) assert(threshold.update(2, logits, now_us) == -1);
        assert(threshold.update(2, logits, now_us) == 2);
        threshold.acknowledge(true, now_us);
    }

    // Invalid/nonfinite input rejects a candidate; a forged non-top index is
    // rejected too. All eight classes participate in the probability sum.
    Debouncer invalid;
    prime(invalid);
    float logits[KWS_CLASS_COUNT]{};
    logits[2] = 4;
    expect_none(invalid, 2, 2);
    assert(invalid.update(2, nullptr, now_us) == -1);
    expect_none(invalid, 2, 2);
    assert(invalid.update(-1, logits, now_us) == -1);
    expect_none(invalid, 2, 2);
    logits[4] = INFINITY;
    assert(invalid.update(2, logits, now_us) == -1);
    logits[4] = NAN;
    assert(invalid.update(2, logits, now_us) == -1);
    logits[4] = 5;
    assert(invalid.update(2, logits, now_us) == -1);
    fire(invalid, 2);

    // Failed output submissions neither commit dedup nor start cooldown.
    Debouncer retry;
    prime(retry);
    expect_none(retry, 2, 2);
    assert(step(retry, 2) == 2);
    retry.acknowledge(false, now_us);
    assert(step(retry, 2) == 2);
    retry.acknowledge(false, now_us);
    assert(step(retry, 2) == 2);
    retry.acknowledge(true, now_us);
    expect_none(retry, 3, 49);
    fire(retry, 3);

    // Resetting audio/model state cannot bypass an unexpired cooldown.
    const int64_t reset_time = now_us;
    retry.reset();
    for (int i = 0; i < 100; ++i) assert(retry.update(2, logits, reset_time) == -1);
    logits[4] = 0;
    for (int i = 0; i < 100; ++i) assert(retry.update(2, logits, reset_time) == -1);
    for (int i = 0; i < 2; ++i)
        assert(retry.update(2, logits, reset_time + kws_model::cooldown_us) == -1);
    assert(retry.update(2, logits, reset_time + kws_model::cooldown_us) == 2);

    // Validate the migrated optimized transpose, including the 98-frame state
    // and odd/tile-edge dimensions. Check guards on every destination.
    for (size_t rows : std::array<size_t, 8>{1, 2, 7, 15, 16, 17, 40, 98}) {
        for (size_t cols : std::array<size_t, 8>{1, 2, 3, 15, 16, 17, 40, 98}) {
            const size_t n = rows * cols;
            std::vector<uint8_t> src8(n), dst8(n + 2, 0xa5);
            std::vector<uint16_t> src16(n), dst16(n + 2, 0xbeef);
            for (size_t i = 0; i < n; ++i) { src8[i] = uint8_t(i); src16[i] = uint16_t(i * 31); }
            kws_transpose_u8(dst8.data() + 1, src8.data(), rows, cols);
            kws_transpose_u16(dst16.data() + 1, src16.data(), rows, cols);
            assert(dst8.front() == 0xa5 && dst8.back() == 0xa5);
            assert(dst16.front() == 0xbeef && dst16.back() == 0xbeef);
            for (size_t r = 0; r < rows; ++r) for (size_t c = 0; c < cols; ++c) {
                assert(dst8[1 + c * rows + r] == src8[r * cols + c]);
                assert(dst16[1 + c * rows + r] == src16[r * cols + c]);
            }
        }
    }
    std::puts("PASS: AFE framing, loss/gap sequences, probability confirmation/cooldown/queue retry, INT8/INT16 transpose");
}
