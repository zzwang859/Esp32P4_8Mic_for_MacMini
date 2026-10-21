"""Run the actual post-AFE queue/NS task on the host, linked to the C algorithm."""
import argparse
import itertools
from pathlib import Path
import subprocess
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/usb_driver/tests"))
from test_capture2_stereo import function

MOCKS = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
#include "audio_pipeline.h"
#include "post_ns_stream.hpp"
#include "nscore/nscore.h"
using namespace array_dsp;
constexpr int pdTRUE = 1, pdFALSE = 0, ns_queue = 1;
constexpr unsigned CUSTOM_WORK_NS=8;
void custom_worker_register(unsigned) {}
bool custom_worker_checkpoint(unsigned) { return false; }
bool custom_maintenance_active() { return false; }
std::deque<PostNsInput> queue;
PostNsInput producer;
PostNsInput *ns_producer = &producer;
uint32_t ns_sequence = 0;
std::atomic<uint32_t> ns_frames{0}, ns_dropped_samples{0}, ns_resets{0};
std::atomic<uint64_t> ns_process_us{0};
std::atomic<int64_t> ns_worst_us{0};
int16_t doa = -18000;
int16_t audio_pipeline_get_doa_sample() { return doa; }
int64_t clock_us = 0;
int64_t esp_timer_get_time() { return clock_us += 10; }
void update_worst(std::atomic<int64_t> &worst, int64_t value) { if (value > worst) worst = value; }
int pdMS_TO_TICKS(int value) { return value; }
void vTaskDelay(int value) { assert(value == 1); }
unsigned resets = 0;
std::vector<int16_t> kws_pcm, usb_pcm;
std::vector<bool> kws_vad;
std::vector<PostNsMetadata> usb_meta, original_meta;
std::vector<int16_t> original_pcm;
void kws_stream_feed(const int16_t *pcm, size_t count) { kws_pcm.insert(kws_pcm.end(), pcm, pcm + count); }
void kws_stream_feed_with_vad(const int16_t *pcm, size_t count, bool high) {
    assert(count==PostNsFramer::frame_samples);
    kws_vad.insert(kws_vad.end(),count,high);
    kws_stream_feed(pcm,count);
}
void kws_stream_discontinuity() { ++resets; }
void publish_capture2(const int16_t *pcm, size_t count, audio_pipeline_vad_state_t, int16_t,
                      const PostNsMetadata *metadata) {
    usb_pcm.insert(usb_pcm.end(), pcm, pcm + count);
    usb_meta.insert(usb_meta.end(), metadata, metadata + count);
}
int xQueueSend(int handle, const PostNsInput *block, int ticks) {
    assert(handle == ns_queue && ticks == 0);
    if (queue.size() == 2) return pdFALSE; // Force bounded-queue overload.
    queue.push_back(*block);
    return pdTRUE;
}
struct Done {};
int phase = 0;
int xQueueReceive(int handle, PostNsInput *block, int ticks);
'''

CASES = r'''
int xQueueReceive(int handle, PostNsInput *block, int ticks) {
    assert(handle == ns_queue && ticks == 10);
    if (queue.empty()) {
        if (phase == 3) throw Done{};
        std::vector<int16_t> input(512);
        for (int i = 0; i < 512; ++i) input[i] = int16_t((i * 173 + phase * 211) % 12000 - 6000);
        if (phase == 2) post_ns_discontinuity(); // AFE fetch failure.
        const int accepted = phase == 2 ? 1 : 2;
        for (int n = 0; n < accepted; ++n) {
            doa = int16_t(phase * 4000 + n * 1000 - 9000);
            const auto vad = n ? AUDIO_PIPELINE_VAD_SPEECH : AUDIO_PIPELINE_VAD_SILENCE;
            post_ns_submit(input.data(), input.size(), vad);
            original_pcm.insert(original_pcm.end(), input.begin(), input.end());
            original_meta.insert(original_meta.end(), input.size(), PostNsMetadata{doa, int8_t(vad)});
        }
        if (phase == 0) post_ns_submit(input.data(), input.size(), AUDIO_PIPELINE_VAD_SPEECH); // Rejected.
        ++phase;
    }
    *block = queue.front(); queue.pop_front();
    // Neither later input nor newer global DOA may overwrite queued metadata.
    doa = 17000;
    return pdTRUE;
}
int main() {
    try { post_ns_task(nullptr); } catch (Done &) {}
    assert(ns_frames == 15 && ns_dropped_samples == 512 && ns_resets == 2);
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    assert(kws_pcm.empty() && resets == 0); // NS gaps cannot reset the other source.
#else
    assert(resets == 2);
    assert(kws_pcm.size() == 2400); // 960 + 960 + 480; partial frames discarded at gaps.
#endif
    const size_t lengths[] = {960, 960, 480};
    [[maybe_unused]] const size_t starts[] = {0, 1024, 2048};
    [[maybe_unused]] size_t output = 0;
    for (int segment = 0; segment < 3; ++segment) {
        for (size_t i = 0; i < lengths[segment]; ++i, ++output) {
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
            if (i % 160 == 0) {
                constexpr size_t delay = CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT == 100 ? 0 : 96;
                bool high = true;
                for (size_t j = i; j < i+160; ++j)
                    high &= j >= delay && original_meta[starts[segment] + j - delay].vad == AUDIO_PIPELINE_VAD_SPEECH;
                for (size_t j = 0; j < 160; ++j) assert(kws_vad[output+j] == high);
            }
#endif
#if CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT == 100
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
            assert(kws_pcm[output] == original_pcm[starts[segment] + i]);
#endif
#if CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT
            assert(usb_pcm[output] == original_pcm[starts[segment] + i]);
#endif
#endif
#if CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT || CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
            const size_t delay = CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT == 100 ? 0 : 96;
            const auto expected = i < delay ? PostNsMetadata{} : original_meta[starts[segment] + i - delay];
            assert(usb_meta[output].vad == expected.vad && usb_meta[output].doa == expected.doa);
#endif
        }
    }
#if CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT != 100 && !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    bool changed = false;
    for (size_t i = 0; i < 960; ++i) changed |= kws_pcm[i] != original_pcm[starts[0] + i];
    assert(changed); // KWS is fed NS output, never the original AFE stream.
#endif
#if CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT || CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
    assert(usb_pcm.size() == 2400);
#if CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
    for (size_t i = 0; i < usb_pcm.size(); ++i)
        assert(usb_pcm[i] == capture2_sine_table[i % 16]);
#else
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    assert(usb_pcm == kws_pcm);
#endif
#endif
#else
    assert(usb_pcm.empty()); // USB diagnostic sources do not receive this stream.
#endif
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", nargs="+", default=["c++"])
    parser.add_argument("--cc", nargs="+", default=["cc"])
    args = parser.parse_args()
    pipeline = (ROOT / "src/audio_processing/audio_pipeline.cpp").read_text(encoding="utf-8")
    tone = re.search(r"constexpr int16_t capture2_sine_table\[16\] = \{.*?\};", pipeline, re.S).group()
    code = MOCKS + "\n#if CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST\n" + tone + "\n#endif\n" + "\n".join(function(pipeline, name) for name in (
        "post_ns_submit", "post_ns_discontinuity", "post_ns_task")) + CASES
    with tempfile.TemporaryDirectory(prefix="post-afe-ns-") as temp:
        temp = Path(temp)
        (temp / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
        source, binary = temp / "test.cpp", temp / "test.exe"
        source.write_text(code, encoding="utf-8")
        objects = []
        for name in ("enc_core", "ns_fft"):
            obj = temp / (name + ".o")
            subprocess.run(args.cc + ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-c",
                str(ROOT / f"src/audio_processing/nscore/{name}.c"), "-o", str(obj)], check=True)
            objects.append(str(obj))
        for gain in (25, 100):
            for capture in (0, 1, 2):
                for usb_kws, vad_check in itertools.product((0, 1), repeat=2):
                    subprocess.run(args.cxx + ["-std=c++17", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror",
                        f"-DCONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT={gain}",
                        f"-DCONFIG_ARRAY_CAPTURE2_AEC_OUTPUT={int(capture == 1)}",
                        f"-DCONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST={int(capture == 2)}",
                        f"-DCONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE={usb_kws}",
                        f"-DCONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES={vad_check}",
                        "-I", str(temp), "-I", str(ROOT / "src/audio_processing"),
                        str(source), *objects, "-o", str(binary)], check=True)
                    subprocess.run([str(binary)], check=True)
                    print(f"PASS real post-AFE NS task: gain={gain}% capture={capture} usb_kws={usb_kws} vad_check={vad_check}, source isolation, queued metadata, drop/gap reset", flush=True)


if __name__ == "__main__":
    main()
