"""Check the real AFE feed task's optional KWS playback tap and source isolation."""
import argparse
import itertools
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/usb_driver/tests"))
from test_capture2_stereo import function

MOCKS = r'''
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
constexpr int afe_feed_samples = 512, afe_stream = 1;
constexpr unsigned CUSTOM_WORK_FEED=2;
void custom_worker_register(unsigned) {}
bool custom_worker_checkpoint(unsigned) { return false; }
bool custom_maintenance_active() { return false; }
int pdMS_TO_TICKS(int ticks) { return ticks; }
[[maybe_unused]] constexpr int capture2_sine_table[16] = {0};
int16_t feed_buffer[afe_feed_samples * 2];
int16_t *afe_feed_buffer = feed_buffer;
std::vector<int16_t> interleaved, expected_reference, kws_pcm;
std::vector<bool> kws_vad;
unsigned feed_calls = 0, capture_calls = 0;
int afe_worst_us = 0;
void *afe_data = nullptr;
struct Done {};
size_t position = 0;
size_t xStreamBufferReceive(int handle, void *out, size_t bytes, int ticks) {
    assert(handle == afe_stream && ticks == 10);
    if (position == interleaved.size()) throw Done{};
    // Split reads at odd sample/channel boundaries, as a byte stream may do.
    const size_t count = std::min({bytes / sizeof(int16_t), size_t(137), interleaved.size() - position});
    std::memcpy(out, interleaved.data() + position, count * sizeof(int16_t));
    position += count;
    return count * sizeof(int16_t);
}
int64_t esp_timer_get_time() { return 0; }
void update_worst(int &, int64_t) {}
[[maybe_unused]] constexpr int AUDIO_PIPELINE_VAD_SPEECH=1;
int audio_pipeline_get_vad_state() { return feed_calls == 0 ? 1 : feed_calls == 1 ? 0 : -1; }
void publish_capture2(const int16_t *, size_t count, int) {
    assert(count == afe_feed_samples);
    ++capture_calls;
}
void kws_stream_feed(const int16_t *pcm, size_t count) {
    assert(count == afe_feed_samples);
    kws_pcm.insert(kws_pcm.end(), pcm, pcm + count);
}
void kws_stream_feed_with_vad(const int16_t *pcm, size_t count, bool high) {
    kws_vad.push_back(high);
    kws_stream_feed(pcm, count);
}
struct Afe {
    void feed(void *data, int16_t *pairs) {
        assert(data == afe_data);
        const size_t start = feed_calls * afe_feed_samples * 2;
        for (int i = 0; i < afe_feed_samples * 2; ++i)
            assert(pairs[i] == interleaved[start + i]); // KWS never modifies AFE input.
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
        assert(kws_pcm.size() == (feed_calls + 1) * afe_feed_samples);
#else
        assert(kws_pcm.empty());
#endif
        // The tap must copy before calling AFE, not rely on an unchanged buffer.
        std::fill_n(pairs, afe_feed_samples * 2, int16_t(123));
        ++feed_calls;
    }
} instance;
Afe *afe = &instance;
'''

CASES = r'''
int main() {
    for (int i = 0; i < afe_feed_samples * 3; ++i) {
        const int16_t mic = int16_t(i * 37 - 28000);
        const int16_t ref = i >= afe_feed_samples ? 0 :
            i % 4 == 0 ? INT16_MIN : i % 4 == 1 ? INT16_MAX : int16_t(17000 - i * 11);
        interleaved.push_back(mic);
        interleaved.push_back(ref);
        expected_reference.push_back(ref);
    }
    try { afe_feed_task(nullptr); } catch (Done &) {}
    assert(feed_calls == 3);
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    assert(kws_pcm == expected_reference); // PCM16 extremes and stopped-playback zeros preserved.
    assert((kws_vad == std::vector<bool>{true, false, false}));
#else
    assert(kws_pcm.empty());
#endif
#if CONFIG_ARRAY_CAPTURE2_SINE_TEST || CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST
    assert(capture_calls == 3);
#else
    assert(capture_calls == 0);
#endif
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", nargs="+", default=["c++"])
    args = parser.parse_args()
    pipeline = (ROOT / "src/audio_processing/audio_pipeline.cpp").read_text(encoding="utf-8")
    code = MOCKS + function(pipeline, "afe_feed_task") + CASES
    with tempfile.TemporaryDirectory(prefix="kws-input-") as temp:
        source, binary = Path(temp) / "test.cpp", Path(temp) / "test.exe"
        source.write_text(code, encoding="utf-8")
        for usb_kws, vad_check in itertools.product((0, 1), repeat=2):
            for capture in ("AEC_OUTPUT", "SINE_TEST", "PLAYBACK_TEST"):
                subprocess.run(args.cxx + ["-std=c++17", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror",
                    f"-DCONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE={usb_kws}",
                    f"-DCONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES={vad_check}",
                    f"-DCONFIG_ARRAY_CAPTURE2_{capture}=1", str(source), "-o", str(binary)], check=True)
                subprocess.run([str(binary)], check=True)
                print(f"PASS real AFE feed: usb_kws={usb_kws} vad_check={vad_check} capture={capture}, mono tap, queued VAD, unchanged AFE input", flush=True)


if __name__ == "__main__":
    main()
