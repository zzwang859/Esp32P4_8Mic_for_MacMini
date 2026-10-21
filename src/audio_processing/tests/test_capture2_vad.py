"""Exercise real Capture2 publication, FIFO reads and AFE/VAD pairing on the host."""

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/usb_driver/tests"))
from test_capture2_stereo import function


MOCKS = r"""
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "audio_pipeline.h"
#include "post_ns_stream.hpp"
using array_dsp::PostNsMetadata;
#define CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT 1
#define pdMS_TO_TICKS(ms) (ms)
#define portENTER_CRITICAL(...) ((void)0)
#define portEXIT_CRITICAL(...) ((void)0)
#define portENTER_CRITICAL_ISR(...) ((void)0)
#define portEXIT_CRITICAL_ISR(...) ((void)0)
#define portYIELD_FROM_ISR() (++yields)
#define ESP_RETURN_ON_FALSE(condition, error, ...) do { if (!(condition)) return error; } while (0)
#define ESP_RETURN_ON_ERROR(expression, ...) do { int error = (expression); if (error) return error; } while (0)
using BaseType_t = int;
constexpr int pdFALSE = 0, ESP_FAIL = -1, VAD_SPEECH = 1;
constexpr int ESP_OK = 0, ESP_ERR_NOT_FOUND = 1, ESP_ERR_INVALID_STATE = 2, ESP_ERR_NO_MEM = 3;
constexpr int afe_working_frames = 4;
constexpr int ESP_PARTITION_TYPE_DATA = 1, ESP_PARTITION_SUBTYPE_ANY = 255;
constexpr const char *ESP_VADN_PREFIX = "vadnet";
struct esp_partition_t { size_t size = 40; };
struct srmodel_list_t {};
esp_partition_t model_partition;
srmodel_list_t model_list, *afe_models;
void *afe_model_blob;
constexpr int MALLOC_CAP_SPIRAM = 1, MALLOC_CAP_8BIT = 2;
uint8_t model_copy[40];
void *heap_caps_aligned_alloc(size_t alignment, size_t size, int caps) {
    assert(alignment == 16 && size == sizeof(model_copy));
    assert(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return model_copy;
}
bool custom_maintenance_active() { return false; }
constexpr int CUSTOM_WORK_FETCH = 1;
void custom_worker_register(int worker) { assert(worker == CUSTOM_WORK_FETCH); }
void custom_worker_checkpoint(int worker) { assert(worker == CUSTOM_WORK_FETCH); }
char medium_name[] = "vadnet1_medium", other_name[] = "WebRTC", *afe_vad_model;
char *filtered_model = medium_name;
std::vector<uint8_t> model_header(40, 0xff);
bool partition_present = false, model_init_fails = false;
int partition_error = 0;
unsigned model_init_calls = 0;
const esp_partition_t *esp_partition_find_first(int type, int subtype, const char *label) {
    assert(type == ESP_PARTITION_TYPE_DATA && subtype == ESP_PARTITION_SUBTYPE_ANY);
    assert(std::strcmp(label, "model") == 0);
    return partition_present ? &model_partition : nullptr;
}
int esp_partition_read(const esp_partition_t *part, size_t offset, void *header, size_t size) {
    assert(part == &model_partition && offset == 0 && size == model_header.size());
    if (partition_error) return partition_error;
    std::memcpy(header, model_header.data(), size);
    return ESP_OK;
}
srmodel_list_t *srmodel_load(void *blob) {
    assert(blob == model_copy && std::memcmp(blob, model_header.data(), model_header.size()) == 0);
    ++model_init_calls;
    return model_init_fails ? nullptr : &model_list;
}
char *esp_srmodel_filter(srmodel_list_t *models, const char *prefix, const char *suffix) {
    assert(models == &model_list && std::strcmp(prefix, "vadnet") == 0);
    assert(std::strcmp(suffix, "medium") == 0);
    return filtered_model;
}
std::atomic<bool> initialized{true}, recording{false};
std::atomic<float> processed_level_dbfs{0};
std::atomic<uint32_t> processed_level_tick{0}, output_waits{0};
std::atomic<uint32_t> capture2_queued_frames{0}, capture2_queued_zero_pairs{0};
constexpr int output_stream = 1;
audio_pipeline_capture2_frame_t scratch[4];
audio_pipeline_capture2_frame_t *capture2_scratch = scratch;
size_t capture2_scratch_frames = 4;
std::vector<uint8_t> fifo;
std::vector<int16_t> metered, kws_audio;
unsigned failures = 0, delays = 0, isr_reads = 0, yields = 0, gaps = 0;
bool in_isr = false, stop_on_delay = false;
audio_pipeline_vad_state_t latest_vad = AUDIO_PIPELINE_VAD_UNKNOWN;
int16_t current_doa = -18000;
int16_t audio_pipeline_get_doa_sample() { return current_doa; }
struct NsQueued { std::vector<int16_t> pcm; audio_pipeline_vad_state_t vad; int16_t doa; };
std::vector<NsQueued> ns_queued;
void post_ns_submit(const int16_t *samples, size_t count, audio_pipeline_vad_state_t state) {
    ns_queued.push_back({std::vector<int16_t>(samples, samples + count), state, current_doa});
}
void post_ns_discontinuity() { ++gaps; }
int xTaskGetTickCount() { return 42; }
bool xPortInIsrContext() { return in_isr; }
float pcm_rms_dbfs(const int16_t *samples, size_t count) {
    metered.assign(samples, samples + count);
    return -17.0f;
}
bool send_whole(int stream, const void *data, size_t bytes) {
    assert(stream == output_stream && bytes % sizeof(audio_pipeline_capture2_frame_t) == 0);
    if (failures) { --failures; return false; }
    if (fifo.size() + bytes > 8192) return false;
    const auto *p = static_cast<const uint8_t *>(data);
    fifo.insert(fifo.end(), p, p + bytes);
    return true;
}
void vTaskDelay(int ticks) {
    assert(ticks == 1); ++delays;
    current_doa += 1000; // A newer steering angle must not relabel a queued block.
    if (stop_on_delay) recording.store(false);
}
size_t xStreamBufferReceive(int stream, void *data, size_t bytes, int ticks) {
    assert(stream == output_stream && ticks == 0);
    bytes = std::min(bytes, fifo.size());
    std::memcpy(data, fifo.data(), bytes);
    fifo.erase(fifo.begin(), fifo.begin() + bytes);
    return bytes;
}
size_t xStreamBufferReceiveFromISR(int stream, void *data, size_t bytes, int *wake) {
    ++isr_reads; *wake = 1;
    return xStreamBufferReceive(stream, data, bytes, 0);
}
size_t xStreamBufferBytesAvailable(int stream) {
    assert(stream == output_stream); return fifo.size();
}
void publish_vad_state(audio_pipeline_vad_state_t state) { latest_vad = state; }
void kws_stream_discontinuity() { ++gaps; }
void kws_stream_feed(const int16_t *audio, size_t count) {
    kws_audio.insert(kws_audio.end(), audio, audio + count);
    // Simulate a newer status publication before the Capture2 producer runs.
    // Its marker must still come from this fetch result, not this snapshot.
    latest_vad = latest_vad == AUDIO_PIPELINE_VAD_SPEECH
               ? AUDIO_PIPELINE_VAD_SILENCE : AUDIO_PIPELINE_VAD_SPEECH;
    current_doa = kws_audio.size() == 3 ? -4500 : 4500;
}
struct Result { int16_t *data; int data_size; int ret_value; int vad_state; };
using afe_fetch_result_t = Result;
struct Done {};
std::vector<Result> results;
size_t result_index = 0;
struct Afe {
    Result *fetch_with_delay(void *, int ticks) {
        assert(ticks == 100);
        if (result_index == results.size()) throw Done{};
        current_doa = result_index == 0 ? -4500 : 4500;
        return &results[result_index++];
    }
} instance;
Afe *afe = &instance;
void *afe_data = nullptr;
"""

CASES = r"""
int main() {
    static_assert(sizeof(audio_pipeline_capture2_frame_t) == 4);
    assert(load_vadnet_model() == ESP_ERR_NOT_FOUND);
    partition_present = true;
    assert(load_vadnet_model() == ESP_ERR_INVALID_STATE); // Erased model flash.
    std::fill(model_header.begin(), model_header.end(), 0);
    assert(load_vadnet_model() == ESP_ERR_INVALID_STATE); // Empty model pack.
    auto valid_header = [] {
        std::fill(model_header.begin(), model_header.end(), 0);
        model_header[0] = 1;
        std::memcpy(model_header.data() + 4, medium_name, sizeof(medium_name));
        model_header[36] = 3;
    };
    valid_header(); model_header[4] = 'X';
    assert(load_vadnet_model() == ESP_ERR_INVALID_STATE);
    valid_header(); model_header[36] = 255;
    assert(load_vadnet_model() == ESP_ERR_INVALID_STATE);
    valid_header(); partition_error = ESP_FAIL;
    assert(load_vadnet_model() == ESP_FAIL);
    assert(model_init_calls == 0); // Never parse a missing, erased or wrong model.
    partition_error = 0; model_init_fails = true;
    assert(load_vadnet_model() == ESP_FAIL);
    model_init_fails = false; filtered_model = nullptr;
    assert(load_vadnet_model() == ESP_ERR_NOT_FOUND);
    filtered_model = other_name;
    assert(load_vadnet_model() == ESP_ERR_NOT_FOUND); // No silent WebRTC fallback.
    filtered_model = medium_name;
    assert(load_vadnet_model() == ESP_OK && afe_models == &model_list && afe_vad_model == medium_name);
    assert(afe_required_ringbuf_frames(16000, 512, 128, 128) == 12);
    assert(afe_required_ringbuf_frames(16000, 512, 180, 128) == 14);
    assert(afe_required_ringbuf_frames(16000, 512, 0, 0) == 4);
    assert(afe_required_ringbuf_frames(16000, 512, 128, 0) == 8);
    assert(afe_required_ringbuf_frames(16000, 512, 128, 129) == 13);
    assert(afe_required_ringbuf_frames(16000, 480, 128, 128) == 13);
    // Reproduce ESP-SR 2.5.1's feed guard: free >= retained VAD history +
    // the new frame. A silent fetch increases retained history up to 256 ms.
    // Even an immediately draining consumer cannot keep a four-frame ring alive.
    auto silent_frames_before_stall = [](int ring_frames, int speech_ms = 128) {
        const int frame_bytes = 512 * 2, history_bytes = (speech_ms + 128) * 16 * 2;
        int retained = 0;
        for (int frame = 0; frame < 10000; ++frame) {
            if (ring_frames * frame_bytes < retained + frame_bytes) return frame;
            retained = std::min(retained + frame_bytes, history_bytes);
        }
        return 10000;
    };
    assert(silent_frames_before_stall(4) == 4);
    assert(silent_frames_before_stall(8) == 8); // History alone is not enough.
    assert(silent_frames_before_stall(afe_required_ringbuf_frames(16000, 512, 128, 128)) == 10000);
    assert(silent_frames_before_stall(12, 512) < 10000); // Fixed default capacity would stall.
    for (int speech_ms : {33, 180, 512, 2000}) {
        assert(silent_frames_before_stall(
            afe_required_ringbuf_frames(16000, 512, speech_ms, 128), speech_ms) == 10000);
    }
    const std::vector<int16_t> left = {-32768, -1, 0, 32767, 1, -123, 456};
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SPEECH);
    assert(fifo.empty() && metered == left && processed_level_dbfs == -17.0f);
    assert(processed_level_tick == 42); // RMS sees only the original audio.
    audio_pipeline_set_recording(true);
    current_doa = -18000;
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SILENCE);
    current_doa = 0;
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SPEECH);
    current_doa = 17990;
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_UNKNOWN);
    current_doa = 8000;
    assert(audio_pipeline_available() == 21 && fifo.size() == 21 * 4);
    audio_pipeline_capture2_frame_t readback[21]{};
    // A packet-sized read crosses an AFE decision boundary without relabeling.
    assert(audio_pipeline_read(readback, 16) == 16);
    in_isr = true;
    assert(audio_pipeline_read(readback + 16, 17) == 5);
    in_isr = false;
    assert(isr_reads == 1 && yields == 1 && audio_pipeline_available() == 0);
    for (size_t i = 0; i < 21; ++i) {
        assert(readback[i].audio == left[i % left.size()]);
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
        assert(readback[i].right == left[i % left.size()]);
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
        assert(readback[i].right == (i < 7 ? -18000 : i < 14 ? 0 : 17990));
#else
        assert(readback[i].right == (i < 7 ? INT16_MIN : i < 14 ? INT16_MAX : 0));
#endif
    }
    // Backpressure retains complete pairs and counts a wait once per block.
    PostNsMetadata per_sample[7];
    for (int i = 0; i < 7; ++i) per_sample[i] = {int16_t(i * 1000 - 3000), int8_t(i % 2)};
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_UNKNOWN, 12345, per_sample);
    assert(audio_pipeline_read(readback, 7) == 7);
    for (int i = 0; i < 7; ++i) {
        assert(readback[i].audio == left[i]);
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
        assert(readback[i].right == left[i]);
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
        assert(readback[i].right == per_sample[i].doa);
#else
        assert(readback[i].right == (i % 2 ? INT16_MAX : INT16_MIN));
#endif
    }
    failures = 2;
    current_doa = 7000;
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SPEECH);
    assert(delays == 2 && output_waits == 1 && audio_pipeline_available() == 7);
    audio_pipeline_capture2_frame_t blocked[7];
    std::memcpy(blocked, fifo.data(), sizeof(blocked));
    for (size_t i = 0; i < 7; ++i) {
        assert(blocked[i].audio == left[i]);
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
        assert(blocked[i].right == left[i]);
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
        assert(blocked[i].right == 7000);
#else
        assert(blocked[i].right == INT16_MAX);
#endif
    }
    audio_pipeline_set_recording(false);
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SILENCE);
    assert(audio_pipeline_available() == 7); // Closed recording never enqueues.
    audio_pipeline_set_recording(true);
    assert(audio_pipeline_available() == 0); // Reopening drains complete L/R frames.
    failures = 1; stop_on_delay = true;
    publish_capture2(left.data(), left.size(), AUDIO_PIPELINE_VAD_SPEECH);
    assert(fifo.empty() && !recording && output_waits == 2);
    stop_on_delay = false;
    initialized = false;
    assert(audio_pipeline_available() == 0 && audio_pipeline_read(readback, 1) == 0);
    initialized = true;

    // Count only accepted writes, including individual zero pairs. Retrying
    // backpressure and stopping before a send must not inflate the totals.
    fifo.clear(); recording = true;
    const uint32_t queued_before = capture2_queued_frames.load();
    const uint32_t zeros_before = capture2_queued_zero_pairs.load();
    int16_t diagnostic[] = {12, 0, -34, 0};
    failures = 2;
    publish_capture2(diagnostic, 4, AUDIO_PIPELINE_VAD_UNKNOWN, 0);
    assert(capture2_queued_frames == queued_before + 4);
    assert(capture2_queued_zero_pairs == zeros_before + 2);
    failures = 1; stop_on_delay = true;
    publish_capture2(diagnostic, 4, AUDIO_PIPELINE_VAD_UNKNOWN, 0);
    assert(capture2_queued_frames == queued_before + 4);
    assert(capture2_queued_zero_pairs == zeros_before + 2);
    stop_on_delay = false;

    // Run the real fetch task: audio, VAD and KWS all use the same AFE result.
    audio_pipeline_set_recording(true);
    int16_t a[] = {-32768, -1, 0}, b[] = {32767, 12};
    results = {{a, sizeof(a), 0, 0}, {b, sizeof(b), 0, VAD_SPEECH},
               {nullptr, 0, ESP_FAIL, 0}};
    try { afe_fetch_task(nullptr); } catch (Done &) {}
    assert(ns_queued.size() == 2 && kws_audio.empty() && fifo.empty());
    assert(latest_vad == AUDIO_PIPELINE_VAD_UNKNOWN);
    // AFE only submits; simulate the NS consumer here. The separate NS task
    // integration test exercises real denoising, gap resets and reblocking.
    for (const auto &queued : ns_queued) {
        kws_stream_feed(queued.pcm.data(), queued.pcm.size());
        publish_capture2(queued.pcm.data(), queued.pcm.size(), queued.vad, queued.doa);
    }
    assert(audio_pipeline_available() == 5 && gaps == 1);
    assert((kws_audio == std::vector<int16_t>{-32768, -1, 0, 32767, 12}));
    current_doa = 12345;
    assert(audio_pipeline_read(readback, 21) == 5);
    for (size_t i = 0; i < 5; ++i) {
        assert(readback[i].audio == kws_audio[i]);
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
        assert(readback[i].right == kws_audio[i]);
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
        assert(readback[i].right == (i < 3 ? -4500 : 4500));
#else
        assert(readback[i].right == (i < 3 ? INT16_MIN : INT16_MAX));
#endif
    }
    std::puts("PASS: right selection, queued snapshots, original left PCM/RMS/KWS, VADNet guard, ring sizing, ISR reads, backpressure, reopen");
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", nargs="+", default=["c++"])
    args = parser.parse_args()
    pipeline = (ROOT / "src/audio_processing/audio_pipeline.cpp").read_text(encoding="utf-8")
    code = MOCKS + "\n".join(function(pipeline, name) for name in (
        "load_vadnet_model", "afe_required_ringbuf_frames", "publish_capture2", "afe_fetch_task", "audio_pipeline_read",
        "audio_pipeline_available", "audio_pipeline_set_recording")) + CASES
    with tempfile.TemporaryDirectory(prefix="capture2-vad-") as temp:
        source, binary = Path(temp) / "test.cpp", Path(temp) / "test.exe"
        (Path(temp) / "esp_err.h").write_text("#pragma once\ntypedef int esp_err_t;\n", encoding="utf-8")
        source.write_text(code, encoding="utf-8")
        for mode in ("COPY_LEFT", "VAD", "DOA"):
            subprocess.run(args.cxx + ["-std=c++17", "-Wall", "-Wextra", "-Werror",
                                      f"-DCONFIG_ARRAY_CAPTURE2_RIGHT_{mode}=1", "-I", temp,
                                      "-I", str(ROOT / "src/audio_processing"), str(source),
                                      "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
            print(f"PASS Capture2 right source: {mode}", flush=True)


if __name__ == "__main__":
    main()
