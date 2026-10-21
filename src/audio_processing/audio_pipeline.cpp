#include "audio_pipeline.h"
#ifdef ESP_PLATFORM
#include "custom_control.h"
#else
enum { CUSTOM_WORK_DSP=1,CUSTOM_WORK_FEED=2,CUSTOM_WORK_FETCH=4,CUSTOM_WORK_NS=8,CUSTOM_WORK_DOA=16 };
static bool custom_maintenance_active() { return false; }
static bool custom_worker_checkpoint(unsigned) { return false; }
static void custom_worker_register(unsigned) {}
#endif
#include "audio_diagnostics.h"
#include "kws_stream.h"
#include "array_dsp.hpp"
#include "doa_tracking.hpp"
#include "post_ns_stream.hpp"
#include "nscore/nscore.h"
#include "input_gain_calibration.hpp"
#include "usb_audio.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <new>
#include <vector>
#include "driver/uart.h"
#include "esp_afe_sr_models.h"
#include "model_path.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"

namespace {
using namespace array_dsp;
constexpr int input_channels = channels;
constexpr int afe_sample_rate = 16000;
constexpr int afe_working_frames = 4;
constexpr int afe_vad_min_speech_ms = CONFIG_ARRAY_VAD_MIN_SPEECH_MS;
constexpr int afe_vad_min_noise_ms = CONFIG_ARRAY_VAD_MIN_NOISE_MS;
constexpr int afe_vad_delay_ms = 128;
// ESP-SR 2.5.1 keeps min_speech + delay of VAD history in the feed ring.
// feed() requires free bytes for that history AND the next processing frame.
constexpr int afe_required_ringbuf_frames(int rate, int frame_samples, int speech_ms, int delay_ms) {
    const int64_t history_samples_ms = int64_t(rate) * (speech_ms + delay_ms);
    const int64_t frame_samples_ms = int64_t(frame_samples) * 1000;
    return afe_working_frames + int((history_samples_ms + frame_samples_ms - 1) / frame_samples_ms);
}
// This AFE version uses 512-sample fetch frames without WakeNet. Validate the
// actual frame size below so a future library change cannot silently stall it.
constexpr int afe_ringbuf_frames = afe_required_ringbuf_frames(
    afe_sample_rate, 512, afe_vad_min_speech_ms, afe_vad_delay_ms);
#if CONFIG_ARRAY_CAPTURE2_SINE_TEST || CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
// One 1-kHz period at 16 kHz; 12000-peak PCM leaves headroom.
constexpr int16_t capture2_sine_table[16] = {
       0,  4592,  8485, 11087, 12000, 11087,  8485,  4592,
       0, -4592, -8485,-11087,-12000,-11087, -8485, -4592,
};
#endif
constexpr size_t input_fifo_bytes = sample_rate * input_channels * sizeof(int16_t) / 4;
constexpr size_t afe_fifo_bytes = afe_sample_rate * 2 * sizeof(int16_t) / 5;
constexpr int reference_capacity = (afe_sample_rate / 1000) * 250;
constexpr float silence_dbfs = -120.0f;
constexpr TickType_t level_stale_ticks = pdMS_TO_TICKS(150);
/* The microphone reaches 16 kHz after one OLA hop and the 24-to-16 kHz FIR.
 * Delay the native 16-kHz playback reference by the equivalent whole-sample
 * latency before feeding AEC. Acoustic/DMA delay remains user configurable. */
constexpr int reference_pipeline_delay =
    (hop * afe_sample_rate + sample_rate - 1) / sample_rate +
    ((resampler_taps - 1) * afe_sample_rate + 2 * sample_rate - 1) /
        (2 * sample_rate);
const char *TAG = "audio_dsp";
bool capture_available = true;
std::atomic<bool> initialized{false};
std::atomic<bool> doa_log_enabled{false};
std::atomic<bool> recording{false};
std::atomic<float> bearing{0};
std::atomic<float> doa_target{0};
std::atomic<int32_t> doa_sample{-18000};
std::atomic<int32_t> vad_state{AUDIO_PIPELINE_VAD_UNKNOWN};
static_assert(std::atomic<int32_t>::is_always_lock_free,
              "USB ISR requires a lock-free integer DOA snapshot");
std::atomic<float> processed_level_dbfs{silence_dbfs};
std::atomic<float> playback_level_dbfs{silence_dbfs};
std::atomic<uint32_t> processed_level_tick{0};
std::atomic<uint32_t> playback_level_tick{0};
std::atomic<uint32_t> capture_waits{0}, afe_waits{0}, output_waits{0}, reference_drops{0}, reference_zeros{0};
std::atomic<uint32_t> capture2_queued_frames{0}, capture2_queued_zero_pairs{0};
std::atomic<size_t> input_peak_bytes{0};
std::atomic<int64_t> afe_worst_us{0}, hop_worst_us{0};
StreamBufferHandle_t input_stream, afe_stream, output_stream;
StaticStreamBuffer_t input_stream_control, afe_stream_control;
uint8_t *input_stream_storage, *afe_stream_storage;
SemaphoreHandle_t stats_mutex;
Statistics *published_stats, *doa_stats;
bool stats_ready;
Beamformer *beamformer;
Locator *locator;
const esp_afe_sr_iface_t *afe;
esp_afe_sr_data_t *afe_data;
srmodel_list_t *afe_models;
char *afe_vad_model;
void *afe_model_blob;
int16_t *afe_feed_buffer;
int afe_feed_samples;
audio_pipeline_capture2_frame_t *capture2_scratch;
size_t capture2_scratch_frames;
static_assert(sizeof(audio_pipeline_capture2_frame_t) == 2 * sizeof(int16_t),
              "Capture2 must contain packed PCM16 left/right samples");
TaskHandle_t dsp_task_handle;
TaskHandle_t ns_task_handle;
constexpr unsigned ns_queue_blocks = 8;
QueueHandle_t ns_queue;
StaticQueue_t ns_queue_control;
uint8_t *ns_queue_storage;
PostNsInput *ns_producer;
uint32_t ns_sequence;
std::atomic<uint32_t> ns_frames{0}, ns_dropped_samples{0}, ns_resets{0};
std::atomic<uint64_t> ns_process_us{0};
std::atomic<int64_t> ns_worst_us{0};
portMUX_TYPE reference_lock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE output_consumer_lock = portMUX_INITIALIZER_UNLOCKED;
int16_t reference_fifo[reference_capacity];
unsigned reference_read, reference_write, reference_count;

esp_err_t load_vadnet_model() {
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "model");
    ESP_RETURN_ON_FALSE(partition, ESP_ERR_NOT_FOUND, TAG,
                        "missing model partition; run idf.py flash including partition table and models");
    // The ESP-SR packed-model loader trusts the model count. Reject a blank
    // or mismatched partition before it parses erased flash as model metadata.
    struct {
        uint32_t models;
        char name[32];
        uint32_t files;
    } header{};
    static_assert(sizeof(header) == 40);
    ESP_RETURN_ON_ERROR(esp_partition_read(partition, 0, &header, sizeof(header)),
                        TAG, "read VADNet model header");
    ESP_RETURN_ON_FALSE(header.models == 1 && header.files == 3 &&
                            std::strncmp(header.name, "vadnet1_medium", sizeof(header.name)) == 0,
                        ESP_ERR_INVALID_STATE, TAG,
                        "model partition does not contain vadnet1_medium; run idf.py flash (not app-flash)");
    // Keep VAD assets in PSRAM: a permanent flash mmap would force cache-off
    // writes even with PSRAM XIP, starving the USB isochronous callbacks.
    afe_model_blob=heap_caps_aligned_alloc(16,partition->size,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(afe_model_blob,ESP_ERR_NO_MEM,TAG,"allocate VAD model copy");
    ESP_RETURN_ON_ERROR(esp_partition_read(partition,0,afe_model_blob,partition->size),TAG,"read VAD model copy");
    afe_models = srmodel_load(afe_model_blob);
    ESP_RETURN_ON_FALSE(afe_models, ESP_FAIL, TAG, "load VADNet models");
    afe_vad_model = esp_srmodel_filter(afe_models, ESP_VADN_PREFIX, "medium");
    ESP_RETURN_ON_FALSE(afe_vad_model && std::strcmp(afe_vad_model, "vadnet1_medium") == 0,
                        ESP_ERR_NOT_FOUND, TAG, "vadnet1_medium model unavailable");
    return ESP_OK;
}

void publish_vad_state(audio_pipeline_vad_state_t state) {
    vad_state.store(state, std::memory_order_relaxed);
}

int16_t pcm(float value) {
    if (!std::isfinite(value)) return 0;
    return int16_t(std::lround(std::clamp(value * 32768.0f, -32768.0f, 32767.0f)));
}
float pcm_rms_dbfs(uint64_t square_sum, size_t sample_count) {
    if (!sample_count || !square_sum) return silence_dbfs;
    const float rms = std::sqrt(static_cast<float>(square_sum) /
                                static_cast<float>(sample_count));
    return std::max(silence_dbfs, 20.0f * std::log10(rms / 32768.0f));
}
float pcm_rms_dbfs(const int16_t *samples, size_t sample_count) {
    uint64_t square_sum = 0;
    for (size_t i = 0; i < sample_count; ++i) {
        const int32_t sample = samples[i];
        square_sum += static_cast<uint64_t>(static_cast<int64_t>(sample) * sample);
    }
    return pcm_rms_dbfs(square_sum, sample_count);
}
// Prevent partial writes from shifting channel/frame alignment in stream buffers.
bool send_whole(StreamBufferHandle_t stream, const void *data, size_t bytes) {
    if (xStreamBufferSpacesAvailable(stream) < bytes) return false;
    return xStreamBufferSend(stream, data, bytes, 0) == bytes;
}
// Single producer per stream. Wait for the whole frame/chunk so a partial
// FreeRTOS stream-buffer write can never change audio channel alignment.
void send_whole_wait(StreamBufferHandle_t stream, const void *data, size_t bytes,
                     std::atomic<uint32_t> &waits) {
    bool waited = false;
    while (!send_whole(stream, data, bytes)) {
        if (custom_maintenance_active()) return;
        if (!waited) {
            waits.fetch_add(1, std::memory_order_relaxed);
            waited = true;
        }
        vTaskDelay(1);
    }
}
void update_worst(std::atomic<int64_t> &peak, int64_t value) {
    int64_t previous = peak.load(std::memory_order_relaxed);
    while (value > previous && !peak.compare_exchange_weak(
               previous, value, std::memory_order_relaxed)) {}
}
void take_reference(int16_t *out, size_t count) {
    portENTER_CRITICAL(&reference_lock);
    for (size_t i = 0; i < count; ++i) {
        if (reference_count) {
            out[i] = reference_fifo[reference_read];
            reference_read = (reference_read + 1) % reference_capacity;
            --reference_count;
        } else {
            out[i] = 0;
            reference_zeros.fetch_add(1, std::memory_order_relaxed);
        }
    }
    portEXIT_CRITICAL(&reference_lock);
}
void publish_bearing(float degrees);
void dsp_task(void *) {
    custom_worker_register(CUSTOM_WORK_DSP);
    // Allocations occur once, never on the per-hop path.
    std::vector<int16_t> input(hop * input_channels);
    std::vector<int16_t> microphones(hop * channels);
    std::vector<float> mono(hop);
    std::vector<int16_t> ref_delay(
        reference_pipeline_delay + CONFIG_ARRAY_AEC_REFERENCE_DELAY_MS *
                                       (afe_sample_rate / 1000),
        0);
    size_t delay_pos = 0;
    Resampler mic_resampler;
    DoaTracking tracking(CONFIG_ARRAY_DOA_ROTATION_SPEED_DEG_PER_SEC, float(hop) / sample_rate);
    std::vector<int16_t> pairs(hop * 2);
    int64_t next_snapshot = esp_timer_get_time() + 450000;
    unsigned processed_hops = 0;
    while (true) {
        if (custom_worker_checkpoint(CUSTOM_WORK_DSP)) {
            std::fill(ref_delay.begin(),ref_delay.end(),0); delay_pos=0; mic_resampler=Resampler();
        }
        size_t got = 0;
        const size_t input_bytes = input.size() * sizeof(int16_t);
        while (got < input_bytes && !custom_maintenance_active())
            got += xStreamBufferReceive(input_stream, reinterpret_cast<uint8_t *>(input.data()) + got,
                                         input_bytes - got, pdMS_TO_TICKS(10));
        if (custom_maintenance_active()) continue;
        const int64_t started = esp_timer_get_time();
        for (int i = 0; i < hop; ++i)
            for (int m = 0; m < channels; ++m) microphones[i * channels + m] = input[i * input_channels + m];
        const float angle = tracking.update(doa_target.load(std::memory_order_relaxed),
            vad_state.load(std::memory_order_relaxed) == AUDIO_PIPELINE_VAD_SPEECH, started);
        if (angle != bearing.load(std::memory_order_relaxed)) publish_bearing(angle);
        beamformer->process(microphones.data(), angle, mono.data());
        size_t pair_count = 0;
        for (int i = 0; i < hop; ++i) {
            float mic16 = 0;
            bool emit = mic_resampler.push(mono[i], mic16);
            if (emit) {
                int16_t reference;
                take_reference(&reference, 1);
                int16_t delayed_reference = ref_delay[delay_pos];
                ref_delay[delay_pos] = reference;
                delay_pos = (delay_pos + 1) % ref_delay.size();
                pairs[2 * pair_count] = pcm(mic16);
                pairs[2 * pair_count + 1] = delayed_reference;
                ++pair_count;
            }
        }
        send_whole_wait(afe_stream, pairs.data(), pair_count * 2 * sizeof(int16_t), afe_waits);
        int64_t now = esp_timer_get_time();
        if (now >= next_snapshot) {
            if (xSemaphoreTake(stats_mutex, 0) == pdTRUE) {
                *published_stats = beamformer->statistics();
                stats_ready = true;
                xSemaphoreGive(stats_mutex);
            }
            next_snapshot += 500000;
            if (next_snapshot <= now) next_snapshot = now + 500000;
        }
        update_worst(hop_worst_us, esp_timer_get_time() - started);
        // Preserve idle-task/watchdog service even if hardware load overruns
        // the DSP budget and the bounded input queue stays continuously full.
        if ((++processed_hops % 32) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void diagnostics_stats_task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    configRUN_TIME_COUNTER_TYPE last_dsp_runtime = 0;
    configRUN_TIME_COUNTER_TYPE last_ns_runtime = 0;
    uint64_t window_dsp_cpu_us = 0;
    uint64_t window_ns_cpu_us = 0;
    int64_t window_started_us = esp_timer_get_time();
    while (true) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10000));
        kws_stream_log_stats();
        TaskStatus_t task_info{};
        vTaskGetInfo(dsp_task_handle, &task_info, pdFALSE, eReady);
        const configRUN_TIME_COUNTER_TYPE runtime = task_info.ulRunTimeCounter;
        const configRUN_TIME_COUNTER_TYPE delta = runtime - last_dsp_runtime;
        last_dsp_runtime = runtime;
        window_dsp_cpu_us += delta;
        vTaskGetInfo(ns_task_handle, &task_info, pdFALSE, eReady);
        const configRUN_TIME_COUNTER_TYPE ns_runtime = task_info.ulRunTimeCounter;
        window_ns_cpu_us += configRUN_TIME_COUNTER_TYPE(ns_runtime - last_ns_runtime);
        last_ns_runtime = ns_runtime;
        // Only the background task formats statistics. Retain peak values
        // throughout quiet periods so the next idle report includes them.
        if (!audio_diagnostics_info_enabled()) continue;
        const int64_t now = esp_timer_get_time();
        const int64_t interval_us = now - window_started_us;
        ESP_LOGI(TAG, "DAS task CPU=%.1f%% runtime=%.3f ms interval=%.3f s core=1 priority=6",
                 interval_us > 0 ? 100.0 * window_dsp_cpu_us / interval_us : 0.0,
                 double(window_dsp_cpu_us) / 1000.0, double(interval_us) / 1000000.0);
        window_dsp_cpu_us = 0;
        const uint32_t processed_ns_frames = ns_frames.exchange(0, std::memory_order_relaxed);
        const uint64_t processed_ns_us = ns_process_us.exchange(0, std::memory_order_relaxed);
        ESP_LOGI(TAG, "post-AFE NS core=1 priority=4 CPU=%.1f%% frames=%lu avg=%.3f ms max=%.3f ms dropped=%lu samples resets=%lu",
                 interval_us > 0 ? 100.0 * window_ns_cpu_us / interval_us : 0.0,
                 (unsigned long)processed_ns_frames,
                 processed_ns_frames ? double(processed_ns_us) / processed_ns_frames / 1000.0 : 0.0,
                 double(ns_worst_us.exchange(0, std::memory_order_relaxed)) / 1000.0,
                 (unsigned long)ns_dropped_samples.load(), (unsigned long)ns_resets.load());
        window_ns_cpu_us = 0;
        window_started_us = now;
        const int64_t worst_hop_us = hop_worst_us.exchange(0, std::memory_order_relaxed);
        const int64_t aec_worst_us = afe_worst_us.exchange(0, std::memory_order_relaxed);
        usb_audio_capture_stats_t usb_stats;
        usb_audio_get_capture_stats(&usb_stats);
        audio_pipeline_capture2_stats_t capture2_stats;
        audio_pipeline_get_capture2_stats(&capture2_stats);
        audio_diagnostics_stats_t log_stats;
        audio_diagnostics_get_stats(&log_stats);
        ESP_LOGI(TAG, "hop worst=%lld us (budget=5333), waits input=%lu AFE=%lu USB=%lu, input peak=%u ms, ref drop=%lu zero=%lu",
                 (long long)worst_hop_us, (unsigned long)capture_waits.load(),
                 (unsigned long)afe_waits.load(), (unsigned long)output_waits.load(),
                 (unsigned)(input_peak_bytes.load() * 1000 /
                            (sample_rate * input_channels * sizeof(int16_t))),
                 (unsigned long)reference_drops.load(), (unsigned long)reference_zeros.load());
        ESP_LOGI(TAG, "capture I2S anomalies=%lu, raw FIFO overflow=%lu, USB null raw=%lu mono=%lu, FIFO blocked=%lu short writes=%lu",
                 (unsigned long)usb_stats.i2s_read_anomalies,
                 (unsigned long)usb_stats.raw_fifo_overflows,
                 (unsigned long)usb_stats.raw_endpoint_null_packets,
                 (unsigned long)usb_stats.mono_endpoint_null_packets,
                 (unsigned long)usb_stats.endpoint_fifo_backpressure,
                 (unsigned long)usb_stats.endpoint_write_shorts);
        ESP_LOGI(TAG, "capture2 TX packets=%lu zero=%lu (scheduled=%lu failed=%lu unexpected=%lu) unusual=%lu, source reads=%lu short=%lu zero=%lu min=%lu samples",
                 (unsigned long)usb_stats.mono_tx_packets,
                 (unsigned long)usb_stats.mono_tx_zero_packets,
                 (unsigned long)usb_stats.mono_tx_zero_scheduled,
                 (unsigned long)usb_stats.mono_tx_zero_failed,
                 (unsigned long)usb_stats.mono_tx_zero_unexpected,
                 (unsigned long)usb_stats.mono_tx_unexpected_sizes,
                 (unsigned long)usb_stats.mono_source_read_packets,
                 (unsigned long)usb_stats.mono_source_short_reads,
                 (unsigned long)usb_stats.mono_source_zero_packets,
                 (unsigned long)usb_stats.mono_source_min_samples);
        ESP_LOGI(TAG, "capture2 lifetime frames queued/read=%lu/%lu zero-pairs queued/read=%lu/%lu; session packets 15/16/17=%lu/%lu/%lu",
                 (unsigned long)capture2_stats.queued_frames,
                 (unsigned long)usb_stats.mono_source_frames,
                 (unsigned long)capture2_stats.queued_zero_pairs,
                 (unsigned long)usb_stats.mono_source_zero_pairs,
                 (unsigned long)usb_stats.mono_source_packets_15,
                 (unsigned long)usb_stats.mono_source_packets_16,
                 (unsigned long)usb_stats.mono_source_packets_17);
        ESP_LOGI(TAG, "AFE feed worst=%lld us, log dropped=%lu",
                 (long long)aec_worst_us, (unsigned long)log_stats.dropped_logs);
        ESP_LOGI(TAG, "playback underruns=%lu dropped=%lu bytes I2S errors=%lu queue peak=%lu bytes",
                 (unsigned long)usb_stats.playback_underflows,
                 (unsigned long)usb_stats.playback_dropped_bytes,
                 (unsigned long)usb_stats.playback_i2s_errors,
                 (unsigned long)usb_stats.playback_peak_bytes);
    }
}
// Task context only: USB callbacks can run in an ISR where FPU use is forbidden.
void publish_bearing(float degrees) {
    bearing.store(degrees, std::memory_order_relaxed);
    const float bounded = std::clamp(degrees, 0.0f, 360.0f);
    doa_sample.store(static_cast<int32_t>(std::lround(bounded * 100.0f - 18000.0f)),
                     std::memory_order_relaxed);
}
void doa_task(void *) {
    custom_worker_register(CUSTOM_WORK_DOA);
    TickType_t last_wake = xTaskGetTickCount();
    bool valid = false;
    float sharpness = 0;
    while (true) {
        // Reporting cadence is independent of USB recording and LED activity.
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
        if (custom_worker_checkpoint(CUSTOM_WORK_DOA)) last_wake=xTaskGetTickCount();
        bool ready = false;
        if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (stats_ready) {
                *doa_stats = *published_stats;
                stats_ready = false;
                ready = true;
            }
            xSemaphoreGive(stats_mutex);
        }
        int64_t started = esp_timer_get_time();
        if (ready && vad_state.load(std::memory_order_relaxed) == AUDIO_PIPELINE_VAD_SPEECH) {
            Bearing result = locator->estimate(*doa_stats);
            valid = result.valid;
            sharpness = result.sharpness;
            // Estimation can be preempted by DSP/AFE. Recheck VAD before
            // accepting the target; only DSP publishes the smoothed angle.
            if (valid && vad_state.load(std::memory_order_relaxed) == AUDIO_PIPELINE_VAD_SPEECH)
                doa_target.store(result.degrees, std::memory_order_relaxed);
        } else valid = false;
        if (doa_log_enabled.load(std::memory_order_relaxed) &&
            audio_diagnostics_info_enabled()) {
            ESP_LOGI(TAG, "DOA SRP-PHAT azimuth=%.1f deg valid=%d sharpness=%.3f compute=%lld us",
                     (double)bearing.load(), valid,
                     (double)sharpness, (long long)(esp_timer_get_time() - started));
        }
    }
}
void publish_capture2(const int16_t *samples, size_t count,
                      [[maybe_unused]] audio_pipeline_vad_state_t state,
                      [[maybe_unused]] int16_t angle_sample = audio_pipeline_get_doa_sample(),
                      [[maybe_unused]] const PostNsMetadata *metadata = nullptr) {
    processed_level_dbfs.store(pcm_rms_dbfs(samples, count), std::memory_order_relaxed);
    processed_level_tick.store(static_cast<uint32_t>(xTaskGetTickCount()),
                               std::memory_order_release);
    // Keep AFE running while USB capture is closed; discard only its USB output.
    if (recording.load()) {
        bool waited = false;
        // Fix metadata before queuing, including across backpressure waits.
        // USB transports complete pairs and never reads newer global status.
        for (size_t offset = 0; offset < count && recording.load() && !custom_maintenance_active();) {
            const size_t frames = std::min(count - offset, capture2_scratch_frames);
            uint32_t zero_pairs = 0;
            for (size_t i = 0; i < frames; ++i) {
                const int16_t left = samples[offset + i];
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
                capture2_scratch[i] = {left, left};
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
                capture2_scratch[i] = {left, metadata ? metadata[offset + i].doa : angle_sample};
#else
                const int sample_vad = metadata ? metadata[offset + i].vad : state;
                const int16_t marker = sample_vad == AUDIO_PIPELINE_VAD_SPEECH ? INT16_MAX
                                     : sample_vad == AUDIO_PIPELINE_VAD_SILENCE ? INT16_MIN : 0;
                capture2_scratch[i] = {left, marker};
#endif
                if (capture2_scratch[i].audio == 0 && capture2_scratch[i].right == 0)
                    ++zero_pairs;
            }
            const size_t bytes = frames * sizeof(audio_pipeline_capture2_frame_t);
            bool sent = false;
            while (recording.load() && !custom_maintenance_active()) {
                if (send_whole(output_stream, capture2_scratch, bytes)) {
                    sent = true;
                    break;
                }
                if (!waited) {
                    output_waits.fetch_add(1, std::memory_order_relaxed);
                    waited = true;
                }
                vTaskDelay(1);
            }
            if (sent) {
                capture2_queued_frames.fetch_add(frames, std::memory_order_relaxed);
                capture2_queued_zero_pairs.fetch_add(zero_pairs, std::memory_order_relaxed);
            }
            offset += frames;
        }
    }
}
// Only AFE fetch writes this queue. A dropped block still consumes its sequence
// so the NS worker resets at the gap after older queued audio, without blocking AFE.
void post_ns_submit(const int16_t *samples, size_t count, audio_pipeline_vad_state_t state) {
    const PostNsMetadata metadata{audio_pipeline_get_doa_sample(), static_cast<int8_t>(state)};
    while (count) {
        const size_t take = std::min(count, size_t(1024));
        ns_producer->sequence = ns_sequence++;
        ns_producer->samples = static_cast<uint16_t>(take);
        ns_producer->metadata = metadata;
        std::memcpy(ns_producer->pcm, samples, take * sizeof(*samples));
        if (xQueueSend(ns_queue, ns_producer, 0) != pdTRUE)
            ns_dropped_samples.fetch_add(take, std::memory_order_relaxed);
        samples += take;
        count -= take;
    }
}
void post_ns_discontinuity() { ++ns_sequence; }
void post_ns_task(void *) {
    custom_worker_register(CUSTOM_WORK_NS);
    constexpr unsigned min_gain = CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT;
    array_nscore_init(min_gain);
    PostNsFramer framer(min_gain == 100);
    PostNsInput block;
    bool have_sequence = false;
    uint32_t expected_sequence = 0;
    unsigned iterations = 0;
#if CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
    size_t tone_phase = 0;
#endif
    while (true) {
        if (custom_worker_checkpoint(CUSTOM_WORK_NS)) {
            array_nscore_init(min_gain); framer.reset(); have_sequence=false;
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
            kws_stream_discontinuity();
#endif
        }
        if (xQueueReceive(ns_queue, &block, pdMS_TO_TICKS(10)) != pdTRUE) continue;
        if (custom_maintenance_active()) continue;
        if (have_sequence && block.sequence != expected_sequence) {
            array_nscore_init(min_gain);
            framer.reset();
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
            kws_stream_discontinuity();
#endif
            ns_resets.fetch_add(1, std::memory_order_relaxed);
        }
        have_sequence = true;
        expected_sequence = block.sequence + 1;
        framer.feed(block.pcm, block.samples, block.metadata,
            [](const int16_t *input, int16_t *output) {
                const int64_t started = esp_timer_get_time();
                array_nscore_process(input, output);
                const int64_t elapsed = esp_timer_get_time() - started;
                ns_process_us.fetch_add(elapsed, std::memory_order_relaxed);
                ns_frames.fetch_add(1, std::memory_order_relaxed);
                update_worst(ns_worst_us, elapsed);
            },
            [&]([[maybe_unused]] const int16_t *pcm, [[maybe_unused]] const PostNsMetadata *metadata,
               [[maybe_unused]] size_t count) {
#if !CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
                // Only the configured source owns the KWS framer/queue.
                // PostNsFramer emits 160-sample blocks with VAD delayed by
                // the same NS overlap-add latency as the PCM. Mixed/unknown
                // frames cannot contribute to continuous speech confirmation.
                const bool vad_high = std::all_of(metadata, metadata + count,
                    [](const PostNsMetadata &sample) { return sample.vad == AUDIO_PIPELINE_VAD_SPEECH; });
                kws_stream_feed_with_vad(pcm, count, vad_high);
#endif
#if CONFIG_ARRAY_CAPTURE2_AEC_OUTPUT
                publish_capture2(pcm, count, AUDIO_PIPELINE_VAD_UNKNOWN, -18000, metadata);
#elif CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
                int16_t diagnostic[PostNsFramer::frame_samples];
                for (size_t i = 0; i < count; ++i) {
                    diagnostic[i] = capture2_sine_table[tone_phase];
                    tone_phase = (tone_phase + 1U) % 16U;
                }
                publish_capture2(diagnostic, count, AUDIO_PIPELINE_VAD_UNKNOWN, -18000, metadata);
#endif
            });
        if ((++iterations % 32) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
}
void afe_feed_task(void *) {
    custom_worker_register(CUSTOM_WORK_FEED);
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    std::vector<int16_t> kws_reference(afe_feed_samples);
#endif
#if CONFIG_ARRAY_CAPTURE2_SINE_TEST
    size_t tone_phase = 0;
    std::vector<int16_t> diagnostic(afe_feed_samples);
#elif CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST
    std::vector<int16_t> diagnostic(afe_feed_samples);
#endif
    while (true) {
        custom_worker_checkpoint(CUSTOM_WORK_FEED);
        size_t got = 0;
        const size_t bytes = afe_feed_samples * 2 * sizeof(int16_t);
        while (got < bytes && !custom_maintenance_active())
            got += xStreamBufferReceive(afe_stream, reinterpret_cast<uint8_t *>(afe_feed_buffer) + got,
                                         bytes - got, pdMS_TO_TICKS(10));
        if (custom_maintenance_active()) continue;
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
        // MR pairs already contain delayed 16-kHz mono playback. Copy before
        // AFE can reuse/mutate the input, without consuming the reference twice.
        for (int i = 0; i < afe_feed_samples; ++i)
            kws_reference[i] = afe_feed_buffer[2 * i + 1];
        // Reference input bypasses AFE fetch; attach its latest published VAD
        // now rather than reading a newer state when queued inference runs.
        kws_stream_feed_with_vad(kws_reference.data(), kws_reference.size(),
            audio_pipeline_get_vad_state() == AUDIO_PIPELINE_VAD_SPEECH);
#endif
        const int64_t started = esp_timer_get_time();
        afe->feed(afe_data, afe_feed_buffer);
        update_worst(afe_worst_us, esp_timer_get_time() - started);
#if CONFIG_ARRAY_CAPTURE2_SINE_TEST
        for (int i = 0; i < afe_feed_samples; ++i) {
            diagnostic[i] = capture2_sine_table[tone_phase];
            tone_phase = (tone_phase + 1U) % 16U;
        }
        publish_capture2(diagnostic.data(), diagnostic.size(), audio_pipeline_get_vad_state());
#elif CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST
        for (int i = 0; i < afe_feed_samples; ++i)
            diagnostic[i] = afe_feed_buffer[2 * i + 1];
        publish_capture2(diagnostic.data(), diagnostic.size(), audio_pipeline_get_vad_state());
#endif
    }
}
void afe_fetch_task(void *) {
    custom_worker_register(CUSTOM_WORK_FETCH);
    while (true) {
        custom_worker_checkpoint(CUSTOM_WORK_FETCH);
        afe_fetch_result_t *result = afe->fetch_with_delay(afe_data, pdMS_TO_TICKS(100));
        if (custom_maintenance_active()) continue;
        if (!result || result->ret_value == ESP_FAIL || !result->data || result->data_size <= 0) {
            publish_vad_state(AUDIO_PIPELINE_VAD_UNKNOWN);
            post_ns_discontinuity();
            continue;
        }
        const auto frame_vad = result->vad_state == VAD_SPEECH ? AUDIO_PIPELINE_VAD_SPEECH
                                                             : AUDIO_PIPELINE_VAD_SILENCE;
        publish_vad_state(frame_vad);
        // Copy before fetch reuses the buffer. Never run NS/KWS on the AFE core
        // or wait for either task; sequence gaps expose any dropped NS input.
        post_ns_submit(result->data,
                       static_cast<size_t>(result->data_size) / sizeof(result->data[0]), frame_vad);
    }
}
#if !CONFIG_WS2812_LED_RING_ENABLE
constexpr uart_port_t console_uart = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
void console_task(void *) {
    while (true) {
        uint8_t command;
        if (uart_read_bytes(console_uart, &command, 1, pdMS_TO_TICKS(20)) == 1)
            audio_pipeline_console_command(command);
    }
}
#endif
}

extern "C" void audio_pipeline_set_capture_available(bool available) { capture_available = available; }
extern "C" bool audio_pipeline_capture_available(void) { return capture_available; }
extern "C" esp_err_t audio_pipeline_init(void) {
    ESP_RETURN_ON_FALSE(!initialized.load(), ESP_ERR_INVALID_STATE, TAG, "already initialized");
    ESP_RETURN_ON_FALSE(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0, ESP_ERR_NO_MEM,
                        TAG, "array processing requires enabled PSRAM");
    ESP_RETURN_ON_ERROR(load_vadnet_model(), TAG, "initialize AFE VADNet model");
    const float radius = CONFIG_ARRAY_RADIUS_MM / 1000.0f;
    beamformer = new(std::nothrow) Beamformer(radius, calibrated_channel_gains);
    locator = new(std::nothrow) Locator(radius);
    published_stats = new(std::nothrow) Statistics;
    doa_stats = new(std::nothrow) Statistics;
    stats_mutex = xSemaphoreCreateMutex();
    ns_queue_storage = static_cast<uint8_t *>(heap_caps_malloc(
        ns_queue_blocks * sizeof(PostNsInput), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    ns_producer = static_cast<PostNsInput *>(heap_caps_malloc(sizeof(PostNsInput), MALLOC_CAP_8BIT));
    if (ns_queue_storage)
        ns_queue = xQueueCreateStatic(ns_queue_blocks, sizeof(PostNsInput), ns_queue_storage, &ns_queue_control);
    // Stream storage lives in PSRAM; control blocks stay in internal RAM.
    // Static stream buffers reserve one byte to distinguish full from empty.
    input_stream_storage = static_cast<uint8_t *>(
        heap_caps_malloc(input_fifo_bytes + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    afe_stream_storage = static_cast<uint8_t *>(
        heap_caps_malloc(afe_fifo_bytes + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (input_stream_storage)
        input_stream = xStreamBufferCreateStatic(input_fifo_bytes + 1,
            hop * input_channels * sizeof(int16_t), input_stream_storage, &input_stream_control);
    if (afe_stream_storage)
        afe_stream = xStreamBufferCreateStatic(afe_fifo_bytes + 1, 4,
                                               afe_stream_storage, &afe_stream_control);
    // Preserve 128 ms of stereo capacity for paired left/right samples.
    output_stream = xStreamBufferCreate(8192, sizeof(audio_pipeline_capture2_frame_t));
    ESP_RETURN_ON_FALSE(beamformer && locator && published_stats && doa_stats && stats_mutex &&
                         input_stream && afe_stream && output_stream && ns_queue && ns_producer,
                        ESP_ERR_NO_MEM, TAG, "allocate DSP/NS buffers");
    // MR: one beamformed microphone followed by the mono playback reference.
    afe_config_t *config = afe_config_init("MR", afe_models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    ESP_RETURN_ON_FALSE(config, ESP_ERR_NO_MEM, TAG, "allocate AFE config");
    config->aec_init = true;
    config->aec_mode = AEC_MODE_FD_HIGH_PERF;
    config->se_init = false;
    config->ns_init = false;
    config->vad_init = true;
    config->vad_model_name = afe_vad_model;
    config->vad_mode = VAD_MODE_1;
    config->vad_min_speech_ms = afe_vad_min_speech_ms;
    config->vad_min_noise_ms = afe_vad_min_noise_ms;
    config->vad_delay_ms = afe_vad_delay_ms;
    config->vad_mute_playback = false;
    config->vad_enable_channel_trigger = false;
    config->wakenet_init = false;
    config->agc_init = true;
    config->agc_mode = AFE_AGC_MODE_WEBRTC;
    config->agc_compression_gain_db = CONFIG_ARRAY_AGC_GAIN_DB;
    config->agc_target_level_dbfs = 3;
    config->afe_perferred_core = 0;
    config->afe_perferred_priority = 5;
    config->afe_ringbuf_size = afe_ringbuf_frames;
    config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    config->fixed_output_channel = true;
    config->output_playback_channel = false;
    afe = esp_afe_handle_from_config(config);
    if (afe) afe_data = afe->create_from_config(config);
    afe_config_free(config);
    ESP_RETURN_ON_FALSE(afe && afe_data, ESP_FAIL, TAG, "create Espressif AFE");
    ESP_RETURN_ON_FALSE(afe->get_feed_channel_num(afe_data) == 2 &&
                        afe->get_fetch_channel_num(afe_data) == 1 &&
                        afe->get_samp_rate(afe_data) == afe_sample_rate,
                        ESP_ERR_INVALID_STATE, TAG, "AFE requires mono mic + AEC reference in, mono audio out at 16 kHz");
    afe_feed_samples = afe->get_feed_chunksize(afe_data);
    ESP_RETURN_ON_FALSE(afe_feed_samples > 0 && afe_feed_samples <= 1024,
                        ESP_ERR_INVALID_SIZE, TAG, "invalid AFE feed frame");
    const int afe_fetch_samples = afe->get_fetch_chunksize(afe_data);
    ESP_RETURN_ON_FALSE(afe_fetch_samples > 0 && afe_fetch_samples <= 1024,
                        ESP_ERR_INVALID_SIZE, TAG, "invalid AFE fetch frame");
    ESP_RETURN_ON_FALSE(afe_ringbuf_frames >= afe_required_ringbuf_frames(
                            afe_sample_rate, afe_fetch_samples,
                            afe_vad_min_speech_ms, afe_vad_delay_ms),
                        ESP_ERR_INVALID_SIZE, TAG, "AFE ring too small for VAD history");
    capture2_scratch_frames = static_cast<size_t>(std::max(afe_feed_samples, afe_fetch_samples));
    capture2_scratch = static_cast<audio_pipeline_capture2_frame_t *>(heap_caps_malloc(
        capture2_scratch_frames * sizeof(audio_pipeline_capture2_frame_t), MALLOC_CAP_8BIT));
    ESP_RETURN_ON_FALSE(capture2_scratch, ESP_ERR_NO_MEM, TAG, "allocate Capture2 stereo buffer");
    ESP_LOGI(TAG, "AFE chunks: feed=%d samples, fetch=%d samples at %d Hz",
             afe_feed_samples, afe_fetch_samples, afe_sample_rate);
    ESP_LOGI(TAG, "AFE channels: microphone=1 reference=1 (MR), output=1; VAD model=%s", afe_vad_model);
    ESP_LOGI(TAG, "AFE ring=%d frames: VAD history=%d ms + %d working frames",
             afe_ringbuf_frames, afe_vad_min_speech_ms + afe_vad_delay_ms, afe_working_frames);
    afe_feed_buffer = static_cast<int16_t *>(heap_caps_aligned_alloc(
        16, afe_feed_samples * 2 * sizeof(int16_t), MALLOC_CAP_8BIT));
    ESP_RETURN_ON_FALSE(afe_feed_buffer, ESP_ERR_NO_MEM, TAG, "allocate aligned AFE feed buffer");
    // Core 1: DAS/OLA/resampling (6) preempts post-AFE NS (4), then DOA (3).
    // Core 0: AFE internal/feed/fetch (5) preempts KWS inference (4).
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(dsp_task, "array_das", 8192, nullptr, 6, &dsp_task_handle, 1) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create DAS task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(doa_task, "array_doa", 8192, nullptr, 3, nullptr, 1) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create DOA task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(post_ns_task, "post_afe_ns", 12288, nullptr, 4, &ns_task_handle, 1) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create post-AFE NS task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(afe_feed_task, "afe_feed", 8192, nullptr, 5, nullptr, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create AFE feed task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(afe_fetch_task, "afe_fetch", 4096, nullptr, 5, nullptr, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create AFE fetch task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(diagnostics_stats_task, "array_stats", 4096, nullptr, 1, nullptr, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create DSP statistics task");
#if !CONFIG_WS2812_LED_RING_ENABLE
    if (!uart_is_driver_installed(console_uart))
        ESP_RETURN_ON_ERROR(uart_driver_install(console_uart, 256, 0, 0, nullptr, 0), TAG, "console UART");
    ESP_RETURN_ON_FALSE(xTaskCreate(console_task, "array_console", 3072, nullptr, 2, nullptr) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "create console task");
#endif
    initialized.store(true);
    ESP_LOGI(TAG, "input calibration before STFT: common %.1f dB + 8 channel trims (gain_cali.json)",
             (double)common_input_gain_db);
    ESP_LOGI(TAG, "24-kHz 8mic -> DAS OLA -> 16-kHz mono AFE AEC/VAD/AGC -> NsCore -> Capture2");
    ESP_LOGI(TAG, "post-AFE NsCore core=1 priority=4 min_gain=%d%% (100=bypass), frame=160 samples, OLA delay=%d samples",
             CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT,
             CONFIG_ARRAY_POST_NS_MIN_GAIN_PERCENT == 100 ? 0 : 96);
#if CONFIG_ARRAY_CAPTURE2_RIGHT_COPY_LEFT
    ESP_LOGI(TAG, "Capture2 right channel: copy left audio");
#elif CONFIG_ARRAY_CAPTURE2_RIGHT_DOA
    ESP_LOGI(TAG, "Capture2 right channel: smoothed DAS DOA, PCM=round(deg*100-18000)");
#else
    ESP_LOGI(TAG, "Capture2 right channel: VAD flag");
#endif
    ESP_LOGI(TAG, "VAD vadnet1_medium mode=1 min_speech=%d ms min_noise=%d ms; continuous USB/KWS audio",
             afe_vad_min_speech_ms, afe_vad_min_noise_ms);
    ESP_LOGI(TAG, "task placement: core1 DAS/OLA/resample=6, post-AFE NS=4, DOA=3; core0 AFE=5, KWS=4");
#if CONFIG_ARRAY_CAPTURE2_SINE_TEST
    ESP_LOGW(TAG, "Capture2 diagnostic: 1-kHz table tone replaces AEC output; AEC still runs");
#elif CONFIG_ARRAY_CAPTURE2_POST_NS_SINE_TEST
    ESP_LOGW(TAG, "Capture2 diagnostic: 1-kHz sine after NsCore; normal NS cadence/metadata and KWS input retained");
#elif CONFIG_ARRAY_CAPTURE2_PLAYBACK_TEST
    ESP_LOGW(TAG, "Capture2 diagnostic: USB playback mono reference replaces AEC output; AEC still runs");
#endif
    ESP_LOGI(TAG, "DOA fixed SRP-PHAT 2Hz, radius=%d mm, result log off; UART e=log on, d=log off",
             CONFIG_ARRAY_RADIUS_MM);
    ESP_LOGI(TAG, "DOA follows speech VAD only, shortest-path steering limit=%d deg/s",
             CONFIG_ARRAY_DOA_ROTATION_SPEED_DEG_PER_SEC);
    return ESP_OK;
}
extern "C" void audio_pipeline_capture(const int16_t *samples, size_t frames) {
    if (!initialized.load() || custom_maintenance_active()) return;
    // Capture task passes <= one USB millisecond at a time.
    while (frames) {
        size_t count = std::min(frames, size_t(24));
        send_whole_wait(input_stream, samples,
                        count * input_channels * sizeof(int16_t), capture_waits);
        const size_t buffered = xStreamBufferBytesAvailable(input_stream);
        if (buffered > input_peak_bytes.load(std::memory_order_relaxed))
            input_peak_bytes.store(buffered, std::memory_order_relaxed);
        frames -= count;
        samples += count * channels;
    }
}
extern "C" void audio_pipeline_reference(const int16_t *stereo, size_t frames) {
    if (!initialized.load() || custom_maintenance_active()) return;
    uint64_t square_sum = 0;
    portENTER_CRITICAL(&reference_lock);
    for (size_t i = 0; i < frames; ++i) {
        if (reference_count == reference_capacity) {
            reference_read = (reference_read + 1) % reference_capacity;
            --reference_count;
            ++reference_drops;
        }
        const int32_t mono = (int32_t(stereo[2 * i]) + stereo[2 * i + 1]) / 2;
        reference_fifo[reference_write] = static_cast<int16_t>(mono);
        reference_write = (reference_write + 1) % reference_capacity;
        ++reference_count;
        square_sum += static_cast<uint64_t>(static_cast<int64_t>(mono) * mono);
    }
    portEXIT_CRITICAL(&reference_lock);
    playback_level_dbfs.store(pcm_rms_dbfs(square_sum, frames),
                              std::memory_order_relaxed);
    playback_level_tick.store(static_cast<uint32_t>(xTaskGetTickCount()),
                              std::memory_order_release);
}
extern "C" size_t audio_pipeline_read(audio_pipeline_capture2_frame_t *frames, size_t count) {
    if (!initialized.load()) return 0;
    const bool in_isr = xPortInIsrContext();
    BaseType_t task_woken = pdFALSE;
    size_t bytes;
    if (in_isr) {
        portENTER_CRITICAL_ISR(&output_consumer_lock);
        bytes = xStreamBufferReceiveFromISR(output_stream, frames, count * sizeof(*frames), &task_woken);
        portEXIT_CRITICAL_ISR(&output_consumer_lock);
    } else {
        portENTER_CRITICAL(&output_consumer_lock);
        bytes = xStreamBufferReceive(output_stream, frames, count * sizeof(*frames), 0);
        portEXIT_CRITICAL(&output_consumer_lock);
    }
    if (task_woken) portYIELD_FROM_ISR();
    return bytes / sizeof(*frames);
}
extern "C" size_t audio_pipeline_available(void) {
    return initialized.load() ? xStreamBufferBytesAvailable(output_stream) / sizeof(audio_pipeline_capture2_frame_t) : 0;
}
extern "C" void audio_pipeline_get_capture2_stats(audio_pipeline_capture2_stats_t *stats) {
    if (!stats) return;
    stats->queued_frames = capture2_queued_frames.load(std::memory_order_relaxed);
    stats->queued_zero_pairs = capture2_queued_zero_pairs.load(std::memory_order_relaxed);
}
extern "C" int16_t audio_pipeline_get_doa_sample(void) {
    return static_cast<int16_t>(doa_sample.load(std::memory_order_relaxed));
}
extern "C" audio_pipeline_vad_state_t audio_pipeline_get_vad_state(void) {
    return static_cast<audio_pipeline_vad_state_t>(vad_state.load(std::memory_order_relaxed));
}
extern "C" void audio_pipeline_set_recording(bool active) {
    recording.store(false);
    if (active && initialized.load()) {
        audio_pipeline_capture2_frame_t discarded[64];
        while (audio_pipeline_read(discarded, 64) != 0) {}
    }
    recording.store(active);
}
extern "C" void audio_pipeline_get_levels(audio_pipeline_levels_t *levels) {
    if (!levels) return;
    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    const uint32_t processed_tick = processed_level_tick.load(std::memory_order_acquire);
    const uint32_t playback_tick =
        playback_level_tick.load(std::memory_order_acquire);
    levels->processed_dbfs = processed_tick != 0U && now - processed_tick <= level_stale_ticks
                           ? processed_level_dbfs.load(std::memory_order_relaxed)
                           : silence_dbfs;
    levels->playback_dbfs =
        playback_tick != 0U && now - playback_tick <= level_stale_ticks
            ? playback_level_dbfs.load(std::memory_order_relaxed)
            : silence_dbfs;
}
extern "C" bool audio_pipeline_console_command(uint8_t command) {
    if (custom_control_console_command(command)) return true;
    if (command == 'p' || command == 'P') {
        audio_diagnostics_set_kws_logits_enabled(!audio_diagnostics_kws_logits_enabled());
        audio_diagnostics_print_kws_logits_state();
        return true;
    }
    if (command == 'l' || command == 'L') {
        const bool enabled = !audio_diagnostics_logs_enabled();
        audio_diagnostics_set_logs_enabled(enabled);
        audio_diagnostics_print_log_state();
        return true;
    }
    if (command == 'e' || command == 'E' || command == 'd' || command == 'D') {
        const bool enabled = command == 'e' || command == 'E';
        doa_log_enabled.store(enabled, std::memory_order_relaxed);
        ESP_LOGI(TAG, "DOA result logging %s", enabled ? "enabled" : "disabled");
        return true;
    }
    return false;
}

#ifdef ESP_PLATFORM
extern "C" void audio_pipeline_maintenance_reset(void) {
    if (!initialized.load()) return;
    // All stream producers/consumers (including raw capture) acknowledged pause.
    xStreamBufferReset(input_stream); xStreamBufferReset(afe_stream);
    xQueueReset(ns_queue);
    afe->reset_buffer(afe_data);
    portENTER_CRITICAL(&reference_lock);
    reference_read=reference_write=reference_count=0;
    portEXIT_CRITICAL(&reference_lock);
    stats_ready=false;
    publish_vad_state(AUDIO_PIPELINE_VAD_UNKNOWN);
    post_ns_discontinuity();
    // USB remains the sole Capture2 consumer: it discards stale data on resume.
}

#endif
