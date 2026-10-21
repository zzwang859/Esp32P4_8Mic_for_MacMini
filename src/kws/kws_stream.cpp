#include "kws_stream.h"
#ifdef ESP_PLATFORM
#include "custom_control.h"
#endif
#include "kws_stream_state.hpp"
#include "audio_diagnostics.h"
#include "keyword_actions.h"
#include <atomic>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {
constexpr const char *TAG = "kws";
constexpr unsigned queue_blocks = 32;  // 320 ms, independent of USB FIFOs.
constexpr uint64_t chunk_us = KWS_CHUNK_SAMPLES * 1000000ULL / kws_model::sample_rate;
constexpr int64_t max_active_window_us = 20000;
// One tick can expire immediately at the next tick boundary. At 1 kHz,
// two ticks leave at least one full millisecond for lower-priority tasks.
constexpr TickType_t recovery_ticks = 2;
static_assert(chunk_us == 10000);
static_assert(KWS_CLASS_COUNT == AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT);
QueueHandle_t input_queue;
StaticQueue_t input_control;
uint8_t *input_storage;
kws_handle_t model;
TaskHandle_t inference_task_handle;
configRUN_TIME_COUNTER_TYPE last_task_runtime = 0;
uint64_t window_cpu_us = 0;
int64_t window_started_us = 0;
std::atomic<bool> ready{false};
kws_stream::Framer framer;
portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;
kws_stream_stats_t totals{};
uint64_t window_chunks = 0, window_us = 0;
uint32_t window_max_us = 0;

void inference_task(void *) {
#ifdef ESP_PLATFORM
    custom_worker_register(CUSTOM_WORK_KWS);
#endif
    kws_stream::Block block;
    kws_stream::Debouncer debounce;
#ifdef ESP_PLATFORM
    uint32_t filter_revision = 0;
#endif
    int32_t pcm[KWS_CHUNK_SAMPLES];
    float logits[KWS_MAX_CLASS_COUNT]{};
#ifdef ESP_PLATFORM
    unsigned warmup_frames=0;
#endif
    bool have_sequence = false;
    uint32_t expected_sequence = 0;
    int64_t last_pause_us = esp_timer_get_time();
    while (true) {
#ifdef ESP_PLATFORM
        if (custom_worker_checkpoint(CUSTOM_WORK_KWS)) {
            debounce=kws_stream::Debouncer(); have_sequence=false; warmup_frames=0;
        }
        if (xQueueReceive(input_queue, &block, pdMS_TO_TICKS(10)) != pdTRUE) continue;
        if (custom_maintenance_active() || !model) continue;
        const int class_count=custom_class_count();
#else
        if (xQueueReceive(input_queue, &block, portMAX_DELAY) != pdTRUE) continue;
        const int class_count=KWS_CLASS_COUNT;
#endif
        if (have_sequence && block.sequence != expected_sequence) {
            kws_reset(model);
            debounce.reset();
#ifdef ESP_PLATFORM
            warmup_frames=0;
            custom_set_runtime_state(2, ESP_OK);
#endif
            portENTER_CRITICAL(&stats_lock);
            ++totals.resets;
            portEXIT_CRITICAL(&stats_lock);
        }
        have_sequence = true;
        expected_sequence = block.sequence + 1;
        for (size_t i = 0; i < KWS_CHUNK_SAMPLES; ++i)
            pcm[i] = static_cast<int32_t>(block.pcm[i]) * 65536;
        int index = -1;
        const int64_t started = esp_timer_get_time();
        const esp_err_t err = kws_process(model, pcm, KWS_CHUNK_SAMPLES, logits, &index);
        const uint32_t elapsed = static_cast<uint32_t>(esp_timer_get_time() - started);
        portENTER_CRITICAL(&stats_lock);
        if (err == ESP_OK) {
            ++totals.chunks;
            totals.process_us += elapsed;
            totals.max_us = std::max(totals.max_us, elapsed);
            ++window_chunks;
            window_us += elapsed;
            window_max_us = std::max(window_max_us, elapsed);
        } else {
            ++totals.errors;
            ++totals.resets;
        }
        portEXIT_CRITICAL(&stats_lock);
#ifdef ESP_PLATFORM
        if (custom_maintenance_active()) continue;
#endif
        if (err != ESP_OK) {
            kws_reset(model);
            debounce.reset();
#ifdef ESP_PLATFORM
            warmup_frames=0;
            custom_set_runtime_state(2, err);
#endif
        } else {
            // Every successful 10-ms inference, including warmup, background
            // and rejected candidates. The UART worker formats raw values.
            if (audio_diagnostics_kws_logits_enabled()) {
#ifdef ESP_PLATFORM
                audio_diagnostics_kws_logits_n(block.sequence, logits, class_count);
#else
                audio_diagnostics_kws_logits(block.sequence, logits);
#endif
            }
            auto filter = kws_model::default_filter;
#ifdef ESP_PLATFORM
            custom_kws_filter_t runtime_filter;
            uint32_t revision;
            custom_kws_filter_get(&runtime_filter, &revision);
            if (revision != filter_revision) {
                debounce.config_changed();
                filter_revision = revision;
            }
            filter = {runtime_filter.keyword_threshold_percent, runtime_filter.cooldown_ms,
                      runtime_filter.stable_frames, runtime_filter.need_vad_checked != 0};
#endif
            const int detected = debounce.update(index, logits, esp_timer_get_time(), class_count,
                                                 block.vad_high, filter);
            if (detected >= 0) {
                const bool accepted = detected < 2 || keyword_actions_submit(detected);
                if (accepted && detected >= 2) {
                    (void)audio_diagnostics_result(TAG, "keyword class=", detected);
                }
                debounce.acknowledge(accepted, esp_timer_get_time());
            }
        }
#ifdef ESP_PLATFORM
        if (err==ESP_OK && warmup_frames<kws_model::warmup_chunks &&
            ++warmup_frames==kws_model::warmup_chunks) custom_set_runtime_state(3, ESP_OK);
#endif
        // Bound catch-up bursts by wall time, including AFE preemption. A
        // 32-frame counter plus a one-tick delay could repeatedly miss IDLE0
        // when the same-core USB reference producer wakes in AFE-sized bursts.
        if (esp_timer_get_time() - last_pause_us >= max_active_window_us) {
            vTaskDelay(recovery_ticks);
            last_pause_us = esp_timer_get_time();
        }
    }
}
}  // namespace

extern "C" esp_err_t kws_stream_start(void) {
    if (input_queue) return ESP_ERR_INVALID_STATE;
    input_storage = static_cast<uint8_t *>(heap_caps_malloc(
        queue_blocks * sizeof(kws_stream::Block), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!input_storage) return ESP_ERR_NO_MEM;
    input_queue = xQueueCreateStatic(queue_blocks, sizeof(kws_stream::Block), input_storage, &input_control);
    esp_err_t err = input_queue ? kws_create(&model) : ESP_ERR_NO_MEM;
    if (err == ESP_OK) {
        window_started_us = esp_timer_get_time();
        if (xTaskCreatePinnedToCore(inference_task, "kws_infer", 8192,
                nullptr, 4, &inference_task_handle, 0) != pdPASS) err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK) {
        kws_destroy(&model);
        if (input_queue) vQueueDelete(input_queue);
        input_queue = nullptr;
        heap_caps_free(input_storage);
        input_storage = nullptr;
        return err;
    }
    ready.store(true, std::memory_order_release);
#ifdef ESP_PLATFORM
    custom_set_runtime_state(2, ESP_OK);
#endif
#if CONFIG_KWS_INPUT_USB_PLAYBACK_REFERENCE
    constexpr const char *input_source = "USB playback AEC reference";
#else
    constexpr const char *input_source = "post-AFE NsCore";
#endif
    ESP_LOGI(TAG, "%s 16-kHz mono -> 160-sample KWS, core=0 priority=4, warmup=%u ms stable=%u chunks probability>=%d%% cooldown=%d ms need_vad_checked=%s, runtime-class softmax + repeated-class suppression",
             input_source,
             kws_model::warmup_chunks * 10, kws_model::stable_chunks,
             CONFIG_KWS_FILTER_KEYWORD_THRESHOLD_PERCENT, CONFIG_KWS_FILTER_COOLDOWN_MS,
             kws_model::need_vad_checked ? "Yes" : "No");
    return ESP_OK;
}

extern "C" void kws_stream_feed(const int16_t *pcm, size_t samples) {
    kws_stream_feed_with_vad(pcm, samples, false);
}

extern "C" void kws_stream_feed_with_vad(const int16_t *pcm, size_t samples, bool vad_high) {
    if (!ready.load(std::memory_order_acquire) || !pcm) return;
#ifdef ESP_PLATFORM
    if (custom_maintenance_active()) return;
#endif
    framer.feed(pcm, samples, [](const kws_stream::Block &block) {
        if (xQueueSend(input_queue, &block, 0) != pdTRUE) {
            portENTER_CRITICAL(&stats_lock);
            totals.dropped_samples += KWS_CHUNK_SAMPLES;
            portEXIT_CRITICAL(&stats_lock);
        }
    }, vad_high);
}

extern "C" void kws_stream_discontinuity(void) {
    if (ready.load(std::memory_order_acquire)) framer.discontinuity();
}

extern "C" void kws_stream_get_stats(kws_stream_stats_t *stats) {
    if (!stats) return;
    portENTER_CRITICAL(&stats_lock);
    *stats = totals;
    portEXIT_CRITICAL(&stats_lock);
}

extern "C" void kws_stream_log_stats(void) {
    if (!ready.load(std::memory_order_acquire)) return;
    // Background sampling avoids adding scheduler queries to every inference.
    // FreeRTOS counters are committed at context switches: a window boundary
    // can lag by one running slice, so interpret these as window estimates.
    TaskStatus_t task_info{};
    // eReady avoids a task-state traversal; pdFALSE skips stack scanning.
    vTaskGetInfo(inference_task_handle, &task_info, pdFALSE, eReady);
    const configRUN_TIME_COUNTER_TYPE runtime = task_info.ulRunTimeCounter;
    const configRUN_TIME_COUNTER_TYPE cpu_delta = runtime - last_task_runtime;
    last_task_runtime = runtime;
    portENTER_CRITICAL(&stats_lock);
    totals.task_cpu_us += cpu_delta;
    window_cpu_us += cpu_delta;
    portEXIT_CRITICAL(&stats_lock);
    // Sample even while muted so a 32-bit counter cannot wrap unnoticed over
    // a long recording. Keep both wall and CPU totals until the same report.
    if (!audio_diagnostics_info_enabled()) return;
    kws_stream_stats_t stats;
    uint64_t chunks, process_us, cpu_us;
    uint32_t max_us;
    portENTER_CRITICAL(&stats_lock);
    stats = totals;
    chunks = window_chunks;
    process_us = window_us;
    max_us = window_max_us;
    cpu_us = window_cpu_us;
    window_cpu_us = 0;
    window_chunks = window_us = 0;
    window_max_us = 0;
    portEXIT_CRITICAL(&stats_lock);
    const int64_t now = esp_timer_get_time();
    const int64_t interval_us = now - window_started_us;
    window_started_us = now;
    if (!chunks && !stats.errors && !stats.dropped_samples) return;
    ESP_LOGI(TAG, "perf chunks=%llu avg=%.3f ms max=%.3f ms RTF=%.3f dropped=%lu resets=%lu errors=%lu",
        (unsigned long long)chunks, chunks ? double(process_us) / chunks / 1000.0 : 0.0,
        double(max_us) / 1000.0, chunks ? double(process_us) / (chunks * chunk_us) : 0.0,
        (unsigned long)stats.dropped_samples, (unsigned long)stats.resets, (unsigned long)stats.errors);
    ESP_LOGI(TAG, "cpu task_avg=%.3f ms task_RTF=%.3f core0_share=%.1f%% interval=%.3f s (whole-task estimate)",
        chunks ? double(cpu_us) / chunks / 1000.0 : 0.0,
        chunks ? double(cpu_us) / (chunks * chunk_us) : 0.0,
        interval_us > 0 ? 100.0 * cpu_us / interval_us : 0.0, double(interval_us) / 1000000.0);
}

#ifdef ESP_PLATFORM
extern "C" void kws_stream_maintenance_reset(void) {
    if (!custom_maintenance_active()) return;
    if (input_queue) xQueueReset(input_queue);
    framer.discontinuity();
    if (model) {
        esp_err_t err=kws_reset(model);
        custom_set_runtime_state(err==ESP_OK ? 2 : 4,err);
    }
}
extern "C" esp_err_t kws_stream_reload(void) {
    if (!custom_maintenance_active()) return ESP_ERR_INVALID_STATE;
    if (!input_queue) return kws_stream_start();
    custom_set_runtime_state(1, ESP_OK);
    kws_destroy(&model);
    xQueueReset(input_queue); framer.discontinuity();
    esp_err_t err=kws_create(&model);
    if (err==ESP_OK) {
        int32_t silence[KWS_CHUNK_SAMPLES]{}; float logits[KWS_MAX_CLASS_COUNT]{}; int index;
        for (unsigned i=0;i<3 && err==ESP_OK;++i) err=kws_process(model,silence,KWS_CHUNK_SAMPLES,logits,&index);
        if (err==ESP_OK) err=kws_reset(model);
    }
    custom_set_runtime_state(err==ESP_OK ? 2 : 4,err);
    return err;
}
#endif
