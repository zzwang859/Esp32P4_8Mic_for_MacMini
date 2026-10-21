// Exercise the actual runtime with deterministic FreeRTOS/model stand-ins.
// Hardware numerics and real scheduling are checked separately on the board.
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <vector>
#include "../kws_stream.cpp"

namespace {
struct Done {};
std::deque<kws_stream::Block> queued;
std::vector<int> detected;
struct RawLogits { uint32_t sequence; std::vector<float> values; };
std::vector<RawLogits> raw_logits;
int64_t clock_us = 0;
configRUN_TIME_COUNTER_TYPE task_runtime = 0;
unsigned model_calls = 0, model_resets = 0, worker_delays = 0;
unsigned action_rejections = 2, rejected_actions = 0, rejected_log_results = 0;
#ifndef KWS_TEST_PROCESS_US
#define KWS_TEST_PROCESS_US 7000
#endif
constexpr uint32_t process_us = KWS_TEST_PROCESS_US;
int64_t last_worker_delay_us = 0, max_worker_active_us = 0;
bool quiet = false, fail_create = false, fail_task = false, fail_process = false;
bool trace_enabled = true;
bool current_vad_high = true, flip_vad_during_inference = false;
int current_class = 2;
float current_logit = 4.0f;
char last_log[512];
std::vector<std::string> log_lines;
struct Action { unsigned blocks; int label; bool gap = false; bool error = false; float logit = 4.0f; bool vad_high = true; };
std::vector<Action> actions;
size_t next_action = 0;

void feed_blocks(unsigned count) {
    int16_t pcm[160];
    const int16_t extremes[] = {-32768, 32767, -1, 0};
    for (size_t i = 0; i < 160; ++i) pcm[i] = extremes[i % 4];
    for (unsigned i = 0; i < count; ++i) {
#if CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES
        kws_stream_feed_with_vad(pcm, 160, current_vad_high);
#else
        kws_stream_feed(pcm, 160);
#endif
    }
}
}

void *heap_caps_malloc(size_t n, unsigned caps) {
    assert(caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return std::malloc(n);
}
void heap_caps_free(void *p) { std::free(p); }
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned bytes, uint8_t *, StaticQueue_t *) {
    assert(count == 32 && bytes == sizeof(kws_stream::Block));
    return &queued;
}
void vQueueDelete(QueueHandle_t q) { assert(q == &queued); queued.clear(); }
BaseType_t xQueueSend(QueueHandle_t q, const void *p, TickType_t ticks) {
    assert(q == &queued && ticks == 0); // No producer backpressure allowed.
    if (queued.size() == 32) return 0;
    queued.push_back(*static_cast<const kws_stream::Block *>(p));
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *p, TickType_t ticks) {
    assert(q == &queued && ticks == portMAX_DELAY);
    if (queued.empty()) {
        if (next_action == actions.size()) throw Done{};
        const auto &a = actions[next_action++];
        current_class = a.label;
        current_logit = a.logit;
        current_vad_high = a.vad_high;
        fail_process = a.error;
        if (a.gap) {
            int16_t partial[159]{};
            kws_stream_feed(partial, 159);
            kws_stream_discontinuity();
        }
        feed_blocks(a.blocks);
    }
    assert(!queued.empty());
    *static_cast<kws_stream::Block *>(p) = queued.front();
    queued.pop_front();
    return pdTRUE;
}
void vTaskDelay(TickType_t ticks) {
    assert(ticks == 2);
    const int64_t active_us = clock_us - last_worker_delay_us;
    max_worker_active_us = std::max(max_worker_active_us, active_us);
    // Frame-boundary recovery may exceed the time limit by at most one
    // inference. In particular, a single slow inference must cause a pause.
    assert(active_us >= max_active_window_us);
    assert(active_us < max_active_window_us + process_us);
    // Simulate calling delay just before a tick: two ticks still provide
    // >1 ms. Task CPU time deliberately excludes this blocked interval.
    clock_us += ticks * 1000 - 999;
    last_worker_delay_us = clock_us;
    ++worker_delays;
}
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name, unsigned stack,
                                 void *, unsigned priority, TaskHandle_t *handle, int core) {
    assert(entry == inference_task && std::string(name) == "kws_infer");
    assert(stack == 8192 && priority == 4 && core == 0);
    if (!fail_task) *handle = &task_runtime;
    return fail_task ? 0 : pdPASS;
}
void vTaskGetInfo(TaskHandle_t handle, TaskStatus_t *info, BaseType_t scan, eTaskState state) {
    assert(handle == &task_runtime);
    assert(scan == pdFALSE && state == eReady);
    info->ulRunTimeCounter = task_runtime;
}
int64_t esp_timer_get_time() { return clock_us; }
void test_log(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(last_log, sizeof(last_log), fmt, args);
    va_end(args);
    log_lines.emplace_back(last_log);
}
extern "C" bool audio_diagnostics_info_enabled() { return !quiet; }
extern "C" bool audio_diagnostics_kws_logits_enabled() { return trace_enabled; }
extern "C" void audio_diagnostics_kws_logits(uint32_t sequence, const float *values) {
    raw_logits.push_back({sequence, std::vector<float>(values, values + KWS_CLASS_COUNT)});
}
extern "C" bool audio_diagnostics_result(const char *tag, const char *message, int index) {
    assert(std::string(tag) == "kws");
    assert(std::string(message) == "keyword class=");
    assert(index >= 2 && index < KWS_CLASS_COUNT); // Background stays internal.
    // Logs can be muted or saturated independently of the action queue.
    ++rejected_log_results;
    return false;
}
extern "C" bool keyword_actions_submit(int index) {
    assert(index >= 2 && index < KWS_CLASS_COUNT);
    if (action_rejections) { --action_rejections; ++rejected_actions; return false; }
    detected.push_back(index);
    return true;
}
extern "C" esp_err_t kws_create(kws_handle_t *p) { *p = fail_create ? nullptr : &queued; return fail_create ? 2 : 0; }
extern "C" void kws_destroy(kws_handle_t *p) { *p = nullptr; }
extern "C" esp_err_t kws_reset(kws_handle_t p) { assert(p == &queued); ++model_resets; return ESP_OK; }
extern "C" esp_err_t kws_process(kws_handle_t p, const int32_t *pcm, size_t n, float *logits, int *index) {
    assert(p == &queued && n == 160);
    const int32_t extremes[] = {INT32_MIN, 2147418112, -65536, 0};
    for (size_t i = 0; i < n; ++i) assert(pcm[i] == extremes[i % 4]);
    ++model_calls;
    clock_us += process_us;
    task_runtime += 2500; // Simulate preemption: 7 ms wall, 2.5 ms actual task CPU.
    std::fill(logits, logits + KWS_CLASS_COUNT, 0);
    logits[current_class] = current_logit;
    *index = current_class;
    if (flip_vad_during_inference) current_vad_high = !current_vad_high;
    if (fail_process) { fail_process = false; return 3; }
    return ESP_OK;
}

int main() {
    // Feeding before initialization is harmless. Failed starts free their
    // queue and model and leave USB/AFE unaffected; a later start can succeed.
    feed_blocks(1);
    assert(queued.empty());
    fail_create = true;
    assert(kws_stream_start() == ESP_ERR_NO_MEM && !input_queue && !input_storage && !model);
    fail_create = false;
    fail_task = true;
    assert(kws_stream_start() == ESP_ERR_NO_MEM && !input_queue && !input_storage && !model);
    fail_task = false;
    assert(kws_stream_start() == ESP_OK);
    assert(kws_stream_start() == ESP_ERR_INVALID_STATE);

    actions = {{25, 2}, {25, 2}, {25, 2}, {25, 2}, {3, 2}};
    // Separate words by confident background for longer than the cooldown;
    // background changes rearm same-word suppression without output events.
    for (int label : {6, 4, 5, 7, 6, 6, 4, 3, 6}) {
        for (int i = 0; i < 3; ++i) actions.push_back({25, 0});
        actions.push_back({3, label});
    }
    for (int i = 0; i < 3; ++i) actions.push_back({25, 0});
    actions.push_back({8, 6, false, false, 0.001f}); // Weak top-1 is not a trigger.
    actions.push_back({3, 6});
    for (int i = 0; i < 3; ++i) actions.push_back({25, 0});
    actions.push_back({34, 2}); // Queue overflow still exposes 2 missing blocks.
    for (int i = 0; i < 4; ++i) actions.push_back({25, 2}); // gap reset, warmup -> yes
    actions.push_back({1, 2, false, true});           // error reset
    for (int i = 0; i < 4; ++i) actions.push_back({25, 2}); // warmup -> yes
    actions.push_back({25, 7, true});                // AFE gap discards partial
    for (int i = 0; i < 3; ++i) actions.push_back({25, 7}); // warmup -> right
    try { inference_task(nullptr); } catch (Done &) {}
    assert((detected == std::vector<int>{2, 6, 4, 5, 7, 6, 6, 4, 3, 6, 6, 2, 2, 2, 7}));
    assert(action_rejections == 0 && rejected_actions == 2);
    assert(rejected_log_results == detected.size()); // Failed logs never repeat/drop actions.
    kws_stream_stats_t stats;
    kws_stream_get_stats(nullptr);
    kws_stream_get_stats(&stats);
    assert(stats.chunks == model_calls - 1 && stats.process_us == stats.chunks * process_us);
    assert(stats.max_us == process_us && stats.dropped_samples == 320);
    assert(stats.errors == 1 && stats.resets == 3 && model_resets == 3);
    assert(raw_logits.size() == stats.chunks && raw_logits.size() > detected.size() * 20);
    bool saw_weak = false, saw_background = false, saw_up = false, saw_down = false;
    for (size_t i = 0; i < raw_logits.size(); ++i) {
        if (i) assert(raw_logits[i].sequence > raw_logits[i - 1].sequence);
        for (float value : raw_logits[i].values) saw_weak |= value == 0.001f;
        saw_up |= raw_logits[i].values[4] == 4.0f;
        saw_down |= raw_logits[i].values[5] == 4.0f;
        saw_background |= raw_logits[i].values[0] == 4.0f;
    }
    assert(saw_weak && saw_background && saw_up && saw_down);
    constexpr unsigned calls_per_pause = (max_active_window_us + process_us - 1) / process_us;
    assert(worker_delays == model_calls / calls_per_pause);
    assert(max_worker_active_us > 0 && max_worker_active_us < max_active_window_us + process_us);
    quiet = true;
    const uint64_t retained = window_chunks;
    const size_t logs_before = log_lines.size();
    kws_stream_log_stats();
    assert(window_chunks == retained && retained == stats.chunks);
    assert(log_lines.size() == logs_before);
    assert(totals.task_cpu_us == model_calls * 2500 && window_cpu_us == totals.task_cpu_us);
    quiet = false;
    kws_stream_log_stats();
    assert(window_chunks == 0 && window_us == 0 && window_max_us == 0 && window_cpu_us == 0);
    assert(log_lines.size() == logs_before + 2);
    char expected_process_rtf[64];
    std::snprintf(expected_process_rtf, sizeof(expected_process_rtf), "RTF=%.3f", double(process_us) / 10000);
    assert(log_lines[logs_before].find(expected_process_rtf) != std::string::npos);
    assert(log_lines[logs_before].find("dropped=320") != std::string::npos);
    char expected_rtf[64];
    std::snprintf(expected_rtf, sizeof(expected_rtf), "task_RTF=%.3f", double(model_calls * 2500) / (retained * 10000));
    assert(std::strstr(last_log, expected_rtf));
    char expected_core_share[64];
    std::snprintf(expected_core_share, sizeof(expected_core_share), "core0_share=%.1f%%",
                  100.0 * model_calls * 2500 / clock_us);
    assert(std::strstr(last_log, expected_core_share));
    kws_stream_get_stats(&stats);
    assert(stats.chunks == retained); // Window reports never clear lifetime totals.
    assert(stats.task_cpu_us == model_calls * 2500);
    // Unsigned delta must preserve a 32-bit task timer rollover. Sample while
    // muted too, so a long USB recording cannot hide the rollover.
    last_task_runtime = UINT32_MAX - 499;
    task_runtime = 250;
    quiet = true;
    kws_stream_log_stats();
    kws_stream_get_stats(&stats);
    assert(window_cpu_us == 750 && stats.task_cpu_us == model_calls * 2500 + 750);
    // Disabling raw trace skips the debug callback while inference and
    // keyword confirmation continue normally.
    trace_enabled = false;
    const size_t raw_before = raw_logits.size(), detected_before = detected.size();
    const unsigned calls_before = model_calls;
    actions.assign(4, {25, 2});
    next_action = 0;
    last_worker_delay_us = clock_us;
    try { inference_task(nullptr); } catch (Done &) {}
    assert(model_calls == calls_before + 100);
    assert(raw_logits.size() == raw_before);
    assert(detected.size() == detected_before + 1 && detected.back() == 2);
#if CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES
    const size_t vad_before = detected.size();
    actions.assign(4, {25, 0, false, false, 4.0f, false});
    actions.push_back({2, 3});
    actions.push_back({1, 3, false, false, 4.0f, false});
    actions.push_back({2, 3});
    actions.push_back({1, 3});
    next_action = 0;
    last_worker_delay_us = clock_us;
    flip_vad_during_inference = true;
    try { inference_task(nullptr); } catch (Done &) {}
    assert(detected.size() == vad_before + 1 && detected.back() == 3);
#endif
    std::puts("PASS: actual runtime queue, PCM, reset, bounded catch-up/recovery, wall/task RTF, quiet CPU sampling, timer wrap, init cleanup");
}
