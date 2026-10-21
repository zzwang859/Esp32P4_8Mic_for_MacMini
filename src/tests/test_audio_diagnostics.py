"""Exercise the actual diagnostics module with mocked UART/FreeRTOS.

Run with --cc <host C compiler>. Both quiet-mode configurations are tested.
No ESP-IDF installation or board is needed.
"""

import argparse
from pathlib import Path
import subprocess
import tempfile


HEADERS = {
    "esp_err.h": """#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define ESP_ERR_NO_MEM 2
""",
    "esp_log_level.h": """#pragma once
typedef enum { ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO,
               ESP_LOG_DEBUG, ESP_LOG_VERBOSE } esp_log_level_t;
""",
    "esp_log.h": """#pragma once
#include <stdarg.h>
#include <stdint.h>
#include "esp_log_level.h"
typedef int (*vprintf_like_t)(const char *, va_list);
vprintf_like_t esp_log_set_vprintf(vprintf_like_t);
uint32_t esp_log_timestamp(void);
""",
    "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef int portMUX_TYPE;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
extern unsigned mock_yields;
#define portYIELD_FROM_ISR() (++mock_yields)
/* Single-threaded mocks verify queue/dispatch behavior, not CPU atomics. */
#define __ATOMIC_RELAXED 0
#define __atomic_load_n(p,o) (*(p))
#define __atomic_store_n(p,v,o) (*(p) = (v))
#define __atomic_fetch_add(p,v,o) ((*(p) += (v)) - (v))
#define __atomic_fetch_or(p,v,o) (*(p) |= (v))
#define __atomic_fetch_and(p,v,o) (*(p) &= (v))
static inline uint32_t mock_exchange(uint32_t *p, uint32_t v) {
    uint32_t old = *p; *p = v; return old;
}
#define __atomic_exchange_n(p,v,o) mock_exchange(p,v)
""",
    "freertos/queue.h": """#pragma once
#include "FreeRTOS.h"
typedef struct MockQueue *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned, unsigned);
void vQueueDelete(QueueHandle_t);
BaseType_t xQueueReset(QueueHandle_t);
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t);
BaseType_t xQueueSendFromISR(QueueHandle_t, const void *, BaseType_t *);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
""",
    "freertos/task.h": """#pragma once
#include "FreeRTOS.h"
int xPortInIsrContext(void);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t);
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, unsigned,
                                  void *, unsigned, TaskHandle_t *, int);
void xTaskNotifyGive(TaskHandle_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
""",
    "sdkconfig.h": "#pragma once\n",
    "driver/uart.h": """#pragma once
#include <stdbool.h>
#include <stddef.h>
bool uart_is_driver_installed(int port);
int uart_write_bytes(int port, const void *data, size_t size);
""",
    "sys/reent.h": "#pragma once\nstruct _reent { int unused; };\nstruct _reent *__getreent(void);\n",
}


HARNESS = r"""
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#undef assert
#define assert(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAILED line %d: %s\n", __LINE__, #expr); exit(1); \
} } while (0)
#if defined(__TINYC__) && defined(_WIN32)
/* TCC's legacy MSVCRT returns -1 on truncation; ESP-IDF uses C99 semantics. */
static int c99_vsnprintf(char *buffer, size_t size, const char *format, va_list args) {
    va_list copy;
    va_copy(copy, args);
    int needed = _vscprintf(format, copy);
    va_end(copy);
    vsnprintf(buffer, size, format, args);
    if (size) buffer[size - 1] = 0;
    return needed;
}
#define vsnprintf c99_vsnprintf
#endif
#include "audio_diagnostics.c"
#include "audio_console_gate.c"

struct MockQueue {
    unsigned head, count, length, item_size;
    unsigned char *items;
};
static jmp_buf worker_done;
static char output[16384];
static unsigned output_size, uart_calls, isr_sends, delays;
static unsigned raw_queue_reads, worker_notifications;
static int in_isr;
static uint32_t ticks;
unsigned mock_yields;
static vprintf_like_t installed_hook;
static unsigned file_writes;
static struct _reent mock_reent;
struct _reent *__getreent(void) { return &mock_reent; }
#if CONFIG_ESP_CONSOLE_UART
static bool driver_ready = true;
static unsigned bulk_uart_calls;
static char last_bulk_line[1024];
bool uart_is_driver_installed(int port) { assert(port == 0); return driver_ready; }
int uart_write_bytes(int port, const void *data, size_t size) {
    assert(port == 0 && size > 1); // One complete line, never per-character I/O.
    assert(size < sizeof(last_bulk_line));
    memcpy(last_bulk_line, data, size);
    last_bulk_line[size] = 0;
    ++bulk_uart_calls;
#if CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF
    char normalized[1024];
    size_t used = 0;
    for (size_t i = 0; i < size; ++i) {
        if (((const char *)data)[i] == '\r' && i + 1 < size && ((const char *)data)[i+1] == '\n') continue;
        normalized[used++] = ((const char *)data)[i];
    }
    return (int)__real__write_r(&mock_reent, STDOUT_FILENO, normalized, used);
#else
    return (int)__real__write_r(&mock_reent, STDOUT_FILENO, data, size);
#endif
}
#endif

ssize_t __real__write_r(struct _reent *r, int fd, const void *data, size_t size) {
    if (fd != STDOUT_FILENO && fd != STDERR_FILENO) { ++file_writes; return (ssize_t)size; }
    assert(output_size + size < sizeof(output));
    memcpy(output + output_size, data, size);
    output_size += size; output[output_size] = 0;
    ++uart_calls;
    return (ssize_t)size;
}

static int uart_sink(const char *fmt, va_list args) {
    char line[2048];
    int count = vsnprintf(line, sizeof(line), fmt, args);
    assert(count >= 0 && (size_t)count < sizeof(line));
    return (int)__wrap__write_r(&mock_reent, STDOUT_FILENO, line, (size_t)count);
}
vprintf_like_t esp_log_set_vprintf(vprintf_like_t hook) {
    installed_hook = hook;
    return uart_sink;
}
uint32_t esp_log_timestamp(void) { return ticks; }
int xPortInIsrContext(void) { return in_isr; }
TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t delay) { delays += delay; }
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name,
    unsigned stack, void *arg, unsigned priority, TaskHandle_t *handle, int core) {
    assert(priority == 1 && core == 1);
    *handle = &worker_done;
    return pdPASS;
}
void xTaskNotifyGive(TaskHandle_t handle) { assert(handle == &worker_done); ++worker_notifications; }
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *woken) {
    assert(handle == &worker_done); *woken = pdTRUE;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks_to_wait) {
    assert(clear && ticks_to_wait == 1000);
    assert(!s_log_queue->count && !s_logits_queue->count);
    longjmp(worker_done, 1);
}
QueueHandle_t xQueueCreate(unsigned length, unsigned item_size) {
    struct MockQueue *q = calloc(1, sizeof(*q));
    q->length = length; q->item_size = item_size;
    q->items = calloc(length, item_size);
    return q;
}
void vQueueDelete(QueueHandle_t q) { free(q->items); free(q); }
BaseType_t xQueueReset(QueueHandle_t q) { q->head = q->count = 0; return pdPASS; }
BaseType_t xQueueSend(QueueHandle_t q, const void *data, TickType_t wait) {
    assert(wait == 0); /* Every producer must use a nonblocking queue send. */
    if (q->count == q->length) return pdFALSE;
    memcpy(q->items + ((q->head + q->count) % q->length) * q->item_size, data, q->item_size);
    ++q->count;
    return pdTRUE;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *data, BaseType_t *woken) {
    ++isr_sends;
    BaseType_t result = xQueueSend(q, data, 0);
    *woken = result;
    return result;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *data, TickType_t wait) {
    assert(wait == 0);
    if (q == s_logits_queue) ++raw_queue_reads;
    if (!q->count) return pdFALSE;
    memcpy(data, q->items + q->head * q->item_size, q->item_size);
    q->head = (q->head + 1) % q->length;
    --q->count;
    return pdTRUE;
}
static int emit(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int result = installed_hook(fmt, args);
    va_end(args);
    return result;
}
static void reset(void) {
    s_log_queue->head = s_log_queue->count = 0;
    s_logits_queue->head = s_logits_queue->count = 0;
    s_logits_enabled = s_logits_generation = s_logits_dropped = s_logits_sent = 0;
    s_stream_mask = s_dropped_logs = s_total_dropped_logs = 0;
    s_log_mode = LOG_MODE_AUTO;
    file_writes = 0;
    raw_queue_reads = worker_notifications = 0;
    in_isr = 0;
    ticks = uart_calls = isr_sends = mock_yields = delays = output_size = 0;
    output[0] = 0;
}
static void drain(void) {
    if (setjmp(worker_done) == 0) diagnostics_task(NULL);
}
int main(void) {
    assert(audio_diagnostics_init() == ESP_OK);
    assert(audio_diagnostics_init() == ESP_ERR_INVALID_STATE);

#if CONFIG_AUDIO_DIAGNOSTICS_CAPTURE2_SUSPEND_UART
    // Queue work before capture opens: the dispatch gate must drop old work,
    // too. All UART paths stay closed even after explicit l/p overrides.
    audio_diagnostics_set_logs_enabled(true);
    audio_diagnostics_set_kws_logits_enabled(true);
    float trace[8] = {1,2,3,4,5,6,7,8};
    emit("W (0) driver: queued before capture\n");
    assert(audio_diagnostics_result("kws", "keyword class=", 2));
    audio_diagnostics_kws_logits(0, trace);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    assert(audio_diagnostics_uart_suspended() && !audio_diagnostics_info_enabled());
    unsigned isolation_notifications = worker_notifications;
    emit("E (0) driver: muted error\n");
    audio_diagnostics_event(ESP_LOG_ERROR, "usb", "muted event", 0, false);
    assert(!audio_diagnostics_result("kws", "keyword class=", 2));
    audio_diagnostics_kws_logits(1, trace);
    assert(worker_notifications == isolation_notifications && s_logits_dropped == 1);
    audio_diagnostics_print_log_state();
    audio_diagnostics_print_kws_logits_state();
    audio_diagnostics_write_kws_line("direct\n", 7);
    assert(__wrap__write_r(&mock_reent, STDOUT_FILENO, "raw\n", 4) == 4);
    assert(__wrap__write_r(&mock_reent, STDERR_FILENO, "raw\n", 4) == 4);
    assert(__wrap__write_r(&mock_reent, 4, "file", 4) == 4 && file_writes == 1);
    drain();
    assert(uart_calls == 0 && output_size == 0 && s_logits_sent == 0);
    // Only Capture2 controls isolation; close it while other audio stays open.
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, true);
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, true);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, false);
    assert(!audio_diagnostics_uart_suspended());
    assert(audio_diagnostics_result("kws", "keyword class=", 2));
    audio_diagnostics_kws_logits(2, trace);
    drain();
    assert(strstr(output, "keyword class= 2") && strstr(output, "KL,2,"));
    assert(!strstr(output, "queued before capture") && !strstr(output, "KL,0,"));
    return 0;
#endif

    /* Stream overlap: stopping playback must not unmute either capture. */
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, true);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, true);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE1, false);
    assert(audio_diagnostics_info_enabled() == !CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING);
    emit("I (%u) cpu_usage: usage %d\n", 0u, 55);
    audio_diagnostics_event(ESP_LOG_INFO, "usb_audio", "started", 2, true);
    assert(s_log_queue->count == (CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING ? 0 : 2));
    emit("\033[0;31mE (%u) i2s: disabled\033[0m\n", 0u);
    assert(uart_calls == 0); /* Neither ESP nor event producers touch UART. */
    drain();
    assert(strstr(output, "i2s: disabled"));
#if CONFIG_ESP_CONSOLE_UART
    // Ordinary ESP logs, including warnings/errors, use the same whole-line
    // driver submission as logits instead of returning through vprintf/VFS.
    assert(bulk_uart_calls == uart_calls);
#endif
    assert(delays == 100);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, false);
    assert(audio_diagnostics_info_enabled());

    /* A stream may start after an informational message was queued. */
    reset();
    emit("I (0) idle: old status\n");
    audio_diagnostics_set_stream(AUDIO_DIAG_PLAYBACK, true);
    drain();
    assert(uart_calls == (CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING ? 0 : 1));

    /* KWS results survive quiet mode and stream starts, using the same worker. */
    reset();
    audio_diagnostics_result("kws", "keyword class=", 2);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    audio_diagnostics_result("kws", "keyword class=", 7);
    assert(uart_calls == 0 && s_log_queue->count == 2);
    drain();
    assert(strstr(output, "kws: keyword class= 2\n"));
    assert(strstr(output, "kws: keyword class= 7\n"));
    assert(delays == 0); /* Application results are INFO, not warnings. */

    reset();
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    for (unsigned i = 0; i < LOG_QUEUE_LENGTH + 3; ++i) {
        const bool accepted = audio_diagnostics_result("kws", "keyword class=", 2);
        assert(accepted == (i < LOG_QUEUE_LENGTH));
    }
    assert(uart_calls == 0 && s_dropped_logs == 3);
    drain();
    assert(uart_calls == LOG_QUEUE_LENGTH);
    assert(audio_diagnostics_result("kws", "keyword class=", 2)); // Retry after space returns.
    drain();
    assert(uart_calls == LOG_QUEUE_LENGTH + 1);

    /* A full queue loses logs, never blocks or falls back to UART. */
    reset();
    for (unsigned i = 0; i < LOG_QUEUE_LENGTH + 7; ++i)
        emit("W (0) driver: error %u\n", i);
    assert(s_log_queue->count == LOG_QUEUE_LENGTH && s_dropped_logs == 7);
    assert(uart_calls == 0);
    drain();
    assert(strstr(output, "log queue full, dropped=7"));
    assert(delays == LOG_QUEUE_LENGTH * 100);
    audio_diagnostics_stats_t stats;
    audio_diagnostics_get_stats(&stats);
    assert(stats.dropped_logs == 7); /* Reporting must not clear lifetime counts. */

    /* ISR events use FromISR and leave formatting/output to the worker. */
    reset();
    in_isr = 1;
    audio_diagnostics_event(ESP_LOG_WARN, "usb_audio", "overflow", 0, false);
    assert(isr_sends == 1 && mock_yields == 1 && uart_calls == 0);
    in_isr = 0;
    drain();
    assert(strstr(output, "usb_audio: overflow\n"));

    /* Long SDK messages are bounded, terminated and end with a newline. */
    reset();
    char text[600];
    memset(text, 'x', sizeof(text) - 1); text[sizeof(text) - 1] = 0;
    emit("E (0) driver: %s\n", text);
    drain();
    assert(output_size == LOG_LINE_BYTES - 1 && output[output_size - 1] == '\n');

    /* A console mute closes every severity, including previously queued errors. */
    reset();
    emit("E (0) driver: old error\n");
    audio_diagnostics_event(ESP_LOG_WARN, "usb", "old warning", 0, false);
    audio_diagnostics_result("other", "result=", 1);
    audio_diagnostics_result("kws", "perf=", 1);
    audio_diagnostics_result("kws", "keyword class=", 2);
    audio_diagnostics_set_logs_enabled(false);
    assert(!audio_diagnostics_logs_enabled() && !audio_diagnostics_info_enabled());
    unsigned queued = s_log_queue->count;
    emit("E (0) driver: muted error\n");
    emit("W (0) driver: muted warning\n");
    emit("I (0) kws: muted performance\n");
    audio_diagnostics_event(ESP_LOG_ERROR, "usb", "muted error", 0, false);
    audio_diagnostics_result("other", "result=", 1);
    audio_diagnostics_result("kws", "perf=", 1);
    assert(s_log_queue->count == queued);
    audio_diagnostics_result("kws", "keyword class=", 7);
    s_dropped_logs = 5; // A pending queue-full summary must also stay silent.
    drain();
    assert(strcmp(output, "I (0) kws: keyword class= 2\nI (0) kws: keyword class= 7\n") == 0);
    assert(s_dropped_logs == 5 && delays == 0);
    unsigned saved = output_size;
    assert(__wrap__write_r(&mock_reent, STDOUT_FILENO, "SDK printf\n", 11) == 11);
    assert(__wrap__write_r(&mock_reent, STDERR_FILENO, "SDK stderr\n", 11) == 11);
    assert(output_size == saved);
    assert(__wrap__write_r(&mock_reent, 3, "file data", 9) == 9 && file_writes == 1);

    /* Explicit opening overrides USB quiet but retains the stream mask. */
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    audio_diagnostics_set_logs_enabled(true);
    assert(audio_diagnostics_logs_enabled() && audio_diagnostics_info_enabled());
    emit("I (0) cpu: restored status\n");
    audio_diagnostics_event(ESP_LOG_WARN, "usb", "restored warning", 0, false);
    drain();
    assert(strstr(output, "cpu: restored status") && strstr(output, "usb: restored warning"));
    assert(__wrap__write_r(&mock_reent, STDOUT_FILENO, "SDK restored\n", 13) == 13);
    assert(strstr(output, "SDK restored"));
    audio_diagnostics_set_logs_enabled(false);
    assert(!audio_diagnostics_info_enabled());

    /* Console acknowledgments survive mute AND a full log queue; no KWS
     * result is evicted to make room for the response. */
    reset();
    for (unsigned i = 0; i < LOG_QUEUE_LENGTH - 1; ++i)
        emit("E (0) driver: pending error\n");
    audio_diagnostics_result("kws", "keyword class=", 2);
    assert(s_log_queue->count == LOG_QUEUE_LENGTH);
    audio_diagnostics_set_logs_enabled(false);
    audio_diagnostics_print_log_state();
    assert(strcmp(output, "I (0) audio_diag: non-KWS logs OFF; KWS results only\n") == 0);
    assert(s_log_queue->count == LOG_QUEUE_LENGTH);
    drain();
    assert(strstr(output, "kws: keyword class= 2\n"));
    assert(!strstr(output, "driver:") && !strstr(output, "log queue full"));
    audio_diagnostics_set_logs_enabled(true);
    audio_diagnostics_print_log_state();
    assert(strstr(output, "audio_diag: non-KWS logs ON\n"));

    /* Raw logits are off by default; enabling is an explicit exception to
     * global/USB mute. Preserve the model's values before softmax/debounce. */
    reset();
    float values[8] = {-1.25f, 0, 3.5f, -4, 5.25f, -6.5f, 7, 0.126f};
    audio_diagnostics_kws_logits(41, values);
    assert(!s_logits_queue->count);
    audio_diagnostics_set_logs_enabled(false);
    audio_diagnostics_set_stream(AUDIO_DIAG_CAPTURE2, true);
    audio_diagnostics_set_kws_logits_enabled(true);
    audio_diagnostics_print_kws_logits_state();
    assert(strstr(output, "KL ON") && strstr(output, "KL,seq,sil,unk,yes,no,up,down,left,right"));
    audio_diagnostics_kws_logits(42, values);
    memset(values, 0, sizeof(values)); // No pointer to model scratch may escape.
    assert(uart_calls == 1 && s_logits_queue->count == 1 && s_log_queue->count == 0);
    drain();
    assert(strstr(output, "KL,42,-1.25,0.00,3.50,-4.00,5.25,-6.50,7.00,0.13\n"));
    assert(s_logits_sent == 1 && s_logits_dropped == 0);
    audio_diagnostics_set_kws_logits_enabled(false);
    audio_diagnostics_print_kws_logits_state();
    assert(strstr(output, "KL OFF tx=1 drop=0"));
    saved = output_size;
    audio_diagnostics_kws_logits(43, values);
    drain();
    assert(output_size == saved);

    /* A saturated raw queue must never occupy the keyword-result queue. */
    reset();
    for (unsigned i = 0; i < LOG_QUEUE_LENGTH - 1; ++i)
        emit("E (0) driver: stale error\n");
    audio_diagnostics_result("kws", "keyword class=", 2);
    audio_diagnostics_set_logs_enabled(false);
    audio_diagnostics_set_kws_logits_enabled(true);
    for (unsigned i = 0; i < LOGITS_QUEUE_LENGTH + 3; ++i)
        audio_diagnostics_kws_logits(i, values);
    assert(s_logits_queue->count == LOGITS_QUEUE_LENGTH && s_logits_dropped == 3);
    assert(s_log_queue->count == LOG_QUEUE_LENGTH && uart_calls == 0);
    ticks = 1000;
    drain();
    assert(strncmp(output, "I (0) kws: keyword class= 2\n", sizeof("I (0) kws: keyword class= 2\n") - 1) == 0);
    assert(strstr(output, "KD,3\n") && s_logits_sent == LOGITS_QUEUE_LENGTH);
    assert(delays == 1); // Continuous tracing still services idle/watchdog.

    /* Disabling/re-enabling must not print buffered data from a prior session. */
    reset();
    audio_diagnostics_set_kws_logits_enabled(true);
    audio_diagnostics_kws_logits(1, values);
    logits_message_t stale;
    assert(xQueueReceive(s_logits_queue, &stale, 0) == pdTRUE);
    audio_diagnostics_set_kws_logits_enabled(false);
    audio_diagnostics_set_kws_logits_enabled(true);
    dispatch_logits(&stale);
    assert(uart_calls == 0);
    audio_diagnostics_kws_logits(2, values);
    audio_diagnostics_set_kws_logits_enabled(false);
    assert(s_logits_queue->count == 0);
    const unsigned reads_before = raw_queue_reads, notifications_before = worker_notifications;
    for (unsigned frame = 0; frame < 100; ++frame) audio_diagnostics_kws_logits(frame, values);
    drain();
    assert(uart_calls == 0 && raw_queue_reads == reads_before);
    assert(worker_notifications == notifications_before && s_logits_dropped == 0);

    /* Fixed-point float32 extrema exceed the bounded line: drop the whole
     * debug frame, never transmit a truncated/misaligned CSV record. */
    reset();
    audio_diagnostics_set_kws_logits_enabled(true);
    float large_values[34];
    for (unsigned i = 0; i < 34; ++i) large_values[i] = -3.402823466e+38f;
    audio_diagnostics_kws_logits_n(UINT32_MAX, large_values, 34);
    drain();
    assert(s_logits_sent == 0 && s_logits_dropped == 1 && output_size == 0);
    /* Compare the fast binary32 formatter with the existing printf contract,
     * including exact decimal ties, signed zero and very small magnitudes. */
    logits_message_t sample = {.sequence = UINT32_MAX, .count = AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT};
    uint32_t random_bits = 1729;
    const float boundaries[8] = {0.015f, 8.125f, 8.375f, -0.015f, -0.0f, 0.0f, 1.005f, -1.005f};
    for (unsigned row = 0; row < 10001; ++row) {
        for (unsigned col = 0; col < 8; ++col) {
            random_bits = random_bits * 1664525U + 1013904223U;
            uint32_t bits = (random_bits & 0x807fffffU) | ((random_bits % 149U) << 23);
            memcpy(&sample.values[col], &bits, sizeof(bits));
            if (row == 0) sample.values[col] = boundaries[col];
        }
        char fast[128], expected[128];
        assert(format_logits_line(fast, sizeof(fast), &sample));
        snprintf(expected, sizeof(expected), "KL,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            (unsigned long)sample.sequence, (double)sample.values[0], (double)sample.values[1],
            (double)sample.values[2], (double)sample.values[3], (double)sample.values[4],
            (double)sample.values[5], (double)sample.values[6], (double)sample.values[7]);
        assert(strcmp(fast, expected) == 0);
    }
#if CONFIG_ESP_CONSOLE_UART
    assert(bulk_uart_calls > 0);
    reset();
    audio_diagnostics_set_logs_enabled(false);
    unsigned before = bulk_uart_calls;
    audio_diagnostics_write_kws_line("first\nsecond\n", 13);
    assert(bulk_uart_calls == before + 1);
#if CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF
    assert(strcmp(last_bulk_line, "first\r\nsecond\r\n") == 0);
#else
    assert(strcmp(last_bulk_line, "first\nsecond\n") == 0);
#endif
    reset();
    before = bulk_uart_calls;
    driver_ready = false;
    audio_diagnostics_write_kws_line("fallback\n", 9);
    assert(strcmp(output, "fallback\n") == 0 && bulk_uart_calls == before);
#endif
    vQueueDelete(s_logits_queue);
    vQueueDelete(s_log_queue);
    puts("audio diagnostics behavior passed");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", nargs="+", required=True)
    args = parser.parse_args()
    src = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="audio-diagnostics-") as directory:
        temp = Path(directory)
        for name, content in HEADERS.items():
            target = temp / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(content, encoding="utf-8")
        harness = temp / "test.c"
        harness.write_text(HARNESS, encoding="utf-8")
        for quiet, suspend in ((1, 0), (0, 0), (1, 1), (0, 1)):
            for uart in (0, 1):
                exe = temp / f"test-quiet-{quiet}-suspend-{suspend}-uart-{uart}.exe"
                subprocess.run(args.cc + [
                    f"-I{temp}", f"-I{src}",
                    f"-DCONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING={quiet}",
                    f"-DCONFIG_AUDIO_DIAGNOSTICS_CAPTURE2_SUSPEND_UART={suspend}",
                    f"-DCONFIG_ESP_CONSOLE_UART={uart}", "-DCONFIG_ESP_CONSOLE_UART_NUM=0",
                    f"-DCONFIG_LIBC_STDOUT_LINE_ENDING_CRLF={quiet}",
                    str(harness), "-o", str(exe),
                ], check=True, timeout=30)
                subprocess.run([str(exe)], check=True, timeout=30)
                print(f"quiet_while_streaming={quiet}, suspend_capture2={suspend}, bulk_uart={uart}: passed")


if __name__ == "__main__":
    main()
