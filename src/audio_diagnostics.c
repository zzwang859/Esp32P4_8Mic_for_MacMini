#include "audio_diagnostics.h"
#ifdef ESP_PLATFORM
#include "custom_control.h"
#endif

#include <stdarg.h>
#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define LOG_QUEUE_LENGTH 24U
#define LOG_LINE_BYTES 256U
#define LOGITS_QUEUE_LENGTH 32U
typedef struct {
    uint32_t sequence;
    uint32_t generation;
    unsigned count;
    float values[34];
} logits_message_t;

typedef struct {
    bool formatted;
    bool application_result;
    esp_log_level_t level;
    uint32_t timestamp;
    union {
        char line[LOG_LINE_BYTES];
        struct {
            const char *tag;
            const char *text;
            int value;
            bool has_value;
        } event;
    } data;
} diagnostic_message_t;

static QueueHandle_t s_log_queue;
static QueueHandle_t s_logits_queue;
static TaskHandle_t s_uart_task;
static uint32_t s_logits_enabled, s_logits_generation;
static uint32_t s_logits_dropped, s_logits_sent;
static uint32_t s_stream_mask;
enum { LOG_MODE_AUTO, LOG_MODE_KWS_ONLY, LOG_MODE_ALL };
static uint32_t s_log_mode = LOG_MODE_AUTO;
/* Retained in RAM; queue losses are reported only after all streams close. */
static uint32_t s_dropped_logs;
static uint32_t s_total_dropped_logs;

void audio_diagnostics_set_stream(audio_diag_stream_t stream, bool active)
{
    const uint32_t bit = 1U << (unsigned)stream;
    if (active) __atomic_fetch_or(&s_stream_mask, bit, __ATOMIC_RELAXED);
    else __atomic_fetch_and(&s_stream_mask, ~bit, __ATOMIC_RELAXED);
}

bool audio_diagnostics_uart_suspended(void)
{
#if CONFIG_AUDIO_DIAGNOSTICS_CAPTURE2_SUSPEND_UART
    return (__atomic_load_n(&s_stream_mask, __ATOMIC_RELAXED) & (1U << AUDIO_DIAG_CAPTURE2)) != 0U;
#else
    return false;
#endif
}

bool audio_diagnostics_info_enabled(void)
{
    if (audio_diagnostics_uart_suspended()) return false;
    const uint32_t mode = __atomic_load_n(&s_log_mode, __ATOMIC_RELAXED);
    if (mode == LOG_MODE_KWS_ONLY) return false;
    if (mode == LOG_MODE_ALL) return true;
#if CONFIG_AUDIO_DIAGNOSTICS_QUIET_WHILE_STREAMING
    return __atomic_load_n(&s_stream_mask, __ATOMIC_RELAXED) == 0U;
#else
    return true;
#endif
}

bool audio_diagnostics_logs_enabled(void)
{
    return __atomic_load_n(&s_log_mode, __ATOMIC_RELAXED) != LOG_MODE_KWS_ONLY;
}

void audio_diagnostics_set_logs_enabled(bool enabled)
{
    if (!enabled) __atomic_store_n(&s_log_mode, LOG_MODE_KWS_ONLY, __ATOMIC_RELAXED);
    // Discard buffered fragments under the closed gate, rather than releasing
    // muted printf output when logging is enabled again.
    if (!audio_diagnostics_logs_enabled()) {
        fflush(stdout);
        fflush(stderr);
    }
    if (enabled) __atomic_store_n(&s_log_mode, LOG_MODE_ALL, __ATOMIC_RELAXED);
}

static bool is_kws_result(const diagnostic_message_t *message)
{
    return message->application_result && message->data.event.tag && message->data.event.text &&
           strcmp(message->data.event.tag, "kws") == 0 &&
           strcmp(message->data.event.text, "keyword class=") == 0;
}

void audio_diagnostics_print_log_state(void)
{
    char line[LOG_LINE_BYTES];
    const bool enabled = audio_diagnostics_logs_enabled();
    const int length = snprintf(line, sizeof(line), "I (%lu) audio_diag: non-KWS logs %s%s\n",
        (unsigned long)esp_log_timestamp(), enabled ? "ON" : "OFF",
        enabled ? "" : audio_diagnostics_kws_logits_enabled()
            ? "; KWS results + logits" : "; KWS results only");
    // A command response must remain visible even if the log queue was full
    // before muting. Only the console task calls this, never an audio task.
    if (length > 0) audio_diagnostics_write_kws_line(line,
        (size_t)length < sizeof(line) ? (size_t)length : sizeof(line) - 1);
}

bool audio_diagnostics_kws_logits_enabled(void)
{
    return __atomic_load_n(&s_logits_enabled, __ATOMIC_RELAXED) != 0;
}

void audio_diagnostics_set_kws_logits_enabled(bool enabled)
{
    if (!s_logits_queue) return;
    __atomic_store_n(&s_logits_enabled, 0U, __ATOMIC_RELAXED);
    __atomic_fetch_add(&s_logits_generation, 1U, __ATOMIC_RELAXED);
    xQueueReset(s_logits_queue); // Only debug copies; never reset the KWS input queue.
    if (enabled) {
        __atomic_store_n(&s_logits_dropped, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&s_logits_sent, 0U, __ATOMIC_RELAXED);
        __atomic_store_n(&s_logits_enabled, 1U, __ATOMIC_RELAXED);
    }
}

void audio_diagnostics_print_kws_logits_state(void)
{
    char line[160];
    int length;
    if (audio_diagnostics_kws_logits_enabled()) {
#ifdef ESP_PLATFORM
        if (custom_model_partition()) {
            length = snprintf(line, sizeof(line),
                "KL ON (10ms/frame)\nKL,seq,class_index[0..%u]; labels: control HID classes\n",
                (unsigned)custom_class_count()-1);
        } else
#endif
        {
        length = snprintf(line, sizeof(line),
            "KL ON (10ms/frame)\nKL,seq,sil,unk,yes,no,up,down,left,right\n");
        }
    } else {
        length = snprintf(line, sizeof(line), "KL OFF tx=%lu drop=%lu\n",
            (unsigned long)__atomic_load_n(&s_logits_sent, __ATOMIC_RELAXED),
            (unsigned long)__atomic_load_n(&s_logits_dropped, __ATOMIC_RELAXED));
    }
    if (length > 0) audio_diagnostics_write_kws_line(line, (size_t)length);
}

void audio_diagnostics_kws_logits(uint32_t sequence,
    const float logits[AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT])
{
    audio_diagnostics_kws_logits_n(sequence,logits,AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT);
}
void audio_diagnostics_kws_logits_n(uint32_t sequence,const float *logits,unsigned count)
{
    if (!audio_diagnostics_kws_logits_enabled() || !logits || !count || count>34) return;
    if (audio_diagnostics_uart_suspended()) {
        __atomic_fetch_add(&s_logits_dropped, 1U, __ATOMIC_RELAXED);
        return;
    }
    logits_message_t message = {
        .sequence = sequence,
        .generation = __atomic_load_n(&s_logits_generation, __ATOMIC_RELAXED),
    };
    message.count=count;
    memcpy(message.values, logits, count * sizeof(float));
    if (xQueueSend(s_logits_queue, &message, 0) != pdTRUE)
        __atomic_fetch_add(&s_logits_dropped, 1U, __ATOMIC_RELAXED);
    xTaskNotifyGive(s_uart_task);
}

void audio_diagnostics_get_stats(audio_diagnostics_stats_t *stats)
{
    if (!stats) return;
    stats->dropped_logs = __atomic_load_n(&s_total_dropped_logs, __ATOMIC_RELAXED);
}

static bool enqueue_message(const diagnostic_message_t *message)
{
    if (!s_log_queue) return false;
    BaseType_t sent;
    if (xPortInIsrContext()) {
        BaseType_t woken = pdFALSE;
        sent = xQueueSendFromISR(s_log_queue, message, &woken);
        vTaskNotifyGiveFromISR(s_uart_task, &woken);
        if (woken) portYIELD_FROM_ISR();
    } else {
        sent = xQueueSend(s_log_queue, message, 0);
        xTaskNotifyGive(s_uart_task);
    }
    if (sent != pdTRUE) {
        __atomic_fetch_add(&s_dropped_logs, 1U, __ATOMIC_RELAXED);
        __atomic_fetch_add(&s_total_dropped_logs, 1U, __ATOMIC_RELAXED);
    }
    return sent == pdTRUE;
}

/* ESP-IDF Log V1 supplies a complete line with a literal severity prefix. */
static esp_log_level_t format_level(const char *format)
{
    if (format[0] == '\033' && format[1] == '[') {
        while (*format && *format != 'm') ++format;
        if (*format) ++format;
    }
    switch (*format) {
    case 'E': return ESP_LOG_ERROR;
    case 'W': return ESP_LOG_WARN;
    case 'I': return ESP_LOG_INFO;
    case 'D': return ESP_LOG_DEBUG;
    case 'V': return ESP_LOG_VERBOSE;
    default: return ESP_LOG_WARN; // Preserve unrecognized diagnostics.
    }
}

static int enqueue_vprintf(const char *format, va_list args)
{
    if (audio_diagnostics_uart_suspended()) return 0;
    if (!audio_diagnostics_logs_enabled()) return 0;
    const esp_log_level_t level = format_level(format);
    if (level >= ESP_LOG_INFO && !audio_diagnostics_info_enabled()) return 0;
    diagnostic_message_t message = {.formatted = true, .level = level};
    const int needed = vsnprintf(message.data.line, sizeof(message.data.line), format, args);
    if (needed < 0) return needed;
    if ((size_t)needed >= sizeof(message.data.line)) {
        message.data.line[sizeof(message.data.line) - 2] = '\n';
        message.data.line[sizeof(message.data.line) - 1] = '\0';
    }
    enqueue_message(&message);
    return needed;
}

void audio_diagnostics_event(esp_log_level_t level, const char *tag,
                             const char *text, int value, bool has_value)
{
    if (audio_diagnostics_uart_suspended()) return;
    if (!audio_diagnostics_logs_enabled()) return;
    if (level >= ESP_LOG_INFO && !audio_diagnostics_info_enabled()) return;
    diagnostic_message_t message = {
        .level = level,
        .timestamp = esp_log_timestamp(),
        .data.event = {tag, text, value, has_value},
    };
    enqueue_message(&message);
}

static void uart_print(const char *format, ...)
{
    if (audio_diagnostics_uart_suspended()) return;
    va_list args;
    va_start(args, format);
    char line[LOG_LINE_BYTES];
    const int needed = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (needed <= 0) return;
    if ((size_t)needed >= sizeof(line)) line[sizeof(line) - 2] = '\n';
    // Ordinary logs and application results use the same whole-line UART
    // path as logits. Never return to the byte-at-a-time stdio UART VFS.
    audio_diagnostics_write_kws_line(line,
        (size_t)needed < sizeof(line) ? (size_t)needed : sizeof(line) - 1);
}

bool audio_diagnostics_result(const char *tag, const char *text, int value)
{
    if (audio_diagnostics_uart_suspended()) return false;
    diagnostic_message_t message = {
        .application_result = true,
        .level = ESP_LOG_INFO,
        .timestamp = esp_log_timestamp(),
        .data.event = {tag, text, value, true},
    };
    if (!audio_diagnostics_logs_enabled() && !is_kws_result(&message)) return false;
    return enqueue_message(&message);
}

static bool dispatch_message(const diagnostic_message_t *message)
{
    if (audio_diagnostics_uart_suspended()) return false;
    const bool kws_result = is_kws_result(message);
    // Recheck at dispatch, including messages queued just before mute/start.
    if (!audio_diagnostics_logs_enabled() && !kws_result) return false;
    if (!message->application_result && message->level >= ESP_LOG_INFO &&
        !audio_diagnostics_info_enabled()) return false;
    if (message->formatted) {
        audio_diagnostics_write_kws_line(message->data.line, strlen(message->data.line));
    } else if (message->data.event.has_value) {
        uart_print("%c (%lu) %s: %s %d\n",
                   message->level == ESP_LOG_ERROR ? 'E' : message->level == ESP_LOG_WARN ? 'W' : 'I',
                   (unsigned long)message->timestamp, message->data.event.tag,
                   message->data.event.text, message->data.event.value);
    } else {
        uart_print("%c (%lu) %s: %s\n",
                   message->level == ESP_LOG_ERROR ? 'E' : message->level == ESP_LOG_WARN ? 'W' : 'I',
                   (unsigned long)message->timestamp, message->data.event.tag, message->data.event.text);
    }
    if (message->level <= ESP_LOG_WARN) vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

static bool append_unsigned(char **cursor, const char *end, uint32_t value)
{
    char digits[10];
    size_t count = 0;
    do { digits[count++] = (char)('0' + value % 10); value /= 10; } while (value);
    if ((size_t)(end - *cursor) < count) return false;
    while (count) *(*cursor)++ = digits[--count];
    return true;
}

static size_t format_logits_line(char *line, size_t capacity, const logits_message_t *message)
{
    _Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
                   "Raw logits require IEEE binary32");
    char *cursor = line;
    const char *end = line + capacity - 1;
    memcpy(cursor, "KL,", 3);
    cursor += 3;
    if (!append_unsigned(&cursor, end, message->sequence)) return 0;
    for (unsigned i = 0; i < message->count; ++i) {
        uint32_t bits;
        memcpy(&bits, &message->values[i], sizeof(bits));
        const unsigned exponent = (bits >> 23) & 255U;
        if (exponent > 148U) return 0; // Unusual magnitudes use snprintf below.
        // Exact binary32 value * 100 = mantissa * 25 * 2^(exponent-148).
        // Round ties to even, matching %.2f without float printf/dtoa work.
        const uint32_t numerator = ((bits & 0x7fffffU) | 0x800000U) * 25U;
        const unsigned shift = 148U - exponent;
        uint32_t cents = shift < 32 ? numerator >> shift : 0;
        if (shift > 0 && shift < 32) {
            const uint32_t remainder = numerator & ((1U << shift) - 1U);
            const uint32_t half = 1U << (shift - 1U);
            if (remainder > half || (remainder == half && (cents & 1U))) ++cents;
        }
        if (end - cursor < 2) return 0;
        *cursor++ = ',';
        if (bits >> 31) *cursor++ = '-';
        if (!append_unsigned(&cursor, end, cents / 100) || end - cursor < 3) return 0;
        *cursor++ = '.';
        *cursor++ = (char)('0' + cents / 10 % 10);
        *cursor++ = (char)('0' + cents % 10);
    }
    if (cursor == end) return 0;
    *cursor++ = '\n';
    *cursor = 0;
    return (size_t)(cursor - line);
}

static void dispatch_logits(const logits_message_t *message)
{
    if (audio_diagnostics_uart_suspended()) return;
    if (!audio_diagnostics_kws_logits_enabled() || message->generation !=
        __atomic_load_n(&s_logits_generation, __ATOMIC_RELAXED)) return;
    // Fixed two-decimal output keeps ordinary logits compact. Keep the line
    // bounded; unusually large values use the existing debug-frame drop path.
    char line[512];
    const float *v = message->values;
    int length = (int)format_logits_line(line, sizeof(line), message);
    if (!length) {
        length=snprintf(line,sizeof(line),"KL,%lu",(unsigned long)message->sequence);
        for (unsigned i=0;i<message->count && length>0 && (size_t)length<sizeof(line);++i) {
            int n=snprintf(line+length,sizeof(line)-(size_t)length,",%.2f",(double)v[i]);
            if (n<0) { length=-1; break; } length+=n;
        }
        if (length>0 && (size_t)length<sizeof(line)-1) { line[length++]='\n'; line[length]=0; }
        else length=-1;
    }
    if (length <= 0 || (size_t)length >= sizeof(line)) {
        __atomic_fetch_add(&s_logits_dropped, 1U, __ATOMIC_RELAXED);
        return;
    }
    // A console toggle can preempt float formatting; recheck before output.
    if (!audio_diagnostics_kws_logits_enabled() || message->generation !=
        __atomic_load_n(&s_logits_generation, __ATOMIC_RELAXED)) return;
    audio_diagnostics_write_kws_line(line, (size_t)length);
    if (message->generation == __atomic_load_n(&s_logits_generation, __ATOMIC_RELAXED))
        __atomic_fetch_add(&s_logits_sent, 1U, __ATOMIC_RELAXED);
}

static void diagnostics_task(void *arg)
{
    (void)arg;
    diagnostic_message_t message;
    logits_message_t logits;
    unsigned trace_iterations = 0;
    uint32_t notice_generation = 0, last_notice_ms = 0, last_dropped = 0;
    while (true) {
        bool worked = false;
        // Recognition results retain their own queue and go first. Raw trace
        // traffic can never occupy their queue or backpressure inference.
        while (xQueueReceive(s_log_queue, &message, 0) == pdTRUE) {
            worked = true;
            // Drain masked backlog immediately so it cannot delay a keyword
            // result behind dozens of UART trace transmissions.
            if (dispatch_message(&message)) break;
        }
        if (audio_diagnostics_kws_logits_enabled()) {
            if (xQueueReceive(s_logits_queue, &logits, 0) == pdTRUE) {
                dispatch_logits(&logits);
                worked = true;
                // Keep idle/watchdog service during continuous tracing.
                if ((++trace_iterations % 32U) == 0U) vTaskDelay(pdMS_TO_TICKS(1));
            }
            const uint32_t generation = __atomic_load_n(&s_logits_generation, __ATOMIC_RELAXED);
            if (generation != notice_generation) {
                notice_generation = generation;
                last_notice_ms = last_dropped = 0;
            }
            const uint32_t dropped = __atomic_load_n(&s_logits_dropped, __ATOMIC_RELAXED);
            const uint32_t now = esp_log_timestamp();
            if (dropped != last_dropped && now - last_notice_ms >= 1000U) {
                char line[32];
                const int length = snprintf(line, sizeof(line), "KD,%lu\n", (unsigned long)dropped);
                audio_diagnostics_write_kws_line(line, (size_t)length);
                last_notice_ms = now;
                last_dropped = dropped;
            }
        }
        if (audio_diagnostics_logs_enabled() &&
            __atomic_load_n(&s_stream_mask, __ATOMIC_RELAXED) == 0U) {
            const uint32_t dropped = __atomic_exchange_n(&s_dropped_logs, 0U, __ATOMIC_RELAXED);
            if (dropped)
                uart_print("W (%lu) audio_diag: log queue full, dropped=%lu\n",
                           (unsigned long)esp_log_timestamp(), (unsigned long)dropped);
        }
        if (!worked) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
}

esp_err_t audio_diagnostics_init(void)
{
    if (s_log_queue) return ESP_ERR_INVALID_STATE;
    s_log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(diagnostic_message_t));
    if (!s_log_queue) return ESP_ERR_NO_MEM;
    s_logits_queue = xQueueCreate(LOGITS_QUEUE_LENGTH, sizeof(logits_message_t));
    if (!s_logits_queue) {
        vQueueDelete(s_log_queue);
        s_log_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    // Formatting all eight floats at 100 Hz must not compete with AFE/KWS
    // on core0. DSP/NS/DOA retain higher priority than this UART worker.
    if (xTaskCreatePinnedToCore(diagnostics_task, "audio_diag", 4096, NULL, 1,
                                &s_uart_task, 1) != pdPASS) {
        vQueueDelete(s_logits_queue);
        s_logits_queue = NULL;
        vQueueDelete(s_log_queue);
        s_log_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    esp_log_set_vprintf(enqueue_vprintf);
    return ESP_OK;
}
