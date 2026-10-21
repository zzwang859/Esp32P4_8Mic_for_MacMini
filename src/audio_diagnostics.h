#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log_level.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AUDIO_DIAG_PLAYBACK = 0,
    AUDIO_DIAG_CAPTURE1,
    AUDIO_DIAG_CAPTURE2,
} audio_diag_stream_t;

/* Initialize before starting USB, DSP or peripheral tasks. */
esp_err_t audio_diagnostics_init(void);
void audio_diagnostics_set_stream(audio_diag_stream_t stream, bool active);
bool audio_diagnostics_info_enabled(void);
/* Optional Capture2 A/B isolation: overrides every console output path. */
bool audio_diagnostics_uart_suspended(void);
/* Task context. false: only KWS recognition results; true: normal logs enabled
 * even during USB streaming. Until first override, keep menuconfig quiet policy.
 * Runtime setting only, reset on reboot. DOA's e/d switch remains independent. */
void audio_diagnostics_set_logs_enabled(bool enabled);
bool audio_diagnostics_logs_enabled(void);
/* Console task only: print one explicit switch acknowledgment, even when muted
 * (except during the optional Capture2 UART isolation experiment).
 * This does not consume the log queue or discard pending KWS results. */
void audio_diagnostics_print_log_state(void);
/* Explicit debug trace, independent of l/L and USB quiet. Default off.
 * Optional Capture2 UART isolation suppresses it without stopping inference.
 * Set/print from console task; submit from KWS inference (zero-wait copy).
 * Raw model output, before warmup and debounce. Disabled: no new debug copies,
 * queue polling, formatting or trace notifications. Normal KWS still runs.
 * A dispatch already in progress may finish; queued trace copies are cleared. */
#define AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT 8U
bool audio_diagnostics_kws_logits_enabled(void);
void audio_diagnostics_set_kws_logits_enabled(bool enabled);
void audio_diagnostics_print_kws_logits_state(void);
void audio_diagnostics_kws_logits(uint32_t sequence,
    const float logits[AUDIO_DIAGNOSTICS_KWS_LOGIT_COUNT]);
void audio_diagnostics_kws_logits_n(uint32_t sequence, const float *logits, unsigned count);
/* Unbuffered path for KWS results, enabled trace and console/control acknowledgments. */
void audio_diagnostics_write_kws_line(const char *line, size_t length);
typedef struct {
    uint32_t dropped_logs;
} audio_diagnostics_stats_t;
/* Lifetime counters, including diagnostics discarded/coalesced during audio. */
void audio_diagnostics_get_stats(audio_diagnostics_stats_t *stats);

/* Nonblocking, ISR-safe. tag/text must remain valid (use string literals). */
void audio_diagnostics_event(esp_log_level_t level, const char *tag,
                             const char *text, int value, bool has_value);

/* Application results stay visible during USB audio. Same bounded queue and
 * priority-1 UART worker; no formatting or UART I/O in the producer.
 * tag/text must remain valid. Returns true only when enqueued; false on
 * saturation, uninitialized diagnostics or a masked non-KWS result. */
bool audio_diagnostics_result(const char *tag, const char *text, int value);

#ifdef __cplusplus
}
#endif
