#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start after audio/USB/LED allocation, before using inference. */
esp_err_t kws_stream_start(void);
/* Control task only, while the inference worker and its producer are paused. */
esp_err_t kws_stream_reload(void);
void kws_stream_maintenance_reset(void);
/* Single producer selected at build time: core1 post-AFE NS task, or core0 AFE
 * feed task tapping USB playback reference. Copies PCM16; never waits for KWS.
 * discontinuity() must be called only by that same selected producer. */
void kws_stream_feed(const int16_t *pcm, size_t samples);
/* Associate VAD with the submitted PCM, before inference queue latency.
 * A confirming 160-sample block must have high VAD for every supplied span.
 * The legacy feed() supplies unknown/low VAD; No mode ignores this metadata. */
void kws_stream_feed_with_vad(const int16_t *pcm, size_t samples, bool vad_high);
void kws_stream_discontinuity(void);

typedef struct {
    uint64_t chunks;
    uint64_t process_us;
    uint64_t task_cpu_us;
    uint32_t max_us;
    uint32_t dropped_samples;
    uint32_t resets;
    uint32_t errors;
} kws_stream_stats_t;

/* Lifetime statistics. RTF = process_us / (chunks * 10000 us).
 * process_us includes MFCC, inference and feedback, plus task preemption;
 * excludes queue waits, PCM conversion, debounce and logging. */
void kws_stream_get_stats(kws_stream_stats_t *stats);
/* Call every 10 s from the priority-1 statistics task, including quiet periods.
 * Samples FreeRTOS task CPU time (microseconds), handling 32-bit rollover.
 * Printing follows USB quiet policy. task_cpu_us is updated by this call and
 * covers the whole task, including reset/queue/conversion, not just inference. */
void kws_stream_log_stats(void);

#ifdef __cplusplus
}
#endif
