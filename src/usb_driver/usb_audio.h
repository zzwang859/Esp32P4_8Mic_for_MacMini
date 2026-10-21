#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t usb_audio_init(void);
/* bit0 playback, bit1 raw capture, bit2 processed capture. Read-only snapshot. */
unsigned usb_audio_stream_mask(void);
/* Preserve enumeration/isochronous clocks, send zero PCM on both captures. */
void usb_audio_set_maintenance(bool active);

typedef struct {
    uint32_t raw_fifo_overflows;
    uint32_t i2s_read_anomalies;
    uint32_t raw_endpoint_null_packets;
    uint32_t mono_endpoint_null_packets;
    uint32_t endpoint_fifo_backpressure;
    uint32_t endpoint_write_shorts;
    uint32_t mono_tx_packets;
    uint32_t mono_tx_zero_packets;
    uint32_t mono_tx_zero_scheduled;
    uint32_t mono_tx_zero_failed;
    uint32_t mono_tx_zero_unexpected;
    uint32_t mono_tx_unexpected_sizes;
    uint32_t mono_source_short_reads;
    uint32_t mono_source_read_packets;
    uint32_t mono_source_zero_packets;
    uint32_t mono_source_min_samples;
    uint32_t mono_source_frames; /* Lifetime, excludes frames discarded on reopen. */
    uint32_t mono_source_zero_pairs; /* Lifetime L=0 AND R=0, not all-zero packets. */
    uint32_t mono_source_packets_15;
    uint32_t mono_source_packets_16;
    uint32_t mono_source_packets_17;
    uint32_t playback_underflows;
    uint32_t playback_dropped_bytes;
    uint32_t playback_i2s_errors;
    uint32_t playback_peak_bytes;
} usb_audio_capture_stats_t;

void usb_audio_get_capture_stats(usb_audio_capture_stats_t *stats);

#ifdef __cplusplus
}
#endif

