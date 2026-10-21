#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
void audio_pipeline_set_capture_available(bool available);
bool audio_pipeline_capture_available(void);
esp_err_t audio_pipeline_init(void);
/* Control task only, after every registered processing worker has paused. */
void audio_pipeline_maintenance_reset(void);

typedef struct {
    float processed_dbfs;
    float playback_dbfs;
} audio_pipeline_levels_t;

/* Single writer: TDM capture task; input is 24-kHz interleaved 8-channel PCM. */
void audio_pipeline_capture(const int16_t *samples, size_t frames);
/* Single writer: 16-kHz I2S playback task, after the actual write of host PCM. */
void audio_pipeline_reference(const int16_t *stereo, size_t frames);
typedef struct {
    int16_t audio; /* Left: selected audio, unchanged PCM16. */
    int16_t right; /* Selected right source: copy of left, VAD marker or encoded DOA. */
} audio_pipeline_capture2_frame_t;
/* Single reader: USB task. Nonblocking, returns complete 16-kHz L/R frames.
 * count and return value are stereo frames, not individual channel samples. */
size_t audio_pipeline_read(audio_pipeline_capture2_frame_t *frames, size_t count);
/* Number of complete Capture2 L/R frames available. */
size_t audio_pipeline_available(void);
typedef struct {
    uint32_t queued_frames;
    uint32_t queued_zero_pairs;
} audio_pipeline_capture2_stats_t;
/* Lifetime counters of successfully queued stereo frames; zero pairs are
 * observations, not errors (silence/unknown VAD and DOA=180 degrees are valid). */
void audio_pipeline_get_capture2_stats(audio_pipeline_capture2_stats_t *stats);
/* Current DAS steering bearing, encoded as round(degrees * 100 - 18000).
 * Moves by the configured shortest-path speed only while AFE VAD is speech.
 * Retains the current angle while frozen; initial 0 degrees encodes as -18000.
 * ISR-safe: only loads an integer snapshot encoded by the DSP task. */
int16_t audio_pipeline_get_doa_sample(void);
typedef enum {
    AUDIO_PIPELINE_VAD_UNKNOWN = -1, /* Before the first valid fetch or after a fetch failure. */
    AUDIO_PIPELINE_VAD_SILENCE = 0,  /* Noise or silence. */
    AUDIO_PIPELINE_VAD_SPEECH = 1,
} audio_pipeline_vad_state_t;
/* Latest AFE VAD state. Nonblocking and ISR-safe; UNKNOWN on a fetch failure.
 * This labels continuous audio; it does not gate USB audio or KWS input. */
audio_pipeline_vad_state_t audio_pipeline_get_vad_state(void);
/* Called only by the USB consumer when the Capture2 alternate setting changes. */
void audio_pipeline_set_recording(bool active);
/* Latest processed-capture and stereo-downmix RMS levels. Stale sources read as silence. */
void audio_pipeline_get_levels(audio_pipeline_levels_t *levels);
/* e=DOA log on, d=off; l/L toggles non-KWS logs; p/P toggles raw KWS logits. */
bool audio_pipeline_console_command(uint8_t command);
#ifdef __cplusplus
}
#endif
