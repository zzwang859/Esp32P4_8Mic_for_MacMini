#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "led_ring_effects.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_RING_TOP_MODE_PREVIOUS = -1,
    LED_RING_TOP_MODE_DEFAULT = 0,
    LED_RING_TOP_MODE_CONVERSATION = 1,
    LED_RING_TOP_MODE_KEYWORD = 2,
    LED_RING_TOP_MODE_FAULT = 4,
} led_ring_top_mode_t;

typedef struct {
    uint32_t duration_ms; /* Zero selects CONFIG_WS2812_LED_KEYWORD_DURATION_MS. */
    bool has_next_mode; /* False returns to the interrupted mode. */
    led_ring_top_mode_t next_mode;
    bool guarded; /* Revalidate a queued KWS action at actual dispatch. */
    int class_index;
    uint32_t model_epoch, config_revision;
} led_ring_keyword_options_t;

/** Initialize RMT, run the blocking 6-second RGB self-test, then start LED tasks. */
esp_err_t led_ring_start(void);

/** Select persistent top-level mode 0, 1 or 4. */
esp_err_t led_ring_set_mode(led_ring_top_mode_t mode);

/** Trigger mode 2. Color is required; NULL options use default duration/return. */
esp_err_t led_ring_trigger_keyword(led_ring_rgb_t color,
                                   const led_ring_keyword_options_t *options);

/** Set/get the shared processed-capture/playback threshold in dBFS (-96 <= value < 0). */
esp_err_t led_ring_set_audio_threshold_db(float threshold_db);
float led_ring_get_audio_threshold_db(void);

/** Return the mode most recently accepted by led_ring_set_mode(). */
led_ring_top_mode_t led_ring_get_mode(void);

#ifdef __cplusplus
}
#endif
