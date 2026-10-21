#pragma once
#include "custom_protocol.h"
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool actions_enabled, led_enabled, keyboard_enabled;
    uint8_t r, g, b, modifier, keycodes[6];
    uint16_t led_duration_ms, key_hold_ms;
} custom_binding_t;

typedef struct {
    uint16_t keyword_threshold_percent;
    uint32_t cooldown_ms;
    uint16_t stable_frames;
    uint8_t need_vad_checked;
} custom_kws_filter_t;

void custom_kws_filter_get(custom_kws_filter_t *filter, uint32_t *revision);

/* Initialize storage/cache before USB or inference; start workers after audio. */
esp_err_t custom_control_init(void);
esp_err_t custom_control_start(void);
void custom_control_boot_fallback(void);
/* UART: a/A enables actions, x/X disables them; u/U toggles HID UART logs
 * (default off at boot, RAM only, independent of ordinary log mute).
 * Commands join the HID control FIFO; true means the character was handled. */
bool custom_control_console_command(uint8_t command);
void custom_hid_receive(const uint8_t *data, uint16_t size);
void custom_hid_disconnected(void);
void custom_keyword_event(int class_index);

uint16_t custom_class_count(void);
uint32_t custom_model_epoch(void);
uint32_t custom_binding_revision(void);
bool custom_binding_get(int class_index, custom_binding_t *binding, uint32_t *revision);
/* binding_get's revision is a per-class action fence. The separate
 * custom_binding_revision() is the global configuration revision for PC writes. */
bool custom_action_allowed(int class_index, uint32_t epoch, uint32_t revision, bool keyboard);
void custom_set_runtime_state(unsigned state, esp_err_t error);
/* Only the paused KWS loader uses these; labels remain valid until next reload. */
const char *custom_model_partition(void);
const char *custom_class_text(int class_index);

/* Registered processing tasks cooperate at safe boundaries. No forced suspend. */
enum {
    CUSTOM_WORK_DSP = 1,
    CUSTOM_WORK_FEED = 2,
    CUSTOM_WORK_FETCH = 4,
    CUSTOM_WORK_NS = 8,
    CUSTOM_WORK_DOA = 16,
    CUSTOM_WORK_KWS = 32,
    CUSTOM_WORK_CAPTURE = 64
};
void custom_worker_register(uint32_t bit);
bool custom_worker_checkpoint(uint32_t bit);
bool custom_maintenance_active(void);

#ifdef __cplusplus
}
#endif
