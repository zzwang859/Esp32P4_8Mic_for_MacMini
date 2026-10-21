#include "custom_internal.h"
#include <string.h>
#include "sdkconfig.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_audio.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static custom_slot_meta_t s_meta;
static custom_binding_t s_bindings[CUSTOM_MAX_CLASSES];
static uint32_t s_action_revision[CUSTOM_MAX_CLASSES];
static uint32_t s_boot, s_epoch, s_revision, s_saved_revision;
static int s_slot = -1;
static unsigned s_runtime;
static esp_err_t s_error;
static uint32_t s_maintenance, s_registered, s_paused;
static custom_kws_filter_t s_filter;
static uint32_t s_filter_revision;

bool custom_filter_valid(const custom_kws_filter_t *filter) {
    return filter && filter->keyword_threshold_percent <= 100 &&
           filter->cooldown_ms >= 1 && filter->cooldown_ms <= 60000 &&
           filter->stable_frames >= 1 && filter->stable_frames <= 1000 &&
           filter->need_vad_checked <= 1;
}
void custom_state_filter_init(const custom_kws_filter_t *saved) {
    custom_kws_filter_t defaults = {
        .keyword_threshold_percent = CONFIG_KWS_FILTER_KEYWORD_THRESHOLD_PERCENT,
        .cooldown_ms = CONFIG_KWS_FILTER_COOLDOWN_MS,
        .stable_frames = CONFIG_KWS_FILTER_STABLE_FRAMES,
#if CONFIG_KWS_FILTER_NEED_VAD_CHECKED_YES
        .need_vad_checked = 1,
#else
        .need_vad_checked = 0,
#endif
    };
    portENTER_CRITICAL(&s_lock);
    s_filter = custom_filter_valid(saved) ? *saved : defaults;
    s_filter_revision = 1;
    portEXIT_CRITICAL(&s_lock);
}
void custom_kws_filter_get(custom_kws_filter_t *filter, uint32_t *revision) {
    portENTER_CRITICAL(&s_lock);
    if (filter) *filter = s_filter;
    if (revision) *revision = s_filter_revision;
    portEXIT_CRITICAL(&s_lock);
}
void custom_state_filter_set(const custom_kws_filter_t *filter) {
    portENTER_CRITICAL(&s_lock);
    s_filter = *filter;
    ++s_filter_revision;
    portEXIT_CRITICAL(&s_lock);
}

void custom_state_select(const custom_slot_meta_t *meta, int slot,
                         const custom_saved_config_t *saved) {
    portENTER_CRITICAL(&s_lock);
    s_meta = *meta;
    s_slot = slot;
    ++s_epoch;
    ++s_revision;
    memset(s_bindings, 0, sizeof(s_bindings));
    for (unsigned i = 0; i < CUSTOM_MAX_CLASSES; ++i)
        s_action_revision[i] = s_revision;
    bool matching = saved && !memcmp(saved->package_sha, meta->package_sha, 32);
    if (matching)
        memcpy(s_bindings, saved->bindings, sizeof(s_bindings));
    /* Preserve factory colors/keycodes, but require explicit action enable. */
    else if (slot < 0) {
        const uint8_t keys[] = {28, 17, 24, 7, 15, 21};
        const uint8_t rgb[][3] = {{0, 255, 0}, {255, 0, 0},   {255, 255, 0},
                                  {0, 0, 255}, {0, 255, 255}, {255, 0, 255}};
        for (unsigned i = 0; i < 6; ++i) {
            custom_binding_t *b = &s_bindings[i + 2];
            b->r = rgb[i][0];
            b->g = rgb[i][1];
            b->b = rgb[i][2];
            b->keycodes[0] = keys[i];
            b->led_duration_ms = 600;
        }
    }
    s_saved_revision = matching ? s_revision : 0;
    portEXIT_CRITICAL(&s_lock);
}
void custom_state_init(const custom_saved_config_t *saved, const custom_slot_meta_t *meta,
                       int slot) {
    s_boot = esp_random();
    if (!s_boot)
        s_boot = 1;
    custom_state_select(meta, slot, saved);
    /* Saved enable flags never arm keyword actions at power-on or reset. */
    custom_state_set_actions_enabled(false);
}
void custom_state_set_actions_enabled(bool enabled) {
    portENTER_CRITICAL(&s_lock);
    bool changed = false;
    for (unsigned i = 2; i < s_meta.class_count; ++i) {
        custom_binding_t *b = &s_bindings[i];
        bool has_key = false;
        for (unsigned k = 0; k < 6; ++k)
            has_key |= b->keycodes[k] != 0;
        bool keyboard = enabled && has_key;
        if (b->actions_enabled == enabled && b->led_enabled == enabled &&
            b->keyboard_enabled == keyboard)
            continue;
        if (!changed) {
            ++s_revision;
            changed = true;
        }
        b->actions_enabled = b->led_enabled = enabled;
        b->keyboard_enabled = keyboard;
        s_action_revision[i] = s_revision;
    }
    portEXIT_CRITICAL(&s_lock);
}
uint32_t custom_boot_id(void) {
    return s_boot;
}
uint16_t custom_class_count(void) {
    return __atomic_load_n(&s_meta.class_count, __ATOMIC_ACQUIRE);
}
uint32_t custom_model_epoch(void) {
    return __atomic_load_n(&s_epoch, __ATOMIC_ACQUIRE);
}
uint32_t custom_binding_revision(void) {
    return __atomic_load_n(&s_revision, __ATOMIC_ACQUIRE);
}
int custom_active_slot(void) {
    return s_slot;
}
const char *custom_model_partition(void) {
    return s_slot < 0 ? NULL : s_slot == 0 ? "kws_a" : "kws_b";
}
const char *custom_class_text(int i) {
    return i >= 0 && i < custom_class_count() ? s_meta.labels[i] : "";
}
bool custom_config_dirty(void) {
    return s_saved_revision != s_revision;
}
void custom_state_meta(custom_slot_meta_t *meta) {
    portENTER_CRITICAL(&s_lock);
    *meta = s_meta;
    portEXIT_CRITICAL(&s_lock);
}
bool custom_binding_get(int i, custom_binding_t *b, uint32_t *revision) {
    bool valid;
    portENTER_CRITICAL(&s_lock);
    valid = i >= 2 && i < (int)s_meta.class_count;
    if (valid)
        *b = s_bindings[i];
    if (revision)
        *revision = valid ? s_action_revision[i] : 0;
    portEXIT_CRITICAL(&s_lock);
    return valid;
}
bool custom_action_allowed(int i, uint32_t epoch, uint32_t revision, bool keyboard) {
    custom_binding_t b;
    uint32_t current;
    return !custom_maintenance_active() && custom_binding_get(i, &b, &current) &&
           epoch == custom_model_epoch() && revision == current && b.actions_enabled &&
           (keyboard ? b.keyboard_enabled : b.led_enabled);
}
uint16_t custom_state_set_binding(uint32_t boot, uint32_t epoch, uint32_t revision, unsigned index,
                                  const custom_binding_t *b) {
    if (boot != s_boot || epoch != s_epoch)
        return CUSTOM_MODEL_CHANGED;
    if (index < 2 || index >= s_meta.class_count)
        return CUSTOM_OUT_OF_RANGE;
    if (revision != s_revision)
        return CUSTOM_CONFLICT;
    if (b->key_hold_ms > 1000 || b->led_duration_ms > 60000)
        return CUSTOM_OUT_OF_RANGE;
    bool has_key = false;
    for (unsigned i = 0; i < 6; ++i) {
        uint8_t key = b->keycodes[i];
        if (key && (key < 4 || key >= 0xE0))
            return CUSTOM_OUT_OF_RANGE;
        has_key |= key != 0;
        for (unsigned j = 0; key && j < i; ++j)
            if (b->keycodes[j] == key)
                return CUSTOM_OUT_OF_RANGE;
    }
    if (b->keyboard_enabled && !has_key)
        return CUSTOM_OUT_OF_RANGE;
    portENTER_CRITICAL(&s_lock);
    s_bindings[index] = *b;
    ++s_revision;
    s_action_revision[index] = s_revision;
    portEXIT_CRITICAL(&s_lock);
    return CUSTOM_OK;
}
void custom_state_capture(custom_saved_config_t *s) {
    memset(s, 0, sizeof(*s));
    portENTER_CRITICAL(&s_lock);
    s->slot = s_slot;
    memcpy(s->package_sha, s_meta.package_sha, 32);
    memcpy(s->bindings, s_bindings, sizeof(s_bindings));
    portEXIT_CRITICAL(&s_lock);
}
void custom_state_mark_saved(uint32_t revision) {
    s_saved_revision = revision;
}
void custom_set_runtime_state(unsigned state, esp_err_t error) {
    s_runtime = state;
    s_error = error;
}
unsigned custom_runtime_state(void) {
    return s_runtime;
}
esp_err_t custom_runtime_error(void) {
    return s_error;
}
bool custom_maintenance_active(void) {
    return __atomic_load_n(&s_maintenance, __ATOMIC_ACQUIRE) != 0;
}
void custom_worker_register(uint32_t bit) {
    __atomic_fetch_or(&s_registered, bit, __ATOMIC_RELEASE);
}
bool custom_worker_checkpoint(uint32_t bit) {
    if (!custom_maintenance_active())
        return false;
    __atomic_fetch_or(&s_paused, bit, __ATOMIC_RELEASE);
    while (custom_maintenance_active())
        vTaskDelay(pdMS_TO_TICKS(2));
    __atomic_fetch_and(&s_paused, ~bit, __ATOMIC_RELEASE);
    return true;
}
esp_err_t custom_maintenance_enter(void) {
    __atomic_store_n(&s_maintenance, 1, __ATOMIC_RELEASE);
    /* Fence every action already queued before the discontinuity, even if
     * this maintenance cycle only downloads and does not activate a model. */
    __atomic_fetch_add(&s_epoch, 1, __ATOMIC_RELEASE);
    usb_audio_set_maintenance(true);
    TickType_t start = xTaskGetTickCount();
    while ((__atomic_load_n(&s_paused, __ATOMIC_ACQUIRE) & s_registered) != s_registered) {
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(3000))
            return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    /* Allow already-armed capture DMA packets to drain into zero packets. */
    vTaskDelay(pdMS_TO_TICKS(3));
    return ESP_OK;
}
void custom_maintenance_leave(void) {
    __atomic_store_n(&s_maintenance, 0, __ATOMIC_RELEASE);
    while (__atomic_load_n(&s_paused, __ATOMIC_ACQUIRE))
        vTaskDelay(pdMS_TO_TICKS(2));
    usb_audio_set_maintenance(false);
}
