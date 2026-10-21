#include "keyword_actions.h"
#include "custom_control.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#if CONFIG_WS2812_LED_RING_ENABLE
#include "led_ring.h"
#endif

typedef struct {
    int index;
    uint32_t epoch, revision;
} action_event_t;
static QueueHandle_t s_events;
static void keyword_actions_task(void *arg) {
    (void)arg;
    action_event_t event;
    while (true) {
        if (xQueueReceive(s_events, &event, portMAX_DELAY) != pdTRUE)
            continue;
        custom_binding_t b;
        uint32_t revision;
        if (!custom_binding_get(event.index, &b, &revision) || revision != event.revision ||
            event.epoch != custom_model_epoch() || !b.actions_enabled ||
            custom_maintenance_active())
            continue;
#if CONFIG_WS2812_LED_RING_ENABLE
        if (b.led_enabled &&
            custom_action_allowed(event.index, event.epoch, event.revision, false)) {
            led_ring_rgb_t color = {b.r, b.g, b.b};
            led_ring_keyword_options_t options = {.duration_ms = b.led_duration_ms,
                                                  .guarded = true,
                                                  .class_index = event.index,
                                                  .model_epoch = event.epoch,
                                                  .config_revision = event.revision};
            (void)led_ring_trigger_keyword(color, &options);
        }
#endif
        if (b.keyboard_enabled &&
            custom_action_allowed(event.index, event.epoch, event.revision, true)) {
            usb_hid_keyboard_chord_t chord = {.modifier = b.modifier};
            memcpy(chord.keycodes, b.keycodes, 6);
            (void)usb_hid_keyboard_send_bound(event.index, event.epoch, event.revision, &chord,
                                              b.key_hold_ms);
        }
    }
}
esp_err_t keyword_actions_init(void) {
    if (s_events)
        return ESP_ERR_INVALID_STATE;
    s_events = xQueueCreate(8, sizeof(action_event_t));
    if (!s_events)
        return ESP_ERR_NO_MEM;
    if (xTaskCreate(keyword_actions_task, "keyword_actions", 3072, NULL, 2, NULL) != pdPASS) {
        vQueueDelete(s_events);
        s_events = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
bool keyword_actions_submit(int index) {
    if (index < 2 || index >= custom_class_count())
        return false;
    /* Recognition acknowledgement and host events never depend on action enable. */
    custom_keyword_event(index);
    custom_binding_t b;
    uint32_t revision;
    if (!s_events || !custom_binding_get(index, &b, &revision) || !b.actions_enabled ||
        custom_maintenance_active())
        return true;
    action_event_t event = {index, custom_model_epoch(), revision};
    (void)xQueueSend(s_events, &event, 0);
    return true;
}
