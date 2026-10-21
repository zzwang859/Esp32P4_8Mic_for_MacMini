#include "usb_hid_keyboard.h"
#include "custom_control.h"
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tusb.h"

typedef struct {
    usb_hid_keyboard_chord_t chord;
    int class_index;
    uint32_t epoch, revision, serial;
    uint16_t hold_ms;
} keyboard_message_t;
static QueueHandle_t s_queue;
static uint32_t s_issued, s_completed;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool allowed(const keyboard_message_t *m) {
    return !custom_maintenance_active() &&
           (m->class_index < 0 ||
            custom_action_allowed(m->class_index, m->epoch, m->revision, true));
}
static bool wait_ready(const keyboard_message_t *m, bool release) {
    TickType_t start = xTaskGetTickCount();
    do {
        if (!release && !allowed(m))
            return false;
        if (tud_hid_n_ready(0))
            return true;
        if (!tud_mounted())
            return false;
        vTaskDelay(pdMS_TO_TICKS(2));
    } while (xTaskGetTickCount() - start < pdMS_TO_TICKS(2000));
    return false;
}
static void keyboard_task(void *arg) {
    (void)arg;
    keyboard_message_t m;
    bool release_pending = false;
    while (true) {
        if (release_pending) {
            if (tud_hid_n_ready(0) && tud_hid_n_keyboard_report(0, 0, 0, NULL))
                release_pending = false;
            else {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
        }
        if (xQueueReceive(s_queue, &m, portMAX_DELAY) != pdTRUE)
            continue;
        if (wait_ready(&m, false) && allowed(&m) &&
            tud_hid_n_keyboard_report(0, 0, m.chord.modifier, m.chord.keycodes)) {
            TickType_t start = xTaskGetTickCount();
            while (allowed(&m) && xTaskGetTickCount() - start < pdMS_TO_TICKS(m.hold_ms))
                vTaskDelay(pdMS_TO_TICKS(2));
            if (!wait_ready(&m, true) || !tud_hid_n_keyboard_report(0, 0, 0, NULL))
                release_pending = true;
        }
        __atomic_store_n(&s_completed, m.serial, __ATOMIC_RELEASE);
    }
}
esp_err_t usb_hid_keyboard_init(void) {
    if (s_queue)
        return ESP_ERR_INVALID_STATE;
    s_queue = xQueueCreate(8, sizeof(keyboard_message_t));
    if (!s_queue)
        return ESP_ERR_NO_MEM;
    if (xTaskCreate(keyboard_task, "usb_keyboard", 3072, NULL,
                    CONFIG_USB_AUDIO_USB_TASK_PRIORITY - 1, NULL) != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
static esp_err_t enqueue(keyboard_message_t *m) {
    if (!s_queue)
        return ESP_ERR_INVALID_STATE;
    bool has_key = false;
    for (unsigned i = 0; i < 6; ++i) {
        uint8_t key = m->chord.keycodes[i];
        if (key && (key < 4 || key >= 0xe0))
            return ESP_ERR_INVALID_ARG;
        has_key |= key != 0;
    }
    if (!has_key)
        return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&s_lock);
    m->serial = s_issued + 1;
    BaseType_t result = xQueueSend(s_queue, m, 0);
    if (result == pdTRUE)
        s_issued = m->serial;
    portEXIT_CRITICAL(&s_lock);
    return result == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}
esp_err_t usb_hid_keyboard_send_bound(int index, uint32_t epoch, uint32_t revision,
                                      const usb_hid_keyboard_chord_t *chord, uint16_t hold_ms) {
    if (!chord)
        return ESP_ERR_INVALID_ARG;
    keyboard_message_t m = {.chord = *chord,
                            .class_index = index,
                            .epoch = epoch,
                            .revision = revision,
                            .hold_ms = hold_ms};
    return enqueue(&m);
}
esp_err_t usb_hid_keyboard_send_chord(const usb_hid_keyboard_chord_t *chord) {
    return usb_hid_keyboard_send_bound(-1, 0, 0, chord, 0);
}
esp_err_t usb_hid_keyboard_send(usb_hid_keyboard_action_t action) {
    usb_hid_keyboard_chord_t chord = {0};
    if (action == USB_HID_KEYBOARD_CTRL_C) {
        chord.modifier = KEYBOARD_MODIFIER_LEFTCTRL;
        chord.keycodes[0] = HID_KEY_C;
    } else if (action == USB_HID_KEYBOARD_CTRL_V) {
        chord.modifier = KEYBOARD_MODIFIER_LEFTCTRL;
        chord.keycodes[0] = HID_KEY_V;
    } else if (action == USB_HID_KEYBOARD_A)
        chord.keycodes[0] = HID_KEY_A;
    else
        return ESP_ERR_INVALID_ARG;
    return usb_hid_keyboard_send_chord(&chord);
}
esp_err_t usb_hid_keyboard_barrier(uint32_t timeout_ms) {
    if (!s_queue)
        return ESP_OK;
    uint32_t serial = __atomic_load_n(&s_issued, __ATOMIC_ACQUIRE);
    TickType_t start = xTaskGetTickCount();
    while ((int32_t)(__atomic_load_n(&s_completed, __ATOMIC_ACQUIRE) - serial) < 0) {
        if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(timeout_ms))
            return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_OK;
}
