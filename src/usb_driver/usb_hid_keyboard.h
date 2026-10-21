#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t modifier; /* HID modifier mask, e.g. KEYBOARD_MODIFIER_LEFTCTRL. */
    uint8_t keycodes[6]; /* Simultaneously pressed HID usages; unused entries are zero. */
} usb_hid_keyboard_chord_t;

typedef enum {
    USB_HID_KEYBOARD_CTRL_C = 0,
    USB_HID_KEYBOARD_CTRL_V,
    USB_HID_KEYBOARD_A,
} usb_hid_keyboard_action_t;

/** Start the HID keyboard report worker. TinyUSB must already be initialized. */
esp_err_t usb_hid_keyboard_init(void);

/** Queue one complete key press/release sequence without blocking the caller. */
esp_err_t usb_hid_keyboard_send(usb_hid_keyboard_action_t action);

/** Copy a configurable chord into the worker queue; never waits for USB. */
esp_err_t usb_hid_keyboard_send_chord(const usb_hid_keyboard_chord_t *chord);
esp_err_t usb_hid_keyboard_send_bound(int class_index, uint32_t epoch, uint32_t revision,
    const usb_hid_keyboard_chord_t *chord, uint16_t hold_ms);
esp_err_t usb_hid_keyboard_barrier(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
