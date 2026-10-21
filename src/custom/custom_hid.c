#include "custom_control.h"
#include <string.h>
#include "tusb.h"

/* TinyUSB owns one callback pair for all HID interfaces. Instance 0 remains
 * the existing keyboard; only instance 1 accepts the management protocol. */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                               uint8_t *buffer, uint16_t length) {
    if (instance != 0 || report_id != 0 || type != HID_REPORT_TYPE_INPUT)
        return 0;
    uint16_t n = length < 8 ? length : 8;
    memset(buffer, 0, n);
    return n;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type,
                           const uint8_t *buffer, uint16_t size) {
    if (instance == CUSTOM_HID_INSTANCE && report_id == 0 && type == HID_REPORT_TYPE_OUTPUT)
        custom_hid_receive(buffer, size);
}
