#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "usb_hid_keyboard.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEYWORD_ACTION_FLASH_MS 600U

/** Call after HID and optional LED initialization, before starting KWS. */
esp_err_t keyword_actions_init(void);
/** Zero-wait recognition acknowledgement. Disabled actions still return true;
 * saturated peripheral queues cannot change classification/debounce behavior. */
bool keyword_actions_submit(int class_index);

#ifdef __cplusplus
}
#endif
