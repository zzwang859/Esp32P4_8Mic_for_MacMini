#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_color_t;

/** Initialize the ESP32-P4 RMT TX + GDMA WS2812B driver. */
esp_err_t ws2812_init(void);

/** Disable the RMT channel and release driver resources. */
esp_err_t ws2812_deinit(void);

/** Return the number of pixels configured by menuconfig. */
size_t ws2812_pixel_count(void);

/** Update one pixel in the driver's local framebuffer. */
esp_err_t ws2812_set_pixel(size_t index, uint8_t red, uint8_t green, uint8_t blue);

/** Update every pixel in the driver's local framebuffer. */
esp_err_t ws2812_fill(uint8_t red, uint8_t green, uint8_t blue);

/** Send the local framebuffer through RMT and wait for completion. */
esp_err_t ws2812_refresh(uint32_t timeout_ms);

/**
 * Scale and transmit an RGB frame in one locked operation.
 * brightness is 0..255 and count must match ws2812_pixel_count().
 */
esp_err_t ws2812_show(const ws2812_color_t *pixels, size_t count,
                      uint8_t brightness, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
