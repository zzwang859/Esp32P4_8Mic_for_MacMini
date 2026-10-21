#pragma once

#include <stdint.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_RING_PHYSICAL_LED_COUNT CONFIG_WS2812_LED_COUNT
#define LED_RING_EFFECT_LED_COUNT   42U
#define LED_RING_FPS                CONFIG_WS2812_LED_RING_FPS
#define LED_RING_MAX_DOTS           5U
#define LED_RING_FRAME_DT_S (1.0f / (float)LED_RING_FPS)
#define LED_RING_FRAME_PERIOD_MS ((1000U + LED_RING_FPS - 1U) / LED_RING_FPS)

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_ring_rgb_t;

typedef enum {
    LED_RING_MODE_OFF = 0,
    LED_RING_MODE_RAINBOW,
    LED_RING_MODE_BREATHING,
    LED_RING_MODE_DOT,
    LED_RING_MODE_BLINK,
    LED_RING_MODE_A_GROUP_TEST,
} led_ring_mode_t;

typedef enum {
    LED_RING_CMD_REFRESH = 0,
    LED_RING_CMD_SET_MODE,
    LED_RING_CMD_SET_GLOBAL_PARAMS,
    LED_RING_CMD_SET_DOT_PARAMS,
    LED_RING_CMD_RESET_STATE,
} led_ring_command_t;

typedef struct {
    float color_hue_deg;
    float cycle_s;
} led_ring_global_params_t;

typedef struct {
    uint8_t enabled;
    float color_hue_deg;
    float cycle_s;
    float phase_deg;
    uint8_t clockwise;
    uint8_t tail_mode;
} led_ring_dot_params_t;

typedef struct {
    led_ring_command_t command;
    led_ring_mode_t mode;
    led_ring_global_params_t global;
    uint8_t dot_index;
    led_ring_dot_params_t dot;
} led_ring_control_t;

extern led_ring_rgb_t led_ring_rgb[LED_RING_EFFECT_LED_COUNT];

/** Update effect state or render the next frame when control is NULL. */
void led_ring_control(const led_ring_control_t *control);

#ifdef __cplusplus
}
#endif
