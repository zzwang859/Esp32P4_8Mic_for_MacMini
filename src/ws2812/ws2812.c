#include "ws2812.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#define WS2812_RMT_RESOLUTION_HZ 20000000U
#define WS2812_RESET_US          80U
#define WS2812_BYTES_PER_PIXEL   3U

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t reset_code;
    int state;
} ws2812_rmt_encoder_t;

static const char *TAG = "ws2812";
static rmt_channel_handle_t s_tx_channel;
static rmt_encoder_handle_t s_encoder;
static SemaphoreHandle_t s_lock;
static uint8_t s_grb[CONFIG_WS2812_LED_COUNT * WS2812_BYTES_PER_PIXEL];

RMT_ENCODER_FUNC_ATTR
static size_t encode_ws2812(rmt_encoder_t *encoder,
                            rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2812_rmt_encoder_t *ws_encoder =
        __containerof(encoder, ws2812_rmt_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws_encoder->state) {
    case 0:
        encoded_symbols += ws_encoder->bytes_encoder->encode(
            ws_encoder->bytes_encoder, channel, primary_data, data_size,
            &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        __attribute__((fallthrough));
    case 1:
        encoded_symbols += ws_encoder->copy_encoder->encode(
            ws_encoder->copy_encoder, channel, &ws_encoder->reset_code,
            sizeof(ws_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws_encoder->state = 0;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        }
        break;
    default:
        ws_encoder->state = 0;
        state |= RMT_ENCODING_COMPLETE;
        break;
    }

out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t reset_ws2812_encoder(rmt_encoder_t *encoder)
{
    ws2812_rmt_encoder_t *ws_encoder =
        __containerof(encoder, ws2812_rmt_encoder_t, base);
    ESP_RETURN_ON_ERROR(rmt_encoder_reset(ws_encoder->bytes_encoder), TAG,
                        "reset bytes encoder");
    ESP_RETURN_ON_ERROR(rmt_encoder_reset(ws_encoder->copy_encoder), TAG,
                        "reset copy encoder");
    ws_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t delete_ws2812_encoder(rmt_encoder_t *encoder)
{
    ws2812_rmt_encoder_t *ws_encoder =
        __containerof(encoder, ws2812_rmt_encoder_t, base);
    esp_err_t first_error = ESP_OK;
    esp_err_t err = rmt_del_encoder(ws_encoder->bytes_encoder);
    if (err != ESP_OK) {
        first_error = err;
    }
    err = rmt_del_encoder(ws_encoder->copy_encoder);
    if (first_error == ESP_OK && err != ESP_OK) {
        first_error = err;
    }
    free(ws_encoder);
    return first_error;
}

static esp_err_t new_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    ESP_RETURN_ON_FALSE(ret_encoder, ESP_ERR_INVALID_ARG, TAG,
                        "encoder output is NULL");

    ws2812_rmt_encoder_t *ws_encoder =
        rmt_alloc_encoder_mem(sizeof(ws2812_rmt_encoder_t));
    ESP_RETURN_ON_FALSE(ws_encoder, ESP_ERR_NO_MEM, TAG,
                        "allocate RMT encoder");
    memset(ws_encoder, 0, sizeof(*ws_encoder));

    ws_encoder->base.encode = encode_ws2812;
    ws_encoder->base.reset = reset_ws2812_encoder;
    ws_encoder->base.del = delete_ws2812_encoder;

    /* 20 MHz gives 50 ns ticks and preserves the original 0.35/0.70 us timing. */
    const rmt_bytes_encoder_config_t bytes_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 7,
            .level1 = 0,
            .duration1 = 18,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 14,
            .level1 = 0,
            .duration1 = 11,
        },
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_config,
                                           &ws_encoder->bytes_encoder);
    if (err != ESP_OK) {
        free(ws_encoder);
        return err;
    }

    const rmt_copy_encoder_config_t copy_config = {};
    err = rmt_new_copy_encoder(&copy_config, &ws_encoder->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(ws_encoder->bytes_encoder);
        free(ws_encoder);
        return err;
    }

    const uint32_t half_reset_ticks =
        (WS2812_RMT_RESOLUTION_HZ / 1000000U) * WS2812_RESET_US / 2U;
    ws_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = half_reset_ticks,
        .level1 = 0,
        .duration1 = half_reset_ticks,
    };
    *ret_encoder = &ws_encoder->base;
    return ESP_OK;
}

static uint8_t scale_channel(uint8_t value, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)value * brightness + 127U) / 255U);
}

static esp_err_t transmit_locked(uint32_t timeout_ms)
{
    const rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
        .flags.queue_nonblocking = 0,
    };
    // The caller reports and rate-limits frame failures; avoid duplicate logs.
    esp_err_t err = rmt_transmit(s_tx_channel, s_encoder, s_grb,
                                sizeof(s_grb), &tx_config);
    if (err != ESP_OK) return err;
    return rmt_tx_wait_all_done(s_tx_channel, (int)timeout_ms);
}

esp_err_t ws2812_init(void)
{
    ESP_RETURN_ON_FALSE(!s_tx_channel, ESP_ERR_INVALID_STATE, TAG,
                        "driver already initialized");

    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "create framebuffer lock");

    const rmt_tx_channel_config_t channel_config = {
        .gpio_num = CONFIG_WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        .intr_priority = 0,
        .flags.invert_out = 0,
#if SOC_RMT_SUPPORT_DMA
        .flags.with_dma = 1,
#else
        .flags.with_dma = 0,
#endif
        .flags.allow_pd = 0,
        .flags.init_level = 0,
    };

    esp_err_t err = rmt_new_tx_channel(&channel_config, &s_tx_channel);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }

    err = new_ws2812_encoder(&s_encoder);
    if (err != ESP_OK) {
        rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }

    err = rmt_enable(s_tx_channel);
    if (err != ESP_OK) {
        rmt_del_encoder(s_encoder);
        s_encoder = NULL;
        rmt_del_channel(s_tx_channel);
        s_tx_channel = NULL;
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }

    memset(s_grb, 0, sizeof(s_grb));
    ESP_LOGI(TAG, "RMT TX ready: GPIO%d, %u pixels, 20 MHz, DMA=%d",
             CONFIG_WS2812_GPIO, CONFIG_WS2812_LED_COUNT,
             (int)SOC_RMT_SUPPORT_DMA);
    return ESP_OK;
}

esp_err_t ws2812_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_tx_channel && s_encoder && s_lock,
                        ESP_ERR_INVALID_STATE, TAG, "driver not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "lock framebuffer");

    esp_err_t err = rmt_disable(s_tx_channel);
    esp_err_t encoder_err = rmt_del_encoder(s_encoder);
    esp_err_t channel_err = rmt_del_channel(s_tx_channel);
    s_encoder = NULL;
    s_tx_channel = NULL;
    xSemaphoreGive(s_lock);
    vSemaphoreDelete(s_lock);
    s_lock = NULL;

    if (err != ESP_OK) {
        return err;
    }
    return encoder_err != ESP_OK ? encoder_err : channel_err;
}

size_t ws2812_pixel_count(void)
{
    return CONFIG_WS2812_LED_COUNT;
}

esp_err_t ws2812_set_pixel(size_t index, uint8_t red, uint8_t green,
                           uint8_t blue)
{
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG,
                        "driver not initialized");
    ESP_RETURN_ON_FALSE(index < CONFIG_WS2812_LED_COUNT,
                        ESP_ERR_INVALID_ARG, TAG, "pixel index out of range");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "lock framebuffer");
    s_grb[index * 3U + 0U] = green;
    s_grb[index * 3U + 1U] = red;
    s_grb[index * 3U + 2U] = blue;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t ws2812_fill(uint8_t red, uint8_t green, uint8_t blue)
{
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG,
                        "driver not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "lock framebuffer");
    for (size_t index = 0; index < CONFIG_WS2812_LED_COUNT; ++index) {
        s_grb[index * 3U + 0U] = green;
        s_grb[index * 3U + 1U] = red;
        s_grb[index * 3U + 2U] = blue;
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t ws2812_refresh(uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG,
                        "driver not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "lock framebuffer");
    esp_err_t err = transmit_locked(timeout_ms);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ws2812_show(const ws2812_color_t *pixels, size_t count,
                      uint8_t brightness, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(pixels && count == CONFIG_WS2812_LED_COUNT,
                        ESP_ERR_INVALID_ARG, TAG, "invalid RGB frame");
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG,
                        "driver not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE,
                        ESP_FAIL, TAG, "lock framebuffer");

    for (size_t index = 0; index < count; ++index) {
        s_grb[index * 3U + 0U] = scale_channel(pixels[index].g, brightness);
        s_grb[index * 3U + 1U] = scale_channel(pixels[index].r, brightness);
        s_grb[index * 3U + 2U] = scale_channel(pixels[index].b, brightness);
    }
    esp_err_t err = transmit_locked(timeout_ms);
    xSemaphoreGive(s_lock);
    return err;
}
