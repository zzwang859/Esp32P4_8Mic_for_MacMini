#include "audio_backend.h"
#include "audio_pipeline.h"

#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "usb_audio_config.h"

static const char *TAG = "audio_hw";
static i2s_chan_handle_t s_rx_chan;
static i2s_chan_handle_t s_tx_chan;

static esp_err_t validate_config(void)
{
    ESP_RETURN_ON_FALSE(CONFIG_USB_AUDIO_RX_I2S_PORT != CONFIG_USB_AUDIO_TX_I2S_PORT,
                        ESP_ERR_INVALID_ARG, TAG,
                        "RX and TX use different formats and must use different I2S ports");
    ESP_RETURN_ON_FALSE(CONFIG_USB_AUDIO_RX_BCLK_GPIO >= 0 &&
                        CONFIG_USB_AUDIO_RX_WS_GPIO >= 0 &&
                        CONFIG_USB_AUDIO_RX_DIN_GPIO >= 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "configure RX BCLK/WS/DIN GPIOs in USB audio menu");
    ESP_RETURN_ON_FALSE(CONFIG_USB_AUDIO_TX_BCLK_GPIO >= 0 &&
                        CONFIG_USB_AUDIO_TX_WS_GPIO >= 0 &&
                        CONFIG_USB_AUDIO_TX_DOUT_GPIO >= 0,
                        ESP_ERR_INVALID_ARG, TAG,
                        "configure TX BCLK/WS/DOUT GPIOs in USB audio menu");
    return ESP_OK;
}

esp_err_t audio_backend_init(void)
{
    ESP_RETURN_ON_ERROR(validate_config(), TAG, "invalid I2S configuration");

#if CONFIG_USB_AUDIO_RX_SLAVE
#define USB_AUDIO_RX_ROLE I2S_ROLE_SLAVE
#else
#define USB_AUDIO_RX_ROLE I2S_ROLE_MASTER
#endif
#if CONFIG_USB_AUDIO_TX_SLAVE
#define USB_AUDIO_TX_ROLE I2S_ROLE_SLAVE
#else
#define USB_AUDIO_TX_ROLE I2S_ROLE_MASTER
#endif

    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        CONFIG_USB_AUDIO_RX_I2S_PORT, USB_AUDIO_RX_ROLE);
    rx_chan_cfg.dma_desc_num = 8;
    /* Deliver 4-ms bursts instead of 10-ms bursts. Keep eight descriptors
     * (32 ms total capacity) to tolerate capture-task scheduling delays. */
    rx_chan_cfg.dma_frame_num =
        (USB_AUDIO_MIC_SAMPLE_RATE_HZ / 1000U) * USB_AUDIO_RX_DMA_PERIOD_MS;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&rx_chan_cfg, NULL, &s_rx_chan),
                        TAG, "create TDM RX channel");

    i2s_tdm_config_t rx_cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(USB_AUDIO_MIC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO,
            I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
            I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7),
        .gpio_cfg = {
            .mclk = (gpio_num_t)CONFIG_USB_AUDIO_RX_MCLK_GPIO,
            .bclk = (gpio_num_t)CONFIG_USB_AUDIO_RX_BCLK_GPIO,
            .ws = (gpio_num_t)CONFIG_USB_AUDIO_RX_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t)CONFIG_USB_AUDIO_RX_DIN_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    rx_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_512;
    /* Keep the slot-0 MSB aligned to the active FSYNC edge, but drive WCLK
     * high for half of the 8 x 16-bit frame to obtain a 50% duty cycle. */
    rx_cfg.slot_cfg.ws_width =
        (USB_AUDIO_MIC_CHANNELS * I2S_DATA_BIT_WIDTH_16BIT) / 2U;
    rx_cfg.slot_cfg.ws_pol = true;
    rx_cfg.slot_cfg.total_slot = USB_AUDIO_MIC_CHANNELS;
    ESP_RETURN_ON_ERROR(i2s_channel_init_tdm_mode(s_rx_chan, &rx_cfg),
                        TAG, "configure TDM RX channel");

    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        CONFIG_USB_AUDIO_TX_I2S_PORT, USB_AUDIO_TX_ROLE);
    /* Four 2-ms descriptors keep I2S ahead of USB without letting the
     * playback task consume a 20-ms software queue in a large DMA burst. */
    tx_chan_cfg.dma_desc_num = 4;
    tx_chan_cfg.dma_frame_num = USB_AUDIO_SPK_SAMPLE_RATE_HZ / 500U;
    tx_chan_cfg.auto_clear_after_cb = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan_cfg, &s_tx_chan, NULL),
                        TAG, "create stereo TX channel");

    i2s_std_config_t tx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(USB_AUDIO_SPK_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)CONFIG_USB_AUDIO_TX_MCLK_GPIO,
            .bclk = (gpio_num_t)CONFIG_USB_AUDIO_TX_BCLK_GPIO,
            .ws = (gpio_num_t)CONFIG_USB_AUDIO_TX_WS_GPIO,
            .dout = (gpio_num_t)CONFIG_USB_AUDIO_TX_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx_chan, &tx_cfg),
                        TAG, "configure stereo TX channel");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx_chan), TAG, "enable RX");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx_chan), TAG, "enable TX");

    ESP_LOGI(TAG, "I2S ready: RX 8ch MSB TDM %u Hz, TX 2ch %u Hz, 16-bit",
             USB_AUDIO_MIC_SAMPLE_RATE_HZ, USB_AUDIO_SPK_SAMPLE_RATE_HZ);
    return ESP_OK;
}

esp_err_t audio_backend_read(void *dst, size_t bytes, size_t *bytes_read,
                             uint32_t timeout_ms)
{
    return i2s_channel_read(s_rx_chan, dst, bytes, bytes_read,
                            timeout_ms);
}

esp_err_t audio_backend_write(const void *src, size_t bytes,
                              size_t *bytes_written, uint32_t timeout_ms)
{
    /* One USB millisecond is the largest block passed by usb_audio.c. */
    if (bytes > USB_AUDIO_SPK_MS_BYTES || (bytes % sizeof(int16_t)) != 0U) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* The host applies playback volume/mute before sending USB PCM. */
    esp_err_t err = i2s_channel_write(s_tx_chan, src, bytes, bytes_written,
                                      timeout_ms);
    /* Use the same unmodified PCM accepted by I2S as the AEC reference. */
    audio_pipeline_reference((const int16_t *)src,
                             *bytes_written / USB_AUDIO_SPK_FRAME_BYTES);
    return err;
}
