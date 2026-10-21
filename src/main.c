#include "audio_backend.h"
#include "audio_diagnostics.h"
#include "cpu_usage.h"
#include "esp_check.h"
#include "esp_log.h"
#include "pcmd3180.h"
#include "usb_audio.h"
#include "sdkconfig.h"
#include "audio_pipeline.h"
#include "kws_stream.h"
#include "keyword_actions.h"
#include "custom_control.h"

#if CONFIG_WS2812_LED_RING_ENABLE
#include "led_ring.h"
#endif

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(audio_diagnostics_init());
    ESP_ERROR_CHECK(cpu_usage_monitor_start());
    ESP_ERROR_CHECK(custom_control_init());

    /* PCMD3180 is an ASI slave, so start the ESP32-P4 TDM clocks first. */
    ESP_ERROR_CHECK(audio_backend_init());
    esp_err_t codec_err = pcmd3180_init(NULL);
    if (codec_err != ESP_OK) {
        ESP_LOGE(TAG, "PCMD3180 unavailable: %s; continuing with silent capture",
                 esp_err_to_name(codec_err));
    }
    audio_pipeline_set_capture_available(codec_err == ESP_OK);
    esp_err_t processing_err = audio_pipeline_init();
    if (processing_err != ESP_OK) {
        ESP_LOGE(TAG, "audio processing failed: %s; USB remains available, mono capture is silent",
                 esp_err_to_name(processing_err));
    }
    ESP_ERROR_CHECK(usb_audio_init());
#if CONFIG_WS2812_LED_RING_ENABLE
    ESP_ERROR_CHECK(led_ring_start());
#endif
    if (processing_err == ESP_OK) {
        ESP_ERROR_CHECK(keyword_actions_init());
        esp_err_t kws_err = kws_stream_start();
        if (kws_err != ESP_OK && custom_model_partition()) {
            ESP_LOGE(TAG, "stored KWS model failed; loading embedded fallback");
            custom_control_boot_fallback();
            kws_err = kws_stream_start();
        }
        if (kws_err != ESP_OK) {
            custom_set_runtime_state(4, kws_err);
            ESP_LOGE(TAG, "KWS unavailable: %s", esp_err_to_name(kws_err));
        }
    }
    ESP_ERROR_CHECK(custom_control_start());
    ESP_LOGI(TAG, "ESP32-P4 USB audio started (PCMD3180 %s)",
             codec_err == ESP_OK ? "ready" : "unavailable");
}
