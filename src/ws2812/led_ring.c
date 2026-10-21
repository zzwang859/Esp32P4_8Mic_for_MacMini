#include "led_ring.h"
#include "custom_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "audio_pipeline.h"
#include "audio_diagnostics.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "usb_hid_keyboard.h"
#include "ws2812.h"

#define WS2812_TX_TIMEOUT_MS          100U
#define LED_RING_BUTTON_GPIO          GPIO_NUM_35
#define LED_RING_BUTTON_POLL_MS       10U
#define LED_RING_BUTTON_DEBOUNCE_MS   40U
#define LED_RING_INPUT_TASK_STACK     3072U
#define LED_RING_GROUP_A_COUNT        8U
#define LED_RING_COMMAND_QUEUE_DEPTH  8U
#define LED_RING_KEYWORD_BLINK_MS     100U
#define LED_RING_FAULT_BRIGHTNESS     128U

_Static_assert(LED_RING_PHYSICAL_LED_COUNT == 50,
               "the A/B LED mapping requires CONFIG_WS2812_LED_COUNT=50");
_Static_assert(LED_RING_EFFECT_LED_COUNT + LED_RING_GROUP_A_COUNT ==
                   LED_RING_PHYSICAL_LED_COUNT,
               "logical LED groups must cover the physical chain");

typedef enum {
    LED_COMMAND_SET_MODE,
    LED_COMMAND_TRIGGER_KEYWORD,
    LED_COMMAND_SET_THRESHOLD,
} led_command_type_t;

typedef struct {
    led_ring_rgb_t color;
    led_ring_keyword_options_t options;
} led_keyword_request_t;

typedef struct {
    led_command_type_t type;
    led_ring_top_mode_t mode;
    led_keyword_request_t keyword;
    float threshold_db;
} led_command_t;

typedef struct {
    led_ring_top_mode_t mode;
    led_ring_top_mode_t interrupted_mode;
    led_keyword_request_t keyword;
    TickType_t keyword_started;
    TickType_t keyword_deadline;
    float threshold_db;
} led_runtime_t;

static const char *TAG = "led_ring";
static const uint8_t s_group_a_indices[LED_RING_GROUP_A_COUNT] = {
    0U, 1U, 11U, 17U, 23U, 29U, 35U, 41U,
};
static TaskHandle_t s_led_task;
static TaskHandle_t s_input_task;
static QueueHandle_t s_command_queue;
static volatile led_ring_top_mode_t s_current_mode = LED_RING_TOP_MODE_DEFAULT;
static volatile float s_audio_threshold_db =
    (float)CONFIG_WS2812_LED_AUDIO_THRESHOLD_DB;
static ws2812_color_t s_frame[LED_RING_PHYSICAL_LED_COUNT];

static bool is_persistent_mode(led_ring_top_mode_t mode)
{
    return mode == LED_RING_TOP_MODE_DEFAULT ||
           mode == LED_RING_TOP_MODE_CONVERSATION ||
           mode == LED_RING_TOP_MODE_FAULT;
}

static const char *mode_name(led_ring_top_mode_t mode)
{
    switch (mode) {
    case LED_RING_TOP_MODE_DEFAULT:
        return "mode 0/default";
    case LED_RING_TOP_MODE_CONVERSATION:
        return "mode 1/conversation";
    case LED_RING_TOP_MODE_KEYWORD:
        return "mode 2/keyword";
    case LED_RING_TOP_MODE_FAULT:
        return "mode 4/fault";
    case LED_RING_TOP_MODE_PREVIOUS:
        return "previous";
    default:
        return "unknown";
    }
}

static bool is_group_a(size_t physical_index)
{
    switch (physical_index) {
    case 0U:
    case 1U:
    case 11U:
    case 17U:
    case 23U:
    case 29U:
    case 35U:
    case 41U:
        return true;
    default:
        return false;
    }
}

static void apply_breathing_effect(void)
{
    const led_ring_control_t control = {
        .command = LED_RING_CMD_SET_MODE,
        .mode = LED_RING_MODE_BREATHING,
    };
    led_ring_control(&control);
}

static uint8_t configured_brightness(void)
{
    return (uint8_t)((CONFIG_WS2812_LED_RING_BRIGHTNESS * 255U + 50U) /
                     100U);
}

static uint8_t level_to_channel(float dbfs, float threshold_db)
{
    if (dbfs <= threshold_db) {
        return 0U;
    }
    if (dbfs >= 0.0f) {
        return 255U;
    }
    const float normalized = (dbfs - threshold_db) / -threshold_db;
    return (uint8_t)(normalized * 255.0f + 0.5f);
}

static void set_group_b_color(ws2812_color_t color)
{
    for (size_t physical_index = 0U;
         physical_index < LED_RING_PHYSICAL_LED_COUNT; ++physical_index) {
        if (!is_group_a(physical_index)) {
            s_frame[physical_index] = color;
        }
    }
}

static void build_audio_frame(led_ring_top_mode_t mode, float threshold_db)
{
    audio_pipeline_levels_t levels = {
        .processed_dbfs = -120.0f,
        .playback_dbfs = -120.0f,
    };
    audio_pipeline_get_levels(&levels);
    const uint8_t capture_level = level_to_channel(levels.processed_dbfs, threshold_db);
    const uint8_t playback_level =
        level_to_channel(levels.playback_dbfs, threshold_db);

    led_ring_control(NULL);
    size_t logical_index = 0U;
    for (size_t physical_index = 0U;
         physical_index < LED_RING_PHYSICAL_LED_COUNT; ++physical_index) {
        if (is_group_a(physical_index)) {
            continue;
        }
        const uint8_t breathing_level = led_ring_rgb[logical_index].g;
        if (mode == LED_RING_TOP_MODE_DEFAULT) {
            s_frame[physical_index] = (ws2812_color_t) {
                .r = capture_level,
                .g = breathing_level,
                .b = playback_level,
            };
        } else {
            s_frame[physical_index] = (ws2812_color_t) {
                .r = capture_level,
                .g = playback_level,
                .b = breathing_level,
            };
        }
        ++logical_index;
    }
}

static void build_physical_frame(const led_runtime_t *runtime, TickType_t now)
{
    memset(s_frame, 0, sizeof(s_frame));

    switch (runtime->mode) {
    case LED_RING_TOP_MODE_DEFAULT:
    case LED_RING_TOP_MODE_CONVERSATION:
        build_audio_frame(runtime->mode, runtime->threshold_db);
        break;
    case LED_RING_TOP_MODE_KEYWORD: {
        const TickType_t blink_ticks =
            pdMS_TO_TICKS(LED_RING_KEYWORD_BLINK_MS) > 0
                ? pdMS_TO_TICKS(LED_RING_KEYWORD_BLINK_MS)
                : 1;
        const TickType_t elapsed = now - runtime->keyword_started;
        if ((elapsed % blink_ticks) < (blink_ticks + 1U) / 2U) {
            set_group_b_color((ws2812_color_t) {
                .r = runtime->keyword.color.r,
                .g = runtime->keyword.color.g,
                .b = runtime->keyword.color.b,
            });
        }
        break;
    }
    case LED_RING_TOP_MODE_FAULT:
        for (size_t index = 0U; index < LED_RING_GROUP_A_COUNT; ++index) {
            s_frame[s_group_a_indices[index]].r = 255U;
        }
        break;
    case LED_RING_TOP_MODE_PREVIOUS:
    default:
        break;
    }
}

static void enter_persistent_mode(led_runtime_t *runtime,
                                  led_ring_top_mode_t mode)
{
    runtime->mode = mode;
    if (mode == LED_RING_TOP_MODE_DEFAULT ||
        mode == LED_RING_TOP_MODE_CONVERSATION) {
        apply_breathing_effect();
    }
    s_current_mode = mode;
}

static void process_command(led_runtime_t *runtime,
                            const led_command_t *command,
                            TickType_t now)
{
    switch (command->type) {
    case LED_COMMAND_SET_MODE:
        enter_persistent_mode(runtime, command->mode);
        break;
    case LED_COMMAND_TRIGGER_KEYWORD: {
        if (runtime->mode != LED_RING_TOP_MODE_KEYWORD) {
            runtime->interrupted_mode = runtime->mode;
        }
        if (command->keyword.options.guarded && !custom_action_allowed(
                command->keyword.options.class_index, command->keyword.options.model_epoch,
                command->keyword.options.config_revision, false)) break;
        runtime->keyword = command->keyword;
        runtime->keyword_started = now;
        TickType_t duration_ticks =
            pdMS_TO_TICKS(runtime->keyword.options.duration_ms);
        if (duration_ticks == 0) {
            duration_ticks = 1;
        }
        runtime->keyword_deadline = now + duration_ticks;
        runtime->mode = LED_RING_TOP_MODE_KEYWORD;
        s_current_mode = LED_RING_TOP_MODE_KEYWORD;
        break;
    }
    case LED_COMMAND_SET_THRESHOLD:
        runtime->threshold_db = command->threshold_db;
        s_audio_threshold_db = command->threshold_db;
        break;
    default:
        break;
    }
}

static void led_effect_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frame_period =
        pdMS_TO_TICKS(LED_RING_FRAME_PERIOD_MS) > 0
            ? pdMS_TO_TICKS(LED_RING_FRAME_PERIOD_MS)
            : 1;
    const uint8_t normal_brightness = configured_brightness();
    led_runtime_t runtime = {
        .mode = LED_RING_TOP_MODE_DEFAULT,
        .interrupted_mode = LED_RING_TOP_MODE_DEFAULT,
        .keyword_started = 0,
        .keyword_deadline = 0,
        .threshold_db = (float)CONFIG_WS2812_LED_AUDIO_THRESHOLD_DB,
    };

    s_current_mode = runtime.mode;
    s_audio_threshold_db = runtime.threshold_db;
    apply_breathing_effect();
    TickType_t last_error_log = 0;
    uint32_t frame_errors = 0;
    while (true) {
        const TickType_t now = xTaskGetTickCount();
        led_command_t command;
        while (xQueueReceive(s_command_queue, &command, 0) == pdTRUE) {
            process_command(&runtime, &command, now);
        }

        if (runtime.mode == LED_RING_TOP_MODE_KEYWORD &&
            (int32_t)(now - runtime.keyword_deadline) >= 0) {
            led_ring_top_mode_t next_mode = runtime.keyword.options.has_next_mode
                                                ? runtime.keyword.options.next_mode
                                                : runtime.interrupted_mode;
            enter_persistent_mode(&runtime, next_mode);
        }

        build_physical_frame(&runtime, now);
        const uint8_t brightness = runtime.mode == LED_RING_TOP_MODE_FAULT
                                       ? LED_RING_FAULT_BRIGHTNESS
                                       : normal_brightness;
        esp_err_t err = ws2812_show(s_frame, LED_RING_PHYSICAL_LED_COUNT,
                                    brightness, WS2812_TX_TIMEOUT_MS);
        if (err != ESP_OK) {
            ++frame_errors;
            const TickType_t error_tick = xTaskGetTickCount();
            if (frame_errors == 1U || error_tick - last_error_log >= pdMS_TO_TICKS(1000)) {
                ESP_LOGE(TAG, "RMT frame failed: %s (total=%lu)", esp_err_to_name(err),
                         (unsigned long)frame_errors);
                last_error_log = error_tick;
            }
        }
        xTaskDelayUntil(&last_wake, frame_period);
    }
}

static void print_console_help(void)
{
    if (!audio_diagnostics_info_enabled()) return;
    ESP_LOGI(TAG, "LED top modes: 0/i=default, 1/c=conversation");
    ESP_LOGI(TAG, "LED top modes: 2/k=keyword trigger, 4/f=fault, ?/h=help");
    ESP_LOGI(TAG, "Keyboard tests: 6=Ctrl+C, 7=Ctrl+V, 8=character a");
    ESP_LOGI(TAG, "KWS actions: a/A=enable, x/X=disable (LED and keyboard default off at boot)");
    ESP_LOGI(TAG, "DOA fixed SRP-PHAT; commands: e=log on, d=log off (default)");
    ESP_LOGI(TAG, "Logs: l/L toggles all non-KWS logs; KWS results always remain enabled");
    ESP_LOGI(TAG, "HID operation logs: u/U toggles UART feedback (default off, independent of l/L)");
    ESP_LOGI(TAG, "KWS raw logits: p/P toggles 10-ms CSV trace (default off)");
}

static void send_keyboard_test(usb_hid_keyboard_action_t action,
                               const char *name)
{
    esp_err_t err = usb_hid_keyboard_send(action);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "keyboard test queued: %s", name);
    } else {
        ESP_LOGE(TAG, "keyboard test %s failed: %s", name,
                 esp_err_to_name(err));
    }
}

static void set_mode_from_input(led_ring_top_mode_t mode, const char *source)
{
    esp_err_t err = led_ring_set_mode(mode);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s selected: %s", source, mode_name(mode));
    } else {
        ESP_LOGE(TAG, "%s mode change failed: %s", source,
                 esp_err_to_name(err));
    }
}

static void trigger_keyword_from_input(const char *source)
{
    const led_ring_rgb_t test_color = {.r = 191U, .g = 64U, .b = 223U};
    esp_err_t err = led_ring_trigger_keyword(test_color, NULL);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s selected: mode 2/keyword test", source);
    } else {
        ESP_LOGE(TAG, "%s keyword trigger failed: %s", source,
                 esp_err_to_name(err));
    }
}

static void handle_console_command(uint8_t command)
{
    if (audio_pipeline_console_command(command)) {
        return;
    }
    switch (command) {
    case '0':
    case 'i':
    case 'I':
        set_mode_from_input(LED_RING_TOP_MODE_DEFAULT, "console");
        break;
    case '1':
    case 'c':
    case 'C':
        set_mode_from_input(LED_RING_TOP_MODE_CONVERSATION, "console");
        break;
    case '2':
    case 'k':
    case 'K':
        trigger_keyword_from_input("console");
        break;
    case '4':
    case 'f':
    case 'F':
        set_mode_from_input(LED_RING_TOP_MODE_FAULT, "console");
        break;
    case '6':
        send_keyboard_test(USB_HID_KEYBOARD_CTRL_C, "Ctrl+C");
        break;
    case '7':
        send_keyboard_test(USB_HID_KEYBOARD_CTRL_V, "Ctrl+V");
        break;
    case '8':
        send_keyboard_test(USB_HID_KEYBOARD_A, "a");
        break;
    case '?':
    case 'h':
    case 'H':
        print_console_help();
        break;
    case '\r':
    case '\n':
    case ' ':
    case '\t':
        break;
    default:
        if (audio_diagnostics_info_enabled())
            ESP_LOGW(TAG, "unknown LED command '%c'; send ? for help", command);
        break;
    }
}

static led_ring_top_mode_t next_button_mode(led_ring_top_mode_t mode)
{
    if (mode == LED_RING_TOP_MODE_DEFAULT) {
        return LED_RING_TOP_MODE_CONVERSATION;
    }
    if (mode == LED_RING_TOP_MODE_CONVERSATION) {
        return LED_RING_TOP_MODE_FAULT;
    }
    return LED_RING_TOP_MODE_DEFAULT;
}

static void led_input_task(void *arg)
{
    (void)arg;
    int sampled_level = gpio_get_level(LED_RING_BUTTON_GPIO);
    int stable_level = sampled_level;
    TickType_t level_changed_at = xTaskGetTickCount();
    const TickType_t debounce_ticks =
        pdMS_TO_TICKS(LED_RING_BUTTON_DEBOUNCE_MS);

    print_console_help();
    while (true) {
        uint8_t command;
        while (uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &command, 1U, 0) ==
               1) {
            handle_console_command(command);
        }

        const TickType_t now = xTaskGetTickCount();
        const int level = gpio_get_level(LED_RING_BUTTON_GPIO);
        if (level != sampled_level) {
            sampled_level = level;
            level_changed_at = now;
        } else if (level != stable_level &&
                   now - level_changed_at >= debounce_ticks) {
            stable_level = level;
            if (stable_level == 0) {
                set_mode_from_input(next_button_mode(led_ring_get_mode()),
                                    "GPIO35 button");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(LED_RING_BUTTON_POLL_MS));
    }
}

static esp_err_t init_inputs(void)
{
    const gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << LED_RING_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&button_config), TAG,
                        "configure GPIO35 button");

    if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM)) {
        ESP_RETURN_ON_ERROR(
            uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0),
            TAG, "install console UART driver");
    }
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    return ESP_OK;
}

static esp_err_t queue_command(const led_command_t *command)
{
    ESP_RETURN_ON_FALSE(s_command_queue, ESP_ERR_INVALID_STATE, TAG,
                        "LED task not started");
    return xQueueSend(s_command_queue, command, 0) == pdTRUE ? ESP_OK
                                                             : ESP_ERR_TIMEOUT;
}

esp_err_t led_ring_start(void)
{
    ESP_RETURN_ON_FALSE(!s_led_task && !s_input_task && !s_command_queue,
                        ESP_ERR_INVALID_STATE, TAG, "LED tasks already started");
    s_command_queue = xQueueCreate(LED_RING_COMMAND_QUEUE_DEPTH,
                                   sizeof(led_command_t));
    ESP_RETURN_ON_FALSE(s_command_queue, ESP_ERR_NO_MEM, TAG,
                        "create LED command queue");

    esp_err_t err = ws2812_init();
    if (err == ESP_OK) {
        err = ws2812_fill(0, 0, 0);
    }
    if (err == ESP_OK) {
        err = ws2812_refresh(WS2812_TX_TIMEOUT_MS);
    }
    if (err == ESP_OK) {
        err = init_inputs();
    }
    if (err != ESP_OK) {
        (void)ws2812_deinit();
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        ESP_LOGE(TAG, "initialize LED ring: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t created = xTaskCreatePinnedToCore(
        led_effect_task, "led_ring", CONFIG_WS2812_LED_RING_TASK_STACK_SIZE,
        NULL, CONFIG_WS2812_LED_RING_TASK_PRIORITY, &s_led_task,
        tskNO_AFFINITY);
    if (created != pdPASS) {
        s_led_task = NULL;
        (void)ws2812_deinit();
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreatePinnedToCore(
        led_input_task, "led_input", LED_RING_INPUT_TASK_STACK, NULL,
        CONFIG_WS2812_LED_RING_TASK_PRIORITY, &s_input_task, tskNO_AFFINITY);
    if (created != pdPASS) {
        s_input_task = NULL;
        vTaskDelete(s_led_task);
        s_led_task = NULL;
        (void)ws2812_deinit();
        vQueueDelete(s_command_queue);
        s_command_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "started: mode 0, A=8/B=42, %d FPS, brightness %d%%, threshold %d dBFS",
             CONFIG_WS2812_LED_RING_FPS,
             CONFIG_WS2812_LED_RING_BRIGHTNESS,
             CONFIG_WS2812_LED_AUDIO_THRESHOLD_DB);
    return ESP_OK;
}

esp_err_t led_ring_set_mode(led_ring_top_mode_t mode)
{
    ESP_RETURN_ON_FALSE(is_persistent_mode(mode), ESP_ERR_INVALID_ARG, TAG,
                        "invalid top-level mode");
    const led_command_t command = {
        .type = LED_COMMAND_SET_MODE,
        .mode = mode,
    };
    return queue_command(&command);
}

esp_err_t led_ring_trigger_keyword(led_ring_rgb_t color,
                                   const led_ring_keyword_options_t *options)
{
    led_ring_keyword_options_t resolved = options
                                              ? *options
                                              : (led_ring_keyword_options_t) {0};
    if (resolved.duration_ms == 0U) {
        resolved.duration_ms = CONFIG_WS2812_LED_KEYWORD_DURATION_MS;
    }
    ESP_RETURN_ON_FALSE(!resolved.has_next_mode ||
                            is_persistent_mode(resolved.next_mode),
                        ESP_ERR_INVALID_ARG, TAG, "invalid keyword next mode");
    const led_command_t command = {
        .type = LED_COMMAND_TRIGGER_KEYWORD,
        .keyword = {
            .color = color,
            .options = resolved,
        },
    };
    return queue_command(&command);
}

esp_err_t led_ring_set_audio_threshold_db(float threshold_db)
{
    ESP_RETURN_ON_FALSE(threshold_db >= -96.0f && threshold_db < 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "invalid audio threshold");
    const led_command_t command = {
        .type = LED_COMMAND_SET_THRESHOLD,
        .threshold_db = threshold_db,
    };
    return queue_command(&command);
}

float led_ring_get_audio_threshold_db(void)
{
    return s_audio_threshold_db;
}

led_ring_top_mode_t led_ring_get_mode(void)
{
    return s_current_mode;
}
