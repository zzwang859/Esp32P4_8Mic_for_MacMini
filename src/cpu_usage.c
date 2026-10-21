#include "cpu_usage.h"
#include "audio_diagnostics.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#define CPU_USAGE_PERIOD_MS 10000U
#define CPU_USAGE_TASK_STACK_SIZE 3072U
#define CPU_USAGE_PROBE_STACK_SIZE 2048U
#define CPU_USAGE_TASK_PRIORITY 2U

static const char *TAG = "cpu_usage";
static TaskHandle_t s_report_task;

static configRUN_TIME_COUNTER_TYPE get_core_idle_runtime(BaseType_t core)
{
#if CONFIG_FREERTOS_SMP
    return ulTaskGetRunTimeCounter(xTaskGetIdleTaskHandleForCore(core));
#else
    return ulTaskGetIdleRunTimeCounterForCore(core);
#endif
}

static double calculate_core_usage(configRUN_TIME_COUNTER_TYPE current_idle,
                                   configRUN_TIME_COUNTER_TYPE previous_idle,
                                   uint64_t elapsed_us)
{
    /* Unsigned subtraction also handles a 32-bit run-time counter wrap. */
    const configRUN_TIME_COUNTER_TYPE idle_delta = current_idle - previous_idle;
    uint64_t idle_us = (uint64_t)idle_delta;

    /* Scheduler accounting and timestamp reads are not simultaneous. */
    if (idle_us > elapsed_us) {
        idle_us = elapsed_us;
    }

    return elapsed_us == 0U
        ? 0.0
        : 100.0 * (double)(elapsed_us - idle_us) / (double)elapsed_us;
}

#if configNUMBER_OF_CORES == 2
static TaskHandle_t s_core1_probe_task;
static volatile configRUN_TIME_COUNTER_TYPE s_core1_idle_snapshot;

static void core1_probe_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Waking this task on CPU1 first switches out CPU1's Idle task, so its
         * run-time counter is current even when that core was continuously idle. */
        s_core1_idle_snapshot = get_core_idle_runtime(1);
        xTaskNotifyGive(s_report_task);
    }
}

static configRUN_TIME_COUNTER_TYPE sample_core1_idle_runtime(void)
{
    xTaskNotifyGive(s_core1_probe_task);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return s_core1_idle_snapshot;
}
#endif

static void cpu_usage_task(void *arg)
{
    (void)arg;

    configRUN_TIME_COUNTER_TYPE previous_idle[configNUMBER_OF_CORES];

    s_report_task = xTaskGetCurrentTaskHandle();
#if configNUMBER_OF_CORES == 2
    previous_idle[1] = sample_core1_idle_runtime();
#endif
    previous_idle[0] = get_core_idle_runtime(0);

    int64_t previous_time_us = esp_timer_get_time();
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CPU_USAGE_PERIOD_MS));

        configRUN_TIME_COUNTER_TYPE current_idle[configNUMBER_OF_CORES];
#if configNUMBER_OF_CORES == 2
        current_idle[1] = sample_core1_idle_runtime();
#endif
        current_idle[0] = get_core_idle_runtime(0);

        const int64_t current_time_us = esp_timer_get_time();
        const uint64_t elapsed_us =
            (uint64_t)(current_time_us - previous_time_us);
        const double core0_usage = calculate_core_usage(
            current_idle[0], previous_idle[0], elapsed_us);

#if configNUMBER_OF_CORES == 2
        const double core1_usage = calculate_core_usage(
            current_idle[1], previous_idle[1], elapsed_us);
        const double total_usage = (core0_usage + core1_usage) / 2.0;

        if (audio_diagnostics_info_enabled()) ESP_LOGI(TAG,
                 "CPU usage (10 s avg): core0=%.1f%%, core1=%.1f%%, total=%.1f%%",
                 core0_usage, core1_usage, total_usage);
#else
        if (audio_diagnostics_info_enabled())
            ESP_LOGI(TAG, "CPU usage (10 s avg): core0=%.1f%%", core0_usage);
#endif

        for (BaseType_t core = 0; core < configNUMBER_OF_CORES; ++core) {
            previous_idle[core] = current_idle[core];
        }
        previous_time_us = current_time_us;
    }
}

esp_err_t cpu_usage_monitor_start(void)
{
    BaseType_t created;

#if configNUMBER_OF_CORES == 2
    created = xTaskCreatePinnedToCore(core1_probe_task,
                                     "cpu1_probe",
                                     CPU_USAGE_PROBE_STACK_SIZE,
                                     NULL,
                                     CPU_USAGE_TASK_PRIORITY,
                                     &s_core1_probe_task,
                                     1);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "failed to create CPU1 usage probe task");
        return ESP_ERR_NO_MEM;
    }
#endif

    created = xTaskCreatePinnedToCore(cpu_usage_task,
                                     "cpu_usage",
                                     CPU_USAGE_TASK_STACK_SIZE,
                                     NULL,
                                     CPU_USAGE_TASK_PRIORITY,
                                     &s_report_task,
                                     0);
    if (created != pdPASS) {
#if configNUMBER_OF_CORES == 2
        vTaskDelete(s_core1_probe_task);
        s_core1_probe_task = NULL;
#endif
        ESP_LOGE(TAG, "failed to create CPU usage monitor task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "per-core CPU usage monitor started (period 10 s)");
    return ESP_OK;
}

