#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start the task that reports average CPU usage every ten seconds. */
esp_err_t cpu_usage_monitor_start(void);

#ifdef __cplusplus
}
#endif

