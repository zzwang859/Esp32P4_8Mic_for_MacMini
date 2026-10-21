#pragma once
#include <stdint.h>
using BaseType_t = int;
using TickType_t = uint32_t;
using TaskHandle_t = void *;
using configRUN_TIME_COUNTER_TYPE = uint32_t;
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
