#pragma once
#include "FreeRTOS.h"
void vTaskDelay(TickType_t);
BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *, int);
enum eTaskState { eReady };
struct TaskStatus_t { configRUN_TIME_COUNTER_TYPE ulRunTimeCounter; };
void vTaskGetInfo(TaskHandle_t, TaskStatus_t *, BaseType_t, eTaskState);
