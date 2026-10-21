#pragma once
#include "FreeRTOS.h"
#include <stddef.h>
using QueueHandle_t = void *;
using StaticQueue_t = int;
QueueHandle_t xQueueCreateStatic(unsigned, unsigned, uint8_t *, StaticQueue_t *);
void vQueueDelete(QueueHandle_t);
BaseType_t xQueueSend(QueueHandle_t, const void *, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t);
