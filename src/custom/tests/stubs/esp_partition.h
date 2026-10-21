#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct esp_partition_t { const char *label; uint32_t size; } esp_partition_t;
#define ESP_PARTITION_TYPE_DATA 1
#define ESP_PARTITION_SUBTYPE_ANY 255
const esp_partition_t *esp_partition_find_first(unsigned type, unsigned subtype, const char *label);
esp_err_t esp_partition_read(const esp_partition_t *part, size_t offset, void *data, size_t size);
esp_err_t esp_partition_write(const esp_partition_t *part, size_t offset, const void *data, size_t size);
esp_err_t esp_partition_erase_range(const esp_partition_t *part, size_t offset, size_t size);
