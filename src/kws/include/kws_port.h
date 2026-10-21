#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { KWS_CHUNK_SAMPLES = 160, KWS_CLASS_COUNT = 8, KWS_MAX_CLASS_COUNT = 34 };
typedef void *kws_handle_t;

esp_err_t kws_create(kws_handle_t *handle);
esp_err_t kws_reset(kws_handle_t handle);
esp_err_t kws_process(kws_handle_t handle, const int32_t *pcm, size_t samples,
                      float *logits, int *class_index); /* Capacity >= current class count. */
void kws_destroy(kws_handle_t *handle);

#ifdef __cplusplus
}
#endif
