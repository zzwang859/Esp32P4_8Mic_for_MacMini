#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize 8-channel capture and 2-channel playback hardware. */
esp_err_t audio_backend_init(void);

/** Read interleaved signed 16-bit PCM: ch0, ch1, ... ch7, next frame. */
esp_err_t audio_backend_read(void *dst, size_t bytes, size_t *bytes_read,
                             uint32_t timeout_ms);

/** Write interleaved signed 16-bit PCM: left, right, next frame. */
esp_err_t audio_backend_write(const void *src, size_t bytes,
                              size_t *bytes_written, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

