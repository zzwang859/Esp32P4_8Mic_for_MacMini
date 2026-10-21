#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// Single owner: post-AFE NS task. 16 kHz, PCM16, 160-sample / 10-ms steps.
// 1..99% sets the spectral gain floor; 100% is an exact PCM bypass.
void array_nscore_init(unsigned min_gain_percent);
void array_nscore_process(const int16_t input[160], int16_t output[160]);
#ifdef __cplusplus
}
#endif
