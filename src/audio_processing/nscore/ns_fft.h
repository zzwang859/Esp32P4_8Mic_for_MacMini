#pragma once
#include <stdint.h>

typedef struct { uint16_t length; } nscore_fft_instance;
#ifdef __cplusplus
extern "C" {
#endif
// 256-point real FFT with the original CMSIS packed format:
// [DC, Nyquist, Re(1), Im(1), ..., Re(127), Im(127)]. Inverse is scaled by 1/N.
int nscore_fft_init(nscore_fft_instance *plan, uint16_t length);
void nscore_fft(const nscore_fft_instance *plan, const float *input, float *output, uint8_t inverse);
#ifdef __cplusplus
}
#endif
