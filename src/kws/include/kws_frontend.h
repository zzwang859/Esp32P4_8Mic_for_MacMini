#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { KWS_FEATURE_COUNT = 40 };
typedef struct {
    float history[480];
    float fft[1024];
} kws_frontend_t;

void kws_frontend_reset(kws_frontend_t *frontend);
void kws_frontend_process(kws_frontend_t *frontend, const int32_t pcm[160], float features[40]);

#ifdef __cplusplus
}
#endif
