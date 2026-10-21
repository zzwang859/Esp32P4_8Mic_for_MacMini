#include "kws_frontend.h"

#include <math.h>
#include <string.h>

#include "kws_frontend_data.h"

void kws_frontend_reset(kws_frontend_t *frontend)
{
    memset(frontend, 0, sizeof(*frontend));
}

void kws_frontend_process(kws_frontend_t *frontend, const int32_t pcm[160], float features[40])
{
    // Same 30 ms history and 10 ms stride as data_frame_1 in the source ONNX.
    memmove(frontend->history, frontend->history + 160, 320 * sizeof(float));
    for (int i = 0; i < 160; ++i) {
        frontend->history[320 + i] = (float)pcm[i] / 2147483648.0f;
    }
    float *fft = frontend->fft;
    unsigned reversed = 0;
    for (unsigned i = 0; i < 512; ++i) {
        fft[2 * reversed] = i < 480 ? frontend->history[i] * kKwsWindow[i] : 0.0f;
        fft[2 * reversed + 1] = 0.0f;
        unsigned bit = 256;
        while (reversed & bit) {
            reversed ^= bit;
            bit >>= 1;
        }
        reversed ^= bit;
    }
    // Unscaled forward complex FFT, retaining exactly source bins 0..223.
    for (unsigned span = 2; span <= 512; span *= 2) {
        const unsigned half = span / 2;
        const unsigned step = 512 / span;
        for (unsigned offset = 0; offset < 512; offset += span) {
            for (unsigned j = 0; j < half; ++j) {
                const unsigned a = 2 * (offset + j);
                const unsigned b = 2 * (offset + j + half);
                const float wr = kKwsTwiddleReal[j * step];
                const float wi = kKwsTwiddleImag[j * step];
                const float real = wr * fft[b] - wi * fft[b + 1];
                const float imaginary = wr * fft[b + 1] + wi * fft[b];
                const float ar = fft[a], ai = fft[a + 1];
                fft[a] = ar + real;
                fft[a + 1] = ai + imaginary;
                fft[b] = ar - real;
                fft[b + 1] = ai - imaginary;
            }
        }
    }
    float magnitude[224];
    for (int i = 0; i < 224; ++i) {
        magnitude[i] = sqrtf(fft[2*i] * fft[2*i] + fft[2*i+1] * fft[2*i+1]);
    }
    float log_mel[80];
    for (int band = 0; band < 80; ++band) {
        float sum = 0.0f;
        for (int i = kKwsMelOffsets[band]; i < kKwsMelOffsets[band+1]; ++i) {
            sum += magnitude[kKwsMelBins[i]] * kKwsMelWeights[i];
        }
        // Preserve the source's positive floor, including completely silent frames.
        log_mel[band] = logf(fmaxf(sum, 1e-12f));
    }
    for (int coefficient = 0; coefficient < 40; ++coefficient) {
        float sum = 0.0f;
        for (int band = 0; band < 80; ++band) {
            sum += log_mel[band] * kKwsDct[band * 40 + coefficient];
        }
        features[coefficient] = sum * kKwsNormalization[coefficient];
    }
}
