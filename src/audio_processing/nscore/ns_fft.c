#include "ns_fft.h"
#include <math.h>

// The original algorithm is single-instance. Scratch is static, never on the
// audio task stack; no allocation, trigonometry or table setup per frame.
static float roots[256];
static float work[512];
static int initialized;

int nscore_fft_init(nscore_fft_instance *plan, uint16_t length) {
    if (!plan || length != 256) return -1;
    plan->length = length;
    if (!initialized) {
        for (int k = 0; k < 128; ++k) {
            const float phase = -6.2831853071795864769f * k / 256.0f;
            roots[2*k] = cosf(phase);
            roots[2*k+1] = sinf(phase);
        }
        initialized = 1;
    }
    return 0;
}

void nscore_fft(const nscore_fft_instance *plan, const float *input, float *output, uint8_t inverse) {
    (void)plan; // All instances are the fixed 256-point plan checked at init.
    if (!inverse) {
        for (int i = 0; i < 256; ++i) { work[2*i] = input[i]; work[2*i+1] = 0; }
    } else {
        work[0] = input[0]; work[1] = 0;
        work[256] = input[1]; work[257] = 0;
        for (int k = 1; k < 128; ++k) {
            work[2*k] = work[512-2*k] = input[2*k];
            work[2*k+1] = input[2*k+1];
            work[513-2*k] = -input[2*k+1];
        }
    }
    for (unsigned i = 1, j = 0; i < 256; ++i) {
        unsigned bit = 128;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = work[2*i]; work[2*i] = work[2*j]; work[2*j] = t;
            t = work[2*i+1]; work[2*i+1] = work[2*j+1]; work[2*j+1] = t;
        }
    }
    for (int length = 2; length <= 256; length *= 2) {
        for (int start = 0; start < 256; start += length) {
            for (int j = 0; j < length/2; ++j) {
                const int root = j * (256 / length);
                const float wr = roots[2*root], wi = inverse ? -roots[2*root+1] : roots[2*root+1];
                const int a = 2*(start+j), b = 2*(start+j+length/2);
                const float vr = work[b]*wr - work[b+1]*wi;
                const float vi = work[b]*wi + work[b+1]*wr;
                const float ur = work[a], ui = work[a+1];
                work[a] = ur + vr; work[a+1] = ui + vi;
                work[b] = ur - vr; work[b+1] = ui - vi;
            }
        }
    }
    if (inverse) {
        for (int i = 0; i < 256; ++i) output[i] = work[2*i] / 256.0f;
    } else {
        output[0] = work[0]; output[1] = work[256];
        for (int k = 1; k < 128; ++k) { output[2*k] = work[2*k]; output[2*k+1] = work[2*k+1]; }
    }
}
