#pragma once

#include <stddef.h>
#include <stdint.h>

// Copy a row-major matrix to its transpose. Tiles also handle odd dimensions
// such as the model's 98-frame state, without padding or changing tensor data.
static inline void kws_transpose_u8(uint8_t *dst, const uint8_t *src, size_t rows, size_t cols)
{
    const size_t tile = 16;
    for (size_t row0 = 0; row0 < rows; row0 += tile) {
        const size_t row_end = rows - row0 < tile ? rows : row0 + tile;
        for (size_t col0 = 0; col0 < cols; col0 += tile) {
            const size_t col_end = cols - col0 < tile ? cols : col0 + tile;
            for (size_t row = row0; row < row_end; ++row) {
                const uint8_t *in = src + row * cols + col0;
                uint8_t *out = dst + col0 * rows + row;
                for (size_t col = col0; col < col_end; ++col) {
                    *out = *in++;
                    out += rows;
                }
            }
        }
    }
}

static inline void kws_transpose_u16(uint16_t *dst, const uint16_t *src, size_t rows, size_t cols)
{
    const size_t tile = 16;
    for (size_t row0 = 0; row0 < rows; row0 += tile) {
        const size_t row_end = rows - row0 < tile ? rows : row0 + tile;
        for (size_t col0 = 0; col0 < cols; col0 += tile) {
            const size_t col_end = cols - col0 < tile ? cols : col0 + tile;
            for (size_t row = row0; row < row_end; ++row) {
                const uint16_t *in = src + row * cols + col0;
                uint16_t *out = dst + col0 * rows + row;
                for (size_t col = col0; col < col_end; ++col) {
                    *out = *in++;
                    out += rows;
                }
            }
        }
    }
}
