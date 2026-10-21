#include "custom_protocol.h"
#include <string.h>

uint16_t custom_get_u16(const uint8_t *p) {
    return p[0] | ((uint16_t)p[1] << 8);
}
uint32_t custom_get_u32(const uint8_t *p) {
    return custom_get_u16(p) | ((uint32_t)custom_get_u16(p + 2) << 16);
}
void custom_put_u16(uint8_t *p, uint16_t v) {
    p[0] = v;
    p[1] = v >> 8;
}
void custom_put_u32(uint8_t *p, uint32_t v) {
    custom_put_u16(p, v);
    custom_put_u16(p + 2, v >> 16);
}
uint32_t custom_crc32(const void *data, size_t size) {
    const uint8_t *p = data;
    uint32_t crc = UINT32_MAX;
    while (size--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}
bool custom_utf8_valid(const uint8_t *p, size_t size) {
    for (size_t i = 0; i < size;) {
        uint32_t c = p[i++];
        unsigned n = 0;
        uint32_t minimum = 0;
        if (c < 0x80)
            continue;
        if (c >= 0xc2 && c <= 0xdf) {
            n = 1;
            c &= 31;
            minimum = 0x80;
        } else if (c >= 0xe0 && c <= 0xef) {
            n = 2;
            c &= 15;
            minimum = 0x800;
        } else if (c >= 0xf0 && c <= 0xf4) {
            n = 3;
            c &= 7;
            minimum = 0x10000;
        } else
            return false;
        if (n > size - i)
            return false;
        while (n--) {
            uint8_t next = p[i++];
            if ((next & 0xc0) != 0x80)
                return false;
            c = (c << 6) | (next & 63);
        }
        if (c < minimum || c > 0x10ffff || (c >= 0xd800 && c <= 0xdfff))
            return false;
    }
    return true;
}
uint16_t custom_frame_decode(custom_frame_t *f, const uint8_t *p, size_t size) {
    memset(f, 0, sizeof(*f));
    if (!p || size != CUSTOM_REPORT_BYTES)
        return CUSTOM_BAD_LENGTH;
    f->command = custom_get_u16(p + 4);
    f->sequence = custom_get_u16(p + 6);
    f->transaction = custom_get_u32(p + 8);
    if (p[0] != 'K' || p[1] != 'C' || p[2] != CUSTOM_PROTOCOL_VERSION)
        return CUSTOM_BAD_VERSION;
    if (custom_crc32(p, 60) != custom_get_u32(p + 60))
        return CUSTOM_BAD_CRC;
    f->type = p[3];
    f->offset = custom_get_u32(p + 12);
    f->length = custom_get_u16(p + 16);
    f->status = custom_get_u16(p + 18);
    if (f->length > CUSTOM_PAYLOAD_BYTES || f->type != CUSTOM_REQUEST || f->status)
        return CUSTOM_BAD_LENGTH;
    for (size_t i = f->length; i < CUSTOM_PAYLOAD_BYTES; ++i)
        if (p[20 + i])
            return CUSTOM_BAD_LENGTH;
    memcpy(f->payload, p + 20, f->length);
    return CUSTOM_OK;
}
void custom_frame_encode(uint8_t p[CUSTOM_REPORT_BYTES], const custom_frame_t *f) {
    memset(p, 0, CUSTOM_REPORT_BYTES);
    p[0] = 'K';
    p[1] = 'C';
    p[2] = CUSTOM_PROTOCOL_VERSION;
    p[3] = f->type;
    custom_put_u16(p + 4, f->command);
    custom_put_u16(p + 6, f->sequence);
    custom_put_u32(p + 8, f->transaction);
    custom_put_u32(p + 12, f->offset);
    custom_put_u16(p + 16, f->length);
    custom_put_u16(p + 18, f->status);
    memcpy(p + 20, f->payload, f->length <= CUSTOM_PAYLOAD_BYTES ? f->length : 0);
    custom_put_u32(p + 60, custom_crc32(p, 60));
}
