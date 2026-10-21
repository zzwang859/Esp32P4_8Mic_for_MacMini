#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CUSTOM_REPORT_BYTES 64U
#define CUSTOM_PAYLOAD_BYTES 40U
#define CUSTOM_PROTOCOL_VERSION 1U
#define CUSTOM_HID_INSTANCE 1U
#define CUSTOM_MAX_CLASSES 34U
#define CUSTOM_LABEL_BYTES 65U
#define CUSTOM_MODEL_MAX_BYTES (960U * 1024U)
#define CUSTOM_MANIFEST_MAX_BYTES 8192U
#define CUSTOM_COMMAND_DEPTH 16U

enum custom_command {
    CUSTOM_GET_INFO = 0x0001,
    CUSTOM_GET_CAPS = 0x0002,
    CUSTOM_GET_STATUS = 0x0003,
    CUSTOM_OPEN_SESSION = 0x0004,
    CUSTOM_READ_BEGIN = 0x0010,
    CUSTOM_READ_CHUNK = 0x0011,
    CUSTOM_READ_END = 0x0012,
    CUSTOM_SET_BINDING = 0x0020,
    CUSTOM_SAVE_CONFIG = 0x0021,
    CUSTOM_SUBSCRIBE = 0x0022,
    CUSTOM_SET_KWS_FILTER = 0x0023,
    CUSTOM_UPDATE_BEGIN = 0x0030,
    CUSTOM_OBJECT_BEGIN = 0x0031,
    CUSTOM_OBJECT_DATA = 0x0032,
    CUSTOM_OBJECT_END = 0x0033,
    CUSTOM_UPDATE_COMMIT = 0x0034,
    CUSTOM_UPDATE_ABORT = 0x0035,
    CUSTOM_ACTIVATE = 0x0036,
    CUSTOM_EVENT_KEYWORD = 0x8001,
    CUSTOM_EVENT_SAVE = 0x8002,
};
enum custom_status {
    CUSTOM_OK = 0,
    CUSTOM_ACCEPTED = 1,
    CUSTOM_BUSY = 2,
    CUSTOM_BAD_VERSION = 3,
    CUSTOM_BAD_LENGTH = 4,
    CUSTOM_BAD_CRC = 5,
    CUSTOM_INVALID_STATE = 6,
    CUSTOM_OUT_OF_RANGE = 7,
    CUSTOM_OFFSET_MISMATCH = 8,
    CUSTOM_HASH_MISMATCH = 9,
    CUSTOM_MODEL_INCOMPATIBLE = 10,
    CUSTOM_LOAD_FAILED = 11,
    CUSTOM_NO_SPACE = 12,
    CUSTOM_FLASH_ERROR = 13,
    CUSTOM_CONFLICT = 14,
    CUSTOM_NOT_FOUND = 15,
    CUSTOM_MODEL_CHANGED = 16,
    CUSTOM_QUEUE_FULL = 17,
    CUSTOM_BAD_COMMAND = 18,
};
enum { CUSTOM_REQUEST = 0, CUSTOM_RESPONSE = 1, CUSTOM_EVENT = 2 };
enum {
    CUSTOM_OBJ_INFO = 1,
    CUSTOM_OBJ_CAPS,
    CUSTOM_OBJ_STATUS,
    CUSTOM_OBJ_CLASSES,
    CUSTOM_OBJ_BINDINGS,
    CUSTOM_OBJ_STATS,
    CUSTOM_OBJ_MANIFEST = 0x100,
    CUSTOM_OBJ_MODEL = 0x101
};

/* Logical frame only. Never send a native C structure over USB. */
typedef struct {
    uint8_t type;
    uint16_t command, sequence;
    uint32_t transaction, offset;
    uint16_t length, status;
    uint8_t payload[CUSTOM_PAYLOAD_BYTES];
} custom_frame_t;

uint16_t custom_get_u16(const uint8_t *p);
uint32_t custom_get_u32(const uint8_t *p);
void custom_put_u16(uint8_t *p, uint16_t v);
void custom_put_u32(uint8_t *p, uint32_t v);
uint32_t custom_crc32(const void *data, size_t size);
uint16_t custom_frame_decode(custom_frame_t *frame, const uint8_t *wire, size_t size);
void custom_frame_encode(uint8_t wire[CUSTOM_REPORT_BYTES], const custom_frame_t *frame);
bool custom_utf8_valid(const uint8_t *data, size_t size);
