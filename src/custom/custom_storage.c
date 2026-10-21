#include "custom_internal.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "psa/crypto.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SLOT_MAGIC 0x3153574BU
#define META_OFFSET CUSTOM_MODEL_MAX_BYTES
#define SECTOR 4096U
_Static_assert(sizeof(custom_slot_meta_t) <= SECTOR, "KWS metadata exceeds one erase sector");
extern const uint8_t kws_model_start[] asm("_binary_kws_model_espdl_start");
extern const uint8_t kws_model_end[] asm("_binary_kws_model_espdl_end");
static const esp_partition_t *s_slots[2];
static custom_slot_meta_t s_cached[2], s_candidate;
static bool s_valid[2], s_manifest_done, s_model_done, s_open;
static nvs_handle_t s_nvs;
typedef struct {
    uint32_t magic;
    uint32_t version;
    custom_kws_filter_t filter;
    uint32_t crc;
} saved_filter_t;
#define FILTER_MAGIC 0x31544C46U
static uint32_t s_generation, s_size, s_received, s_written;
static unsigned s_target, s_object;
static uint8_t s_sha[32], s_sector[SECTOR];
static char s_manifest[CUSTOM_MANIFEST_MAX_BYTES + 1];
static size_t s_manifest_size;

static bool digest(const void *p, size_t size, uint8_t sha[32]) {
    size_t out = 0;
    return psa_hash_compute(PSA_ALG_SHA_256, p, size, sha, 32, &out) == PSA_SUCCESS && out == 32;
}
static bool partition_digest(const esp_partition_t *part, uint32_t size, uint8_t sha[32],
                             bool package) {
    psa_hash_operation_t hash = PSA_HASH_OPERATION_INIT;
    if (psa_hash_setup(&hash, PSA_ALG_SHA_256) != PSA_SUCCESS)
        return false;
    esp_err_t err = ESP_OK;
    for (uint32_t off = 0; off < size; off += SECTOR) {
        unsigned n = size - off < SECTOR ? size - off : SECTOR;
        if ((err = esp_partition_read(part, off, s_sector, n)) != ESP_OK ||
            psa_hash_update(&hash, s_sector, n) != PSA_SUCCESS) {
            err = ESP_FAIL;
            break;
        }
        vTaskDelay(1);
    }
    if (err == ESP_OK && package &&
        psa_hash_update(&hash, (const uint8_t *)s_manifest, s_manifest_size) != PSA_SUCCESS)
        err = ESP_FAIL;
    size_t out = 0;
    if (err == ESP_OK && psa_hash_finish(&hash, sha, 32, &out) == PSA_SUCCESS && out == 32)
        return true;
    psa_hash_abort(&hash);
    return false;
}
void custom_storage_embedded_meta(custom_slot_meta_t *m) {
    memset(m, 0, sizeof(*m));
    m->magic = SLOT_MAGIC;
    m->version = 1;
    m->model_bytes = kws_model_end - kws_model_start;
    m->class_count = 8;
    const char *names[] = {"silence", "unknown", "yes", "no", "up", "down", "left", "right"};
    for (unsigned i = 0; i < 8; ++i)
        strcpy(m->labels[i], names[i]);
    digest(kws_model_start, m->model_bytes, m->model_sha);
    /* Factory identity is explicitly its model digest (no downloaded manifest). */
    memcpy(m->package_sha, m->model_sha, 32);
}
static bool read_config(const char *key, custom_saved_config_t *s) {
    size_t size = sizeof(*s);
    return s_nvs && nvs_get_blob(s_nvs, key, s, &size) == ESP_OK && size == sizeof(*s) &&
           s->crc == custom_crc32(s, offsetof(custom_saved_config_t, crc));
}
bool custom_storage_filter_read(custom_kws_filter_t *filter) {
    saved_filter_t record = {0};
    size_t size = sizeof(record);
    if (!s_nvs || nvs_get_blob(s_nvs, "kws_filter", &record, &size) != ESP_OK ||
        size != sizeof(record) || record.magic != FILTER_MAGIC || record.version != 1 ||
        record.crc != custom_crc32(&record, offsetof(saved_filter_t, crc)) ||
        !custom_filter_valid(&record.filter)) return false;
    *filter = record.filter;
    return true;
}
esp_err_t custom_storage_filter_write(const custom_kws_filter_t *filter) {
    if (!s_nvs) return ESP_ERR_INVALID_STATE;
    saved_filter_t record = {0};
    record.magic = FILTER_MAGIC;
    record.version = 1;
    record.filter = *filter;
    record.crc = custom_crc32(&record, offsetof(saved_filter_t, crc));
    esp_err_t err = nvs_set_blob(s_nvs, "kws_filter", &record, sizeof(record));
    return err == ESP_OK ? nvs_commit(s_nvs) : err;
}
bool custom_storage_restore(unsigned slot, custom_saved_config_t *s) {
    return read_config(slot == 0 ? "cfg_a" : slot == 1 ? "cfg_b" : "cfg_factory", s);
}
esp_err_t custom_storage_init(custom_saved_config_t *saved, custom_slot_meta_t *meta, int *slot) {
    psa_crypto_init();
    *slot = -1;
    memset(saved, 0, sizeof(*saved));
    custom_storage_embedded_meta(meta);
    for (unsigned i = 0; i < 2; ++i) {
        s_slots[i] = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY,
                                              i ? "kws_b" : "kws_a");
        custom_slot_meta_t *m = &s_cached[i];
        s_valid[i] = s_slots[i] && s_slots[i]->size >= 1024U * 1024U &&
                     esp_partition_read(s_slots[i], META_OFFSET, m, sizeof(*m)) == ESP_OK &&
                     m->magic == SLOT_MAGIC && m->version == 1 && m->model_bytes >= 32 &&
                     m->model_bytes <= CUSTOM_MODEL_MAX_BYTES && m->class_count >= 3 &&
                     m->class_count <= CUSTOM_MAX_CLASSES &&
                     m->crc == custom_crc32(m, offsetof(custom_slot_meta_t, crc));
        uint8_t sha[32];
        if (s_valid[i])
            s_valid[i] = partition_digest(s_slots[i], m->model_bytes, sha, false) &&
                         !memcmp(sha, m->model_sha, 32);
    }
    esp_err_t err = nvs_flash_init_partition("kws_cfg");
    /* Never automatically erase user configuration on an NVS error. */
    if (err != ESP_OK)
        return err;
    if ((err = nvs_open_from_partition("kws_cfg", "custom", NVS_READWRITE, &s_nvs)) != ESP_OK)
        return err;
    custom_saved_config_t records[2];
    bool found = false;
    for (unsigned i = 0; i < 2; ++i) {
        custom_saved_config_t *r = &records[i];
        if (!read_config(i ? "active1" : "active0", r))
            continue;
        if (r->generation > s_generation)
            s_generation = r->generation;
        if (r->slot < -1 || r->slot > 1)
            continue;
        const custom_slot_meta_t *m = r->slot < 0 ? meta : &s_cached[r->slot];
        if (r->slot >= 0 && !s_valid[r->slot])
            continue;
        if (memcmp(m->package_sha, r->package_sha, 32))
            continue;
        if (!found || r->generation > saved->generation) {
            *saved = *r;
            *slot = r->slot;
            found = true;
        }
    }
    if (*slot >= 0)
        *meta = s_cached[*slot];
    return ESP_OK;
}
bool custom_storage_slot_info(unsigned slot, custom_slot_meta_t *m) {
    if (slot > 1 || !s_valid[slot])
        return false;
    if (m)
        *m = s_cached[slot];
    return true;
}
uint16_t custom_storage_begin(void) {
    if (!s_nvs || !s_slots[0] || !s_slots[1] || s_slots[0]->size < 1024U * 1024U ||
        s_slots[1]->size < 1024U * 1024U)
        return CUSTOM_NOT_FOUND;
    s_target = custom_active_slot() == 0 ? 1 : 0;
    s_valid[s_target] = false;
    s_manifest_done = s_model_done = s_open = false;
    s_size = s_received = s_written = 0;
    memset(&s_candidate, 0, sizeof(s_candidate));
    /* Invalidate the old candidate before touching any model bytes. */
    return esp_partition_erase_range(s_slots[s_target], META_OFFSET, SECTOR) == ESP_OK
               ? CUSTOM_OK
               : CUSTOM_FLASH_ERROR;
}
uint16_t custom_storage_object_begin(unsigned object, uint32_t size, const uint8_t sha[32]) {
    if (s_open || (object != CUSTOM_OBJ_MANIFEST && object != CUSTOM_OBJ_MODEL))
        return CUSTOM_INVALID_STATE;
    if (!size || (object == CUSTOM_OBJ_MANIFEST ? size > CUSTOM_MANIFEST_MAX_BYTES
                                                : size < 32 || size > CUSTOM_MODEL_MAX_BYTES))
        return CUSTOM_OUT_OF_RANGE;
    if ((object == CUSTOM_OBJ_MANIFEST && (s_manifest_done || s_model_done)) ||
        (object == CUSTOM_OBJ_MODEL && (!s_manifest_done || s_model_done)))
        return CUSTOM_INVALID_STATE;
    if (object == CUSTOM_OBJ_MODEL) {
        uint32_t erase_size = (size + SECTOR - 1) & ~(SECTOR - 1);
        for (uint32_t off = 0; off < erase_size; off += SECTOR) {
            if (esp_partition_erase_range(s_slots[s_target], off, SECTOR) != ESP_OK)
                return CUSTOM_FLASH_ERROR;
            vTaskDelay(1);
        }
    }
    s_object = object;
    s_size = size;
    s_received = s_written = 0;
    memcpy(s_sha, sha, 32);
    s_open = true;
    return CUSTOM_OK;
}
uint16_t custom_storage_data(uint32_t offset, const uint8_t *data, unsigned size) {
    if (!s_open)
        return CUSTOM_INVALID_STATE;
    if (offset != s_received)
        return CUSTOM_OFFSET_MISMATCH;
    if (!size || size > CUSTOM_PAYLOAD_BYTES || s_received > s_size || size > s_size - s_received)
        return CUSTOM_OUT_OF_RANGE;
    if (s_object == CUSTOM_OBJ_MANIFEST)
        memcpy(s_manifest + offset, data, size);
    else {
        unsigned used = s_received - s_written;
        unsigned first = SECTOR - used < size ? SECTOR - used : size;
        memcpy(s_sector + used, data, first);
        if (used + first == SECTOR) {
            if (esp_partition_write(s_slots[s_target], s_written, s_sector, SECTOR) != ESP_OK) {
                s_open = false;
                return CUSTOM_FLASH_ERROR;
            }
            s_written += SECTOR;
            memcpy(s_sector, data + first, size - first);
        }
    }
    s_received += size;
    return CUSTOM_OK;
}
static uint16_t parse_manifest(void) {
    s_manifest[s_size] = 0;
    if (memchr(s_manifest, 0, s_size) || !custom_utf8_valid((const uint8_t *)s_manifest, s_size))
        return CUSTOM_MODEL_INCOMPATIBLE;
    const char *end = NULL;
    cJSON *root = cJSON_ParseWithLengthOpts(s_manifest, s_size + 1, &end, true);
    if (!root)
        return CUSTOM_MODEL_INCOMPATIBLE;
    cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "format_version");
    cJSON *profile = cJSON_GetObjectItemCaseSensitive(root, "frontend_profile");
    cJSON *classes = cJSON_GetObjectItemCaseSensitive(root, "classes");
    int count = cJSON_GetArraySize(classes);
    bool valid = cJSON_IsNumber(version) && version->valuedouble == 1 && cJSON_IsString(profile) &&
                 !strcmp(profile->valuestring, "mfcc40_16k_int16_state6_v1") &&
                 cJSON_IsArray(classes) && count >= 3 && count <= CUSTOM_MAX_CLASSES;
    for (int i = 0; valid && i < count; ++i) {
        cJSON *item = cJSON_GetArrayItem(classes, i);
        cJSON *index = cJSON_GetObjectItemCaseSensitive(item, "class_index");
        cJSON *text = cJSON_GetObjectItemCaseSensitive(item, "text");
        valid = cJSON_IsNumber(index) && index->valuedouble == i && cJSON_IsString(text) &&
                strlen(text->valuestring) > 0 && strlen(text->valuestring) < CUSTOM_LABEL_BYTES;
        if (valid && i < 2)
            valid = !strcmp(text->valuestring, i ? "unknown" : "silence");
        if (valid)
            strcpy(s_candidate.labels[i], text->valuestring);
    }
    if (valid) {
        s_candidate.class_count = count;
        s_manifest_size = s_size;
    }
    cJSON_Delete(root);
    return valid ? CUSTOM_OK : CUSTOM_MODEL_INCOMPATIBLE;
}
uint16_t custom_storage_object_end(void) {
    if (!s_open || s_received != s_size)
        return CUSTOM_INVALID_STATE;
    s_open = false;
    uint8_t sha[32];
    if (s_object == CUSTOM_OBJ_MANIFEST) {
        if (!digest(s_manifest, s_size, sha) || memcmp(sha, s_sha, 32))
            return CUSTOM_HASH_MISMATCH;
        uint16_t result = parse_manifest();
        s_manifest_done = result == CUSTOM_OK;
        return result;
    }
    if (s_received != s_written && esp_partition_write(s_slots[s_target], s_written, s_sector,
                                                       s_received - s_written) != ESP_OK)
        return CUSTOM_FLASH_ERROR;
    if (!partition_digest(s_slots[s_target], s_size, sha, false) || memcmp(sha, s_sha, 32))
        return CUSTOM_HASH_MISMATCH;
    uint8_t header[16];
    if (esp_partition_read(s_slots[s_target], 0, header, sizeof(header)) != ESP_OK)
        return CUSTOM_FLASH_ERROR;
    /* Only unencrypted, single-model EDL2 packages exported for the fixed ABI. */
    if (memcmp(header, "EDL2", 4) || custom_get_u32(header + 4) != 0 ||
        custom_get_u32(header + 8) > s_size - 16 || custom_get_u32(header + 8) < 16)
        return CUSTOM_MODEL_INCOMPATIBLE;
    s_candidate.model_bytes = s_size;
    memcpy(s_candidate.model_sha, sha, 32);
    s_model_done = true;
    return CUSTOM_OK;
}
uint16_t custom_storage_finish(void) {
    if (s_open || !s_manifest_done || !s_model_done)
        return CUSTOM_INVALID_STATE;
    if (!partition_digest(s_slots[s_target], s_candidate.model_bytes, s_candidate.package_sha,
                          true))
        return CUSTOM_FLASH_ERROR;
    s_candidate.magic = SLOT_MAGIC;
    s_candidate.version = 1;
    s_candidate.crc = custom_crc32(&s_candidate, offsetof(custom_slot_meta_t, crc));
    if (esp_partition_write(s_slots[s_target], META_OFFSET, &s_candidate, sizeof(s_candidate)) !=
        ESP_OK)
        return CUSTOM_FLASH_ERROR;
    s_cached[s_target] = s_candidate;
    s_valid[s_target] = true;
    return CUSTOM_OK;
}
void custom_storage_abort(void) {
    s_open = s_manifest_done = s_model_done = false;
}
unsigned custom_storage_target(void) {
    return s_target;
}
uint32_t custom_storage_received(void) {
    return s_received;
}
uint32_t custom_storage_total(void) {
    return s_size;
}
const custom_slot_meta_t *custom_storage_candidate(void) {
    return &s_candidate;
}
esp_err_t custom_storage_save(custom_saved_config_t *s) {
    if (!s_nvs)
        return ESP_ERR_INVALID_STATE;
    s->generation = ++s_generation;
    s->crc = custom_crc32(s, offsetof(custom_saved_config_t, crc));
    const char *cfg = s->slot < 0 ? "cfg_factory" : s->slot ? "cfg_b" : "cfg_a";
    esp_err_t err = nvs_set_blob(s_nvs, cfg, s, sizeof(*s));
    if (err == ESP_OK)
        err = nvs_commit(s_nvs);
    if (err == ESP_OK)
        err = nvs_set_blob(s_nvs, s->generation & 1 ? "active1" : "active0", s, sizeof(*s));
    if (err == ESP_OK)
        err = nvs_commit(s_nvs);
    return err;
}
