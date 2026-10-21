#pragma once
#include "custom_control.h"
#include "esp_partition.h"

typedef struct {
    uint32_t magic, version, model_bytes, class_count;
    uint8_t model_sha[32], package_sha[32];
    char labels[CUSTOM_MAX_CLASSES][CUSTOM_LABEL_BYTES];
    uint32_t crc;
} custom_slot_meta_t;

typedef struct {
    uint32_t generation;
    int32_t slot;
    uint8_t package_sha[32];
    custom_binding_t bindings[CUSTOM_MAX_CLASSES];
    uint32_t crc;
} custom_saved_config_t;

void custom_state_init(const custom_saved_config_t *saved, const custom_slot_meta_t *meta,
                       int slot);
void custom_state_select(const custom_slot_meta_t *meta, int slot,
                         const custom_saved_config_t *saved);
/* Control task only; preserve mappings and cancel stale queued actions. */
void custom_state_set_actions_enabled(bool enabled);
uint16_t custom_state_set_binding(uint32_t boot, uint32_t epoch, uint32_t revision, unsigned index,
                                  const custom_binding_t *binding);
void custom_state_capture(custom_saved_config_t *saved);
void custom_state_mark_saved(uint32_t revision);
void custom_state_filter_init(const custom_kws_filter_t *saved);
void custom_state_filter_set(const custom_kws_filter_t *filter);
bool custom_filter_valid(const custom_kws_filter_t *filter);
uint32_t custom_boot_id(void);
unsigned custom_runtime_state(void);
esp_err_t custom_runtime_error(void);
int custom_active_slot(void);
bool custom_config_dirty(void);
void custom_state_meta(custom_slot_meta_t *meta);

esp_err_t custom_storage_init(custom_saved_config_t *saved, custom_slot_meta_t *meta, int *slot);
bool custom_storage_slot_info(unsigned slot, custom_slot_meta_t *meta);
uint16_t custom_storage_begin(void);
uint16_t custom_storage_object_begin(unsigned object, uint32_t size, const uint8_t sha[32]);
uint16_t custom_storage_data(uint32_t offset, const uint8_t *data, unsigned size);
uint16_t custom_storage_object_end(void);
uint16_t custom_storage_finish(void);
void custom_storage_abort(void);
unsigned custom_storage_target(void);
uint32_t custom_storage_received(void);
uint32_t custom_storage_total(void);
const custom_slot_meta_t *custom_storage_candidate(void);
esp_err_t custom_storage_save(custom_saved_config_t *saved);
bool custom_storage_filter_read(custom_kws_filter_t *filter);
esp_err_t custom_storage_filter_write(const custom_kws_filter_t *filter);
bool custom_storage_restore(unsigned slot, custom_saved_config_t *saved);
void custom_storage_embedded_meta(custom_slot_meta_t *meta);

esp_err_t custom_maintenance_enter(void);
void custom_maintenance_leave(void);
