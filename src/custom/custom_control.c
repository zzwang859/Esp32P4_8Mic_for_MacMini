#include "custom_internal.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "audio_pipeline.h"
#include "audio_diagnostics.h"
#include "kws_stream.h"
#include "usb_audio.h"
#include "usb_hid_keyboard.h"
#include "tusb.h"

#define CUSTOM_CONTROL_TASK_PRIORITY 1U

typedef struct {
    uint32_t connection;
    uint8_t console_command;
    uint8_t wire[CUSTOM_REPORT_BYTES];
} message_t;
typedef struct {
    bool valid;
    uint8_t request[64], response[64];
} replay_t;
static QueueHandle_t s_commands, s_replies, s_events;
static uint32_t s_connection, s_subscribed, s_event_sequence, s_event_dropped, s_queue_dropped;
static uint32_t s_transaction, s_update_connection, s_snapshot_id, s_next_id;
static int64_t s_last_update;
static unsigned s_update_state;
static uint16_t s_update_error;
static bool s_save_pending;
static bool s_hid_logs_enabled;
static custom_saved_config_t s_pending_saved;
static uint32_t s_pending_revision, s_pending_epoch;
static char *s_snapshot;
static uint32_t s_snapshot_size;
static replay_t s_replay[16];
static unsigned s_replay_pos;
static uint8_t s_session[16];
static bool s_session_valid;
static custom_saved_config_t s_saved, s_old_saved;
static custom_slot_meta_t s_meta, s_old_meta;

static uint32_t new_id(void) {
    if (!++s_next_id)
        ++s_next_id;
    return s_next_id;
}
static const char *hid_command_name(uint16_t command) {
    switch (command) {
    case CUSTOM_GET_INFO: return "GET_INFO";
    case CUSTOM_GET_CAPS: return "GET_CAPS";
    case CUSTOM_GET_STATUS: return "GET_STATUS";
    case CUSTOM_OPEN_SESSION: return "OPEN_SESSION";
    case CUSTOM_READ_BEGIN: return "READ_BEGIN";
    case CUSTOM_READ_CHUNK: return "READ_CHUNK";
    case CUSTOM_READ_END: return "READ_END";
    case CUSTOM_SET_BINDING: return "SET_BINDING";
    case CUSTOM_SAVE_CONFIG: return "SAVE_CONFIG";
    case CUSTOM_SUBSCRIBE: return "SUBSCRIBE";
    case CUSTOM_SET_KWS_FILTER: return "SET_KWS_FILTER";
    case CUSTOM_UPDATE_BEGIN: return "UPDATE_BEGIN";
    case CUSTOM_OBJECT_BEGIN: return "OBJECT_BEGIN";
    case CUSTOM_OBJECT_DATA: return "OBJECT_DATA";
    case CUSTOM_OBJECT_END: return "OBJECT_END";
    case CUSTOM_UPDATE_COMMIT: return "UPDATE_COMMIT";
    case CUSTOM_UPDATE_ABORT: return "UPDATE_ABORT";
    case CUSTOM_ACTIVATE: return "ACTIVATE";
    default: return "UNKNOWN";
    }
}
static const char *hid_status_name(uint16_t status) {
    static const char *const names[] = {
        "OK", "ACCEPTED", "BUSY", "BAD_VERSION", "BAD_LENGTH", "BAD_CRC",
        "INVALID_STATE", "OUT_OF_RANGE", "OFFSET_MISMATCH", "HASH_MISMATCH",
        "MODEL_INCOMPATIBLE", "LOAD_FAILED", "NO_SPACE", "FLASH_ERROR",
        "CONFLICT", "NOT_FOUND", "MODEL_CHANGED", "QUEUE_FULL", "BAD_COMMAND"};
    return status < sizeof(names) / sizeof(names[0]) ? names[status] : "UNKNOWN";
}
/* Opt-in control-task feedback, independent of l/L and USB streaming quiet mode.
 * Never called from USB callbacks or inference. Successful bulk frames stay
 * silent; object boundaries and every error remain visible. */
static void log_hid_result(const custom_frame_t *in, const custom_frame_t *out, bool replay) {
    if (!s_hid_logs_enabled)
        return;
    if (out->status == CUSTOM_OK && (in->command == CUSTOM_READ_CHUNK ||
        in->command == CUSTOM_READ_END || in->command == CUSTOM_OBJECT_DATA))
        return;
    char detail[240] = "";
    const uint8_t *p = in->payload;
    if (in->command == CUSTOM_SET_BINDING && in->length == 29) {
        snprintf(detail, sizeof(detail),
                 " class=%u flags=0x%02X rgb=%u,%u,%u modifier=0x%02X"
                 " keys=%02X,%02X,%02X,%02X,%02X,%02X led_ms=%u hold_ms=%u revision=%lu",
                 (unsigned)custom_get_u16(p + 12), (unsigned)p[14],
                 (unsigned)p[15], (unsigned)p[16], (unsigned)p[17], (unsigned)p[18],
                 (unsigned)p[19], (unsigned)p[20], (unsigned)p[21], (unsigned)p[22],
                 (unsigned)p[23], (unsigned)p[24], (unsigned)custom_get_u16(p + 25),
                 (unsigned)custom_get_u16(p + 27), (unsigned long)custom_binding_revision());
    } else if (in->command == CUSTOM_SAVE_CONFIG && in->length == 12) {
        unsigned streams = usb_audio_stream_mask();
        snprintf(detail, sizeof(detail), " revision=%lu save_priority=%u stream_mask=0x%02X",
                 (unsigned long)custom_get_u32(p + 8), CUSTOM_CONTROL_TASK_PRIORITY, streams);
    } else if (in->command == CUSTOM_READ_BEGIN && in->length == 2) {
        snprintf(detail, sizeof(detail), " object=0x%04X", (unsigned)custom_get_u16(p));
    } else if (in->command == CUSTOM_OBJECT_BEGIN && in->length == 38) {
        snprintf(detail, sizeof(detail), " object=0x%04X bytes=%lu",
                 (unsigned)custom_get_u16(p), (unsigned long)custom_get_u32(p + 2));
    } else if (in->command == CUSTOM_ACTIVATE && in->length == 1) {
        snprintf(detail, sizeof(detail), " slot=%s(%u)",
                 p[0] == 255 ? "embedded" : p[0] == 0 ? "A" : p[0] == 1 ? "B" : "unknown",
                 (unsigned)p[0]);
    } else if (in->command == CUSTOM_SUBSCRIBE && in->length == 1) {
        snprintf(detail, sizeof(detail), " enabled=%u", (unsigned)p[0]);
    } else if (in->command == CUSTOM_UPDATE_BEGIN && out->status == CUSTOM_OK && out->length == 1) {
        snprintf(detail, sizeof(detail), " target_slot=%u", (unsigned)out->payload[0]);
    }
    char line[400];
    const int length = snprintf(line, sizeof(line),
        "HID %s cmd=0x%04X seq=%u status=%s(%u) replay=%u transaction=%lu offset=%lu%s\n",
        hid_command_name(in->command), (unsigned)in->command, (unsigned)in->sequence,
        hid_status_name(out->status), (unsigned)out->status, (unsigned)replay,
        (unsigned long)out->transaction, (unsigned long)in->offset, detail);
    if (length > 0 && (size_t)length < sizeof(line))
        audio_diagnostics_write_kws_line(line, (size_t)length);
}
static void log_hid_save_result(uint16_t status) {
    if (!s_hid_logs_enabled)
        return;
    char line[128];
    const int length = snprintf(line, sizeof(line),
        "HID SAVE_CONFIG complete status=%s(%u) model_epoch=%lu revision=%lu\n",
        hid_status_name(status), (unsigned)status, (unsigned long)s_pending_epoch,
        (unsigned long)s_pending_revision);
    if (length > 0 && (size_t)length < sizeof(line))
        audio_diagnostics_write_kws_line(line, (size_t)length);
}
static void hex_sha(char text[65], const uint8_t sha[32]) {
    static const char digits[] = "0123456789abcdef";
    for (unsigned i = 0; i < 32; ++i) {
        text[2 * i] = digits[sha[i] >> 4];
        text[2 * i + 1] = digits[sha[i] & 15];
    }
    text[64] = 0;
}
static void number(cJSON *j, const char *name, double value) {
    cJSON_AddNumberToObject(j, name, value);
}
static void string(cJSON *j, const char *name, const char *value) {
    cJSON_AddStringToObject(j, name, value);
}
static cJSON *binding_json(unsigned i) {
    custom_binding_t b;
    if (!custom_binding_get(i, &b, NULL))
        return NULL;
    cJSON *j = cJSON_CreateObject(), *led = cJSON_AddObjectToObject(j, "led"),
          *key = cJSON_AddObjectToObject(j, "keyboard");
    number(j, "class_index", i);
    cJSON_AddBoolToObject(j, "actions_enabled", b.actions_enabled);
    cJSON_AddBoolToObject(led, "enabled", b.led_enabled);
    string(led, "effect", "flash");
    int rgb[] = {b.r, b.g, b.b};
    cJSON_AddItemToObject(led, "rgb", cJSON_CreateIntArray(rgb, 3));
    number(led, "duration_ms", b.led_duration_ms);
    cJSON_AddBoolToObject(key, "enabled", b.keyboard_enabled);
    number(key, "modifier", b.modifier);
    int keys[6];
    for (unsigned k = 0; k < 6; ++k)
        keys[k] = b.keycodes[k];
    cJSON_AddItemToObject(key, "keycodes", cJSON_CreateIntArray(keys, 6));
    number(key, "hold_ms", b.key_hold_ms);
    return j;
}
static char *make_snapshot(unsigned object) {
    cJSON *j = cJSON_CreateObject();
    if (!j)
        return NULL;
    number(j, "boot_id", custom_boot_id());
    number(j, "model_epoch", custom_model_epoch());
    number(j, "config_revision", custom_binding_revision());
    if (object == CUSTOM_OBJ_INFO) {
        const esp_app_desc_t *app = esp_app_get_description();
        uint8_t mac[6];
        char id[20];
        esp_efuse_mac_get_default(mac);
        snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4],
                 mac[5]);
        string(j, "device_id", id);
        string(j, "firmware_version", app->version);
        string(j, "project", app->project_name);
        number(j, "protocol_version", CUSTOM_PROTOCOL_VERSION);
        number(j, "vid", 0x303A);
        number(j, "pid", 0x8003);
    } else if (object == CUSTOM_OBJ_CAPS) {
        number(j, "max_class_count", CUSTOM_MAX_CLASSES);
        number(j, "max_model_bytes", CUSTOM_MODEL_MAX_BYTES);
        number(j, "max_manifest_bytes", CUSTOM_MANIFEST_MAX_BYTES);
        number(j, "max_label_utf8_bytes", CUSTOM_LABEL_BYTES - 1);
        number(j, "command_queue_capacity", CUSTOM_COMMAND_DEPTH);
        number(j, "max_chord_keys", 6);
        number(j, "model_slot_count", 2);
        number(j, "payload_bytes", CUSTOM_PAYLOAD_BYTES);
        string(j, "frontend_profile", "mfcc40_16k_int16_state6_v1");
#if CONFIG_WS2812_LED_RING_ENABLE
        cJSON_AddBoolToObject(j, "supports_led", true);
#else
        cJSON_AddBoolToObject(j, "supports_led", false);
#endif
        cJSON_AddBoolToObject(j, "supports_runtime_binding", true);
        cJSON_AddBoolToObject(j, "supports_runtime_kws_filter", true);
    } else if (object == CUSTOM_OBJ_STATUS) {
        custom_state_meta(&s_meta);
        char sha[65];
        hex_sha(sha, s_meta.package_sha);
        cJSON *model = cJSON_AddObjectToObject(j, "active_model");
        string(model, "source",
               custom_active_slot() < 0 ? "embedded"
               : custom_active_slot()   ? "slot_b"
                                        : "slot_a");
        string(model, "package_sha256", sha);
        number(model, "model_bytes", s_meta.model_bytes);
        number(model, "class_count", s_meta.class_count);
        number(model, "keyword_count", s_meta.class_count - 2);
        unsigned enabled = 0;
        for (unsigned i = 2; i < s_meta.class_count; ++i) {
            custom_binding_t b;
            custom_binding_get(i, &b, NULL);
            enabled += b.actions_enabled;
        }
        number(model, "enabled_keyword_count", enabled);
        cJSON *runtime = cJSON_AddObjectToObject(j, "runtime");
        const char *states[] = {"stopped", "loading", "warming_up", "ready", "error"};
        unsigned state = custom_runtime_state();
        string(runtime, "state", state < 5 ? states[state] : "error");
        number(runtime, "last_error", custom_runtime_error());
        cJSON_AddBoolToObject(runtime, "config_dirty", custom_config_dirty());
        cJSON_AddBoolToObject(runtime, "save_pending", s_save_pending);
        custom_kws_filter_t filter;
        uint32_t filter_revision;
        custom_kws_filter_get(&filter, &filter_revision);
        cJSON *filter_json = cJSON_AddObjectToObject(j, "kws_filter");
        number(filter_json, "revision", filter_revision);
        number(filter_json, "keyword_threshold_percent", filter.keyword_threshold_percent);
        number(filter_json, "cooldown_ms", filter.cooldown_ms);
        number(filter_json, "stable_frames", filter.stable_frames);
        cJSON_AddBoolToObject(filter_json, "need_vad_checked", filter.need_vad_checked);
        cJSON *audio = cJSON_AddObjectToObject(j, "audio");
        unsigned streams = usb_audio_stream_mask();
        number(audio, "stream_mask", streams);
        string(audio, "capture_output_mode",
               custom_maintenance_active() ? "maintenance_silence" : "normal");
        string(audio, "processing_state", custom_maintenance_active() ? "paused" : "running");
        cJSON *update = cJSON_AddObjectToObject(j, "model_update");
        number(update, "state", s_update_state);
        number(update, "transaction", s_transaction);
        number(update, "target_slot", custom_storage_target());
        number(update, "received_bytes", custom_storage_received());
        number(update, "total_bytes", custom_storage_total());
        number(update, "last_error", s_update_error);
        cJSON *slots = cJSON_AddArrayToObject(j, "slots");
        for (unsigned i = 0; i < 2; ++i) {
            cJSON *slot = cJSON_CreateObject();
            custom_slot_meta_t meta;
            bool valid = custom_storage_slot_info(i, &meta);
            number(slot, "slot", i);
            string(slot, "state",
                   s_transaction && i == custom_storage_target() ? "receiving"
                   : valid                                       ? "valid"
                                                                 : "empty_or_invalid");
            cJSON_AddBoolToObject(slot, "selected", custom_active_slot() == (int)i);
            if (valid) {
                hex_sha(sha, meta.package_sha);
                string(slot, "package_sha256", sha);
                number(slot, "class_count", meta.class_count);
            }
            cJSON_AddItemToArray(slots, slot);
        }
        number(j, "queued_commands", uxQueueMessagesWaiting(s_commands));
        number(j, "dropped_commands", s_queue_dropped);
        number(j, "dropped_events", s_event_dropped);
    } else if (object == CUSTOM_OBJ_CLASSES || object == CUSTOM_OBJ_BINDINGS) {
        custom_state_meta(&s_meta);
        char sha[65];
        hex_sha(sha, s_meta.package_sha);
        string(j, "package_sha256", sha);
        number(j, "class_count", s_meta.class_count);
        cJSON *items =
            cJSON_AddArrayToObject(j, object == CUSTOM_OBJ_CLASSES ? "classes" : "bindings");
        for (unsigned i = object == CUSTOM_OBJ_CLASSES ? 0 : 2; i < s_meta.class_count; ++i) {
            cJSON *item = object == CUSTOM_OBJ_CLASSES ? cJSON_CreateObject() : binding_json(i);
            if (object == CUSTOM_OBJ_CLASSES) {
                number(item, "class_index", i);
                string(item, "text", s_meta.labels[i]);
            }
            cJSON_AddItemToArray(items, item);
        }
    } else if (object == CUSTOM_OBJ_STATS) {
        kws_stream_stats_t stats;
        kws_stream_get_stats(&stats);
        number(j, "inference_chunks", stats.chunks);
        number(j, "inference_us", stats.process_us);
        number(j, "max_inference_us", stats.max_us);
        number(j, "dropped_samples", stats.dropped_samples);
        number(j, "resets", stats.resets);
        number(j, "errors", stats.errors);
        number(j, "dropped_events", s_event_dropped);
        number(j, "dropped_commands", s_queue_dropped);
    } else {
        cJSON_Delete(j);
        return NULL;
    }
    if (object == CUSTOM_OBJ_STATUS || object == CUSTOM_OBJ_STATS) {
        usb_audio_capture_stats_t usb;
        usb_audio_get_capture_stats(&usb);
        cJSON *u=cJSON_AddObjectToObject(j,"usb");
        number(u,"raw_null_packets",usb.raw_endpoint_null_packets);
        number(u,"capture2_null_packets",usb.mono_endpoint_null_packets);
        number(u,"capture2_tx_packets",usb.mono_tx_packets);
        number(u,"capture2_zero_packets",usb.mono_tx_zero_packets);
        number(u,"capture2_failed_packets",usb.mono_tx_zero_failed);
        number(u,"capture2_unexpected_zero_packets",usb.mono_tx_zero_unexpected);
        number(u,"capture2_unexpected_sizes",usb.mono_tx_unexpected_sizes);
        number(u,"endpoint_backpressure",usb.endpoint_fifo_backpressure);
        number(u,"endpoint_short_writes",usb.endpoint_write_shorts);
        number(u,"playback_underflows",usb.playback_underflows);
        number(u,"playback_dropped_bytes",usb.playback_dropped_bytes);
        number(u,"playback_i2s_errors",usb.playback_i2s_errors);
    }
    char *result = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    return result;
}
static void snapshot_begin(custom_frame_t *out, unsigned object) {
    if (object < CUSTOM_OBJ_INFO || object > CUSTOM_OBJ_STATS) {
        out->status = CUSTOM_NOT_FOUND;
        return;
    }
    char *snapshot = make_snapshot(object);
    if (!snapshot) {
        out->status = CUSTOM_NO_SPACE;
        return;
    }
    cJSON_free(s_snapshot);
    s_snapshot = snapshot;
    s_snapshot_size = strlen(snapshot);
    s_snapshot_id = new_id();
    out->transaction = s_snapshot_id;
    out->length = 16;
    custom_put_u32(out->payload, s_snapshot_size);
    custom_put_u32(out->payload + 4, custom_boot_id());
    custom_put_u32(out->payload + 8, custom_model_epoch());
    custom_put_u32(out->payload + 12, custom_binding_revision());
}
static void queue_wire(QueueHandle_t q, const uint8_t wire[64], uint32_t connection,
                       TickType_t wait) {
    message_t msg = {.connection = connection};
    memcpy(msg.wire, wire, 64);
    if (xQueueSend(q, &msg, wait) != pdTRUE && q == s_events)
        __atomic_fetch_add(&s_event_dropped, 1, __ATOMIC_RELAXED);
}
static void save_event(uint16_t status) {
    custom_frame_t out = {
        .type = CUSTOM_EVENT, .command = CUSTOM_EVENT_SAVE, .status = status, .length = 8};
    custom_put_u32(out.payload, s_pending_epoch);
    custom_put_u32(out.payload + 4, s_pending_revision);
    uint8_t wire[64];
    custom_frame_encode(wire, &out);
    queue_wire(s_events, wire, s_connection, 0);
    log_hid_save_result(status);
}
static esp_err_t save_current(void) {
    uint32_t revision = custom_binding_revision();
    custom_state_capture(&s_saved);
    esp_err_t err = custom_storage_save(&s_saved);
    if (err == ESP_OK)
        custom_state_mark_saved(revision);
    return err;
}
static void maintenance_end(void) {
    audio_pipeline_maintenance_reset();
    kws_stream_maintenance_reset();
    custom_storage_abort();
    s_transaction = 0;
    custom_maintenance_leave();
}
static uint16_t activate(unsigned slot) {
    int target = slot == 255 ? -1 : (int)slot;
    if (slot == 255)
        custom_storage_embedded_meta(&s_meta);
    else if (slot > 1 || !custom_storage_slot_info(slot, &s_meta))
        return CUSTOM_NOT_FOUND;
    if (custom_active_slot() == target)
        return CUSTOM_INVALID_STATE;
    esp_err_t err = custom_maintenance_enter();
    if (err != ESP_OK) {
        custom_maintenance_leave();
        return CUSTOM_BUSY;
    }
    if (usb_hid_keyboard_barrier(2500) != ESP_OK) {
        maintenance_end();
        return CUSTOM_BUSY;
    }
    custom_state_meta(&s_old_meta);
    custom_state_capture(&s_old_saved);
    int old_slot = custom_active_slot();
    bool restore = custom_storage_restore(slot, &s_saved) &&
                   !memcmp(s_saved.package_sha, s_meta.package_sha, 32);
    custom_state_select(&s_meta, target, restore ? &s_saved : NULL);
    err = kws_stream_reload();
    if (err == ESP_OK)
        err = save_current();
    if (err != ESP_OK) {
        custom_state_select(&s_old_meta, old_slot, &s_old_saved);
        esp_err_t rollback = kws_stream_reload();
        if (rollback != ESP_OK)
            custom_set_runtime_state(4, rollback);
    }
    maintenance_end();
    return err == ESP_OK ? CUSTOM_OK : CUSTOM_LOAD_FAILED;
}
static uint16_t execute(const custom_frame_t *in, custom_frame_t *out) {
    const uint8_t *p = in->payload;
    switch (in->command) {
    case CUSTOM_GET_INFO:
    case CUSTOM_GET_CAPS:
    case CUSTOM_GET_STATUS:
        if (in->length || in->transaction)
            return CUSTOM_BAD_LENGTH;
        snapshot_begin(out, in->command);
        return out->status;
    case CUSTOM_READ_BEGIN:
        if (in->length != 2 || in->transaction)
            return CUSTOM_BAD_LENGTH;
        snapshot_begin(out, custom_get_u16(p));
        return out->status;
    case CUSTOM_READ_CHUNK:
        if (in->length)
            return CUSTOM_BAD_LENGTH;
        if (!s_snapshot || in->transaction != s_snapshot_id)
            return CUSTOM_NOT_FOUND;
        if (in->offset > s_snapshot_size)
            return CUSTOM_OUT_OF_RANGE;
        out->length = s_snapshot_size - in->offset > 40 ? 40 : s_snapshot_size - in->offset;
        memcpy(out->payload, s_snapshot + in->offset, out->length);
        return CUSTOM_OK;
    case CUSTOM_READ_END:
        if (in->length)
            return CUSTOM_BAD_LENGTH;
        if (!s_snapshot || in->transaction != s_snapshot_id)
            return CUSTOM_NOT_FOUND;
        cJSON_free(s_snapshot);
        s_snapshot = NULL;
        s_snapshot_id = 0;
        return CUSTOM_OK;
    case CUSTOM_SET_BINDING: {
        if (in->length != 29 || in->transaction)
            return CUSTOM_BAD_LENGTH;
        if (s_transaction)
            return CUSTOM_BUSY;
        if (p[14] & ~7U)
            return CUSTOM_OUT_OF_RANGE;
        custom_binding_t b = {.actions_enabled = (p[14] & 1) != 0,
                              .led_enabled = (p[14] & 2) != 0,
                              .keyboard_enabled = (p[14] & 4) != 0,
                              .r = p[15],
                              .g = p[16],
                              .b = p[17],
                              .modifier = p[18],
                              .led_duration_ms = custom_get_u16(p + 25),
                              .key_hold_ms = custom_get_u16(p + 27)};
        memcpy(b.keycodes, p + 19, 6);
        uint16_t result =
            custom_state_set_binding(custom_get_u32(p), custom_get_u32(p + 4),
                                     custom_get_u32(p + 8), custom_get_u16(p + 12), &b);
        if (result != CUSTOM_OK)
            return result;
        /* Old tagged presses are canceled; a submitted press is always released. */
        if (usb_hid_keyboard_barrier(2500) != ESP_OK)
            return CUSTOM_BUSY;
        out->length = 4;
        custom_put_u32(out->payload, custom_binding_revision());
        return CUSTOM_OK;
    }
    case CUSTOM_SUBSCRIBE:
        if (in->length != 1 || in->transaction || p[0] > 1)
            return CUSTOM_BAD_LENGTH;
        __atomic_store_n(&s_subscribed, p[0], __ATOMIC_RELEASE);
        return CUSTOM_OK;
    case CUSTOM_SET_KWS_FILTER: {
        if (in->length != 17 || in->transaction || in->offset)
            return CUSTOM_BAD_LENGTH;
        if (custom_get_u32(p) != custom_boot_id())
            return CUSTOM_MODEL_CHANGED;
        custom_kws_filter_t current, filter = {
            .keyword_threshold_percent = custom_get_u16(p + 8),
            .cooldown_ms = custom_get_u32(p + 10),
            .stable_frames = custom_get_u16(p + 14),
            .need_vad_checked = p[16],
        };
        uint32_t revision;
        custom_kws_filter_get(&current, &revision);
        if (custom_get_u32(p + 4) != revision) return CUSTOM_CONFLICT;
        if (!custom_filter_valid(&filter)) return CUSTOM_OUT_OF_RANGE;
        if (s_transaction) return CUSTOM_BUSY;
        if (current.keyword_threshold_percent != filter.keyword_threshold_percent ||
            current.cooldown_ms != filter.cooldown_ms ||
            current.stable_frames != filter.stable_frames ||
            current.need_vad_checked != filter.need_vad_checked) {
            if (custom_storage_filter_write(&filter) != ESP_OK) return CUSTOM_FLASH_ERROR;
            custom_state_filter_set(&filter);
            ++revision;
        }
        out->length = 4;
        custom_put_u32(out->payload, revision);
        return CUSTOM_OK;
    }
    case CUSTOM_SAVE_CONFIG:
        if (in->length != 12 || in->transaction)
            return CUSTOM_BAD_LENGTH;
        if (custom_get_u32(p) != custom_boot_id() || custom_get_u32(p + 4) != custom_model_epoch())
            return CUSTOM_MODEL_CHANGED;
        if (custom_get_u32(p + 8) != custom_binding_revision())
            return CUSTOM_CONFLICT;
        if (s_transaction || s_save_pending)
            return CUSTOM_BUSY;
        custom_state_capture(&s_pending_saved);
        s_pending_revision = custom_binding_revision();
        s_pending_epoch = custom_model_epoch();
        s_save_pending = true;
        return CUSTOM_ACCEPTED;
    case CUSTOM_UPDATE_BEGIN: {
        if (in->length || in->transaction)
            return CUSTOM_BAD_LENGTH;
        if (s_transaction || s_save_pending)
            return CUSTOM_BUSY;
        s_update_connection = s_connection;
        esp_err_t err = custom_maintenance_enter();
        if (err != ESP_OK) {
            custom_maintenance_leave();
            return CUSTOM_BUSY;
        }
        if (usb_hid_keyboard_barrier(2500) != ESP_OK) {
            maintenance_end();
            return CUSTOM_BUSY;
        }
        uint16_t result = custom_storage_begin();
        if (result != CUSTOM_OK) {
            maintenance_end();
            return result;
        }
        s_transaction = new_id();
        s_last_update = esp_timer_get_time();
        s_update_state = 1;
        s_update_error = 0;
        out->transaction = s_transaction;
        out->length = 1;
        out->payload[0] = custom_storage_target();
        return CUSTOM_OK;
    }
    case CUSTOM_ACTIVATE:
        if (in->length != 1 || in->transaction)
            return CUSTOM_BAD_LENGTH;
        if (s_transaction || s_save_pending)
            return CUSTOM_BUSY;
        return activate(p[0]);
    case CUSTOM_OBJECT_BEGIN:
    case CUSTOM_OBJECT_DATA:
    case CUSTOM_OBJECT_END:
    case CUSTOM_UPDATE_COMMIT:
    case CUSTOM_UPDATE_ABORT:
        if (!s_transaction || in->transaction != s_transaction)
            return CUSTOM_INVALID_STATE;
        s_last_update = esp_timer_get_time();
        if (in->command == CUSTOM_UPDATE_ABORT) {
            if (in->length)
                return CUSTOM_BAD_LENGTH;
            maintenance_end();
            s_update_state = 0;
            return CUSTOM_OK;
        }
        if (in->command == CUSTOM_OBJECT_BEGIN) {
            if (in->length != 38)
                return CUSTOM_BAD_LENGTH;
            return custom_storage_object_begin(custom_get_u16(p), custom_get_u32(p + 2), p + 6);
        }
        if (in->command == CUSTOM_OBJECT_DATA) {
            uint16_t result = custom_storage_data(in->offset, p, in->length);
            out->length = 4;
            custom_put_u32(out->payload, custom_storage_received());
            return result;
        }
        if (in->length)
            return CUSTOM_BAD_LENGTH;
        if (in->command == CUSTOM_OBJECT_END)
            return custom_storage_object_end();
        {
            uint16_t result = custom_storage_finish();
            s_update_error = result;
            s_update_state = result == CUSTOM_OK ? 2 : 3;
            maintenance_end();
            return result;
        }
    default:
        return CUSTOM_BAD_COMMAND;
    }
}
static void process(const message_t *msg) {
    if (msg->connection != s_connection)
        return;
    custom_frame_t in, out = {0};
    uint16_t status = custom_frame_decode(&in, msg->wire, 64);
    out.type = CUSTOM_RESPONSE;
    out.command = in.command;
    out.sequence = in.sequence;
    out.transaction = in.transaction;
    out.offset = in.offset;
    if (status == CUSTOM_OK && in.command == CUSTOM_OPEN_SESSION) {
        /* Closing a host HID handle does not disconnect USB. A new session
         * must explicitly retire the previous process's sequence history. */
        if (in.length != sizeof(s_session) || in.transaction || in.offset)
            status = CUSTOM_BAD_LENGTH;
        else if (s_transaction && (!s_session_valid ||
                                   memcmp(s_session, in.payload, sizeof(s_session))))
            status = CUSTOM_BUSY;
        else {
            if (!s_session_valid || memcmp(s_session, in.payload, sizeof(s_session))) {
                memset(s_replay, 0, sizeof(s_replay));
                s_replay_pos = 0;
                cJSON_free(s_snapshot);
                s_snapshot = NULL;
                s_snapshot_id = 0;
                __atomic_store_n(&s_subscribed, 0, __ATOMIC_RELEASE);
                xQueueReset(s_events);
                memcpy(s_session, in.payload, sizeof(s_session));
                s_session_valid = true;
            }
            out.length = sizeof(s_session);
            memcpy(out.payload, s_session, sizeof(s_session));
        }
        out.status = status;
        uint8_t wire[64];
        custom_frame_encode(wire, &out);
        queue_wire(s_replies, wire, msg->connection, pdMS_TO_TICKS(100));
        log_hid_result(&in, &out, false);
        return;
    }
    if (status == CUSTOM_OK) {
        for (unsigned i = 0; i < 16; ++i)
            if (s_replay[i].valid && custom_get_u16(s_replay[i].request + 6) == in.sequence) {
                if (!memcmp(s_replay[i].request, msg->wire, 64)) {
                    queue_wire(s_replies, s_replay[i].response, msg->connection,
                               pdMS_TO_TICKS(100));
                    /* Replay is not a second execution or Flash write. */
                    out.status = custom_get_u16(s_replay[i].response + 18);
                    out.transaction = custom_get_u32(s_replay[i].response + 8);
                    out.length = custom_get_u16(s_replay[i].response + 16);
                    memcpy(out.payload, s_replay[i].response + 20, out.length);
                    log_hid_result(&in, &out, true);
                    return;
                }
                status = CUSTOM_CONFLICT;
                break;
            }
        if (status == CUSTOM_OK)
            status = execute(&in, &out);
        if (s_transaction && in.transaction == s_transaction)
            s_update_error = status;
    }
    out.status = status;
    uint8_t wire[64];
    custom_frame_encode(wire, &out);
    if (status != CUSTOM_BAD_CRC && status != CUSTOM_BAD_VERSION && status != CUSTOM_CONFLICT) {
        replay_t *r = &s_replay[s_replay_pos++ % 16];
        r->valid = true;
        memcpy(r->request, msg->wire, 64);
        memcpy(r->response, wire, 64);
    }
    queue_wire(s_replies, wire, msg->connection, pdMS_TO_TICKS(100));
    log_hid_result(&in, &out, false);
}
static void process_console(uint8_t command) {
    const char *line;
    if (command == 'u' || command == 'U') {
        s_hid_logs_enabled = !s_hid_logs_enabled;
        line = s_hid_logs_enabled ? "HID logs ON\n" : "HID logs OFF\n";
    } else if (s_transaction) {
        line = "KWS actions BUSY (model update)\n";
    } else {
        bool enabled = command == 'a' || command == 'A';
        custom_state_set_actions_enabled(enabled);
        if (usb_hid_keyboard_barrier(2500) != ESP_OK)
            line = "KWS actions updated; keyboard release pending\n";
        else
            line = enabled ? "KWS actions ON (LED; keyboard where mapped)\n"
                           : "KWS actions OFF\n";
    }
    audio_diagnostics_write_kws_line(line, strlen(line));
}
static void save_pending_config(void) {
    /* Open playback/capture interfaces must not starve persistence.
     * This runs in the priority-1 control task, below audio.
     * PSRAM XIP + RAM model copies keep audio service runnable during Flash.
     * Saving never pauses/resets DSP or mutes USB capture. */
    if (!s_save_pending || s_transaction)
        return;
    esp_err_t err = custom_storage_save(&s_pending_saved);
    if (err == ESP_OK)
        custom_state_mark_saved(s_pending_revision);
    s_save_pending = false;
    save_event(err == ESP_OK ? CUSTOM_OK : CUSTOM_FLASH_ERROR);
}
static void control_task(void *arg) {
    (void)arg;
    uint32_t connection = s_connection;
    while (true) {
        if (connection != s_connection) {
            connection = s_connection;
            memset(s_replay, 0, sizeof(s_replay));
            s_session_valid = false;
            s_subscribed = 0;
            cJSON_free(s_snapshot);
            s_snapshot = NULL;
            s_snapshot_id = 0;
        }
        if (s_transaction && (s_update_connection != s_connection ||
                              esp_timer_get_time() - s_last_update > 60000000)) {
            maintenance_end();
            s_update_state = 3;
            s_update_error = CUSTOM_INVALID_STATE;
        }
        message_t msg;
        if (xQueueReceive(s_commands, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (msg.console_command)
                process_console(msg.console_command);
            else
                process(&msg);
        }
        save_pending_config();
    }
}
static void tx_task(void *arg) {
    (void)arg;
    message_t msg;
    while (true) {
        if (xQueueReceive(s_replies, &msg, pdMS_TO_TICKS(2)) != pdTRUE &&
            xQueueReceive(s_events, &msg, 0) != pdTRUE)
            continue;
        while (msg.connection == s_connection && !tud_hid_n_ready(CUSTOM_HID_INSTANCE))
            vTaskDelay(pdMS_TO_TICKS(2));
        if (msg.connection == s_connection)
            (void)tud_hid_n_report(CUSTOM_HID_INSTANCE, 0, msg.wire, 64);
    }
}
esp_err_t custom_control_init(void) {
    s_hid_logs_enabled = false;
    s_next_id = esp_random();
    int slot;
    esp_err_t err = custom_storage_init(&s_saved, &s_meta, &slot);
    custom_kws_filter_t filter;
    custom_state_filter_init(custom_storage_filter_read(&filter) ? &filter : NULL);
    custom_state_init(s_saved.generation ? &s_saved : NULL, &s_meta, slot);
    s_update_error = err == ESP_OK ? CUSTOM_OK : CUSTOM_FLASH_ERROR;
    s_commands = xQueueCreate(CUSTOM_COMMAND_DEPTH, sizeof(message_t));
    s_replies = xQueueCreate(16, sizeof(message_t));
    s_events = xQueueCreate(16, sizeof(message_t));
    return s_commands && s_replies && s_events ? ESP_OK : ESP_ERR_NO_MEM;
}
esp_err_t custom_control_start(void) {
    if (xTaskCreate(control_task, "custom_control", 12288, NULL, CUSTOM_CONTROL_TASK_PRIORITY, NULL) != pdPASS)
        return ESP_ERR_NO_MEM;
    return xTaskCreate(tx_task, "custom_hid_tx", 3072, NULL, 2, NULL) == pdPASS ? ESP_OK
                                                                                : ESP_ERR_NO_MEM;
}
void custom_control_boot_fallback(void) {
    custom_storage_embedded_meta(&s_meta);
    bool restore = custom_storage_restore(255, &s_saved) &&
                   !memcmp(s_saved.package_sha, s_meta.package_sha, 32);
    custom_state_select(&s_meta, -1, restore ? &s_saved : NULL);
    custom_state_set_actions_enabled(false);
}
bool custom_control_console_command(uint8_t command) {
    bool hid_logs_command = command == 'u' || command == 'U';
    if (command != 'a' && command != 'A' && command != 'x' && command != 'X' && !hid_logs_command)
        return false;
    message_t msg = {.console_command = command};
    if (!s_commands || xQueueSend(s_commands, &msg, 0) != pdTRUE) {
        const char *line = hid_logs_command ? "HID logs command not queued; retry\n"
                                            : "KWS actions command not queued; retry\n";
        audio_diagnostics_write_kws_line(line, strlen(line));
    }
    return true;
}
void custom_hid_receive(const uint8_t *data, uint16_t size) {
    if (!s_commands || size != 64)
        return;
    message_t msg = {.connection = s_connection};
    memcpy(msg.wire, data, 64);
    if (xQueueSend(s_commands, &msg, 0) != pdTRUE) {
        __atomic_fetch_add(&s_queue_dropped, 1, __ATOMIC_RELAXED);
        custom_frame_t in, out = {0};
        custom_frame_decode(&in, data, size);
        out.type = CUSTOM_RESPONSE;
        out.command = in.command;
        out.sequence = in.sequence;
        out.transaction = in.transaction;
        out.status = CUSTOM_QUEUE_FULL;
        uint8_t wire[64];
        custom_frame_encode(wire, &out);
        queue_wire(s_replies, wire, s_connection, 0);
    }
}
void custom_hid_disconnected(void) {
    __atomic_fetch_add(&s_connection, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&s_subscribed, 0, __ATOMIC_RELEASE);
}
void custom_keyword_event(int index) {
    if (!__atomic_load_n(&s_subscribed, __ATOMIC_ACQUIRE) || !s_events)
        return;
    custom_binding_t b;
    if (!custom_binding_get(index, &b, NULL))
        return;
    uint32_t seq = __atomic_add_fetch(&s_event_sequence, 1, __ATOMIC_RELAXED);
    custom_frame_t out = {.type = CUSTOM_EVENT,
                          .command = CUSTOM_EVENT_KEYWORD,
                          .sequence = (uint16_t)seq,
                          .length = 31};
    custom_put_u32(out.payload, custom_boot_id());
    custom_put_u32(out.payload + 4, custom_model_epoch());
    custom_put_u32(out.payload + 8, custom_binding_revision());
    custom_put_u32(out.payload + 12, seq);
    uint64_t time = (uint64_t)esp_timer_get_time();
    custom_put_u32(out.payload + 16, (uint32_t)time);
    custom_put_u32(out.payload + 20, time >> 32);
    custom_put_u16(out.payload + 24, index);
    out.payload[26] = b.actions_enabled ? 1 : 0;
    custom_put_u32(out.payload + 27, s_event_dropped);
    uint8_t wire[64];
    custom_frame_encode(wire, &out);
    queue_wire(s_events, wire, s_connection, 0);
}
