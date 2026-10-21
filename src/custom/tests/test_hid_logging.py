"""Exercise HID dispatch, replay and UART feedback without USB hardware."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/usb_driver/tests"))
from test_capture2_stereo import function

HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "custom_protocol.h"
#define pdMS_TO_TICKS(ms) (ms)
#define CUSTOM_CONTROL_TASK_PRIORITY 1U
typedef struct {
    uint32_t connection;
    uint8_t console_command, wire[CUSTOM_REPORT_BYTES];
} message_t;
typedef struct { bool valid; uint8_t request[64], response[64]; } replay_t;
static unsigned s_replies=1, s_events=2, reply_count, event_count, execute_count;
static uint32_t s_connection=1, s_transaction, s_subscribed, s_snapshot_id;
static uint32_t s_pending_epoch=7, s_pending_revision=42, revision=41;
static unsigned stream_mask=6, s_replay_pos;
static char *s_snapshot;
static replay_t s_replay[16];
static uint8_t s_session[16], response[64], event[64];
static bool s_session_valid;
static uint16_t s_update_error;
static uint16_t next_status=CUSTOM_OK;
typedef int esp_err_t;
#define ESP_OK 0
typedef struct { uint32_t token; } custom_saved_config_t;
static custom_saved_config_t s_pending_saved={.token=123};
static bool s_save_pending;
static bool s_hid_logs_enabled;
static unsigned storage_calls;
static esp_err_t storage_result;
static uint32_t saved_revision;
static esp_err_t custom_storage_save(custom_saved_config_t *saved) {
    assert(saved==&s_pending_saved && saved->token==123);
    ++storage_calls; return storage_result;
}
static void custom_state_mark_saved(uint32_t value) { saved_revision=value; }
static void custom_state_set_actions_enabled(bool enabled) { (void)enabled; }
static esp_err_t usb_hid_keyboard_barrier(uint32_t timeout) { assert(timeout==2500); return ESP_OK; }
static char output[32768];
static size_t output_size;
static unsigned usb_audio_stream_mask(void) { return stream_mask; }
static uint32_t custom_binding_revision(void) { return revision; }
static void audio_diagnostics_write_kws_line(const char *line, size_t size) {
    assert(size && line[size-1]=='\n' && output_size+size<sizeof(output));
    memcpy(output+output_size,line,size); output_size+=size; output[output_size]=0;
}
static void cJSON_free(void *p) { free(p); }
static void xQueueReset(unsigned q) { assert(q==s_events); }
static void queue_wire(unsigned q, const uint8_t *wire, uint32_t connection, unsigned wait) {
    assert(connection==s_connection);
    if (q==s_replies) { assert(wait==100); ++reply_count; memcpy(response,wire,64); }
    else { assert(q==s_events && !wait); ++event_count; memcpy(event,wire,64); }
}
static uint16_t execute(const custom_frame_t *in, custom_frame_t *out) {
    ++execute_count;
    if (next_status!=CUSTOM_OK) return next_status;
    if (in->command==CUSTOM_SET_BINDING) {
        out->length=4; custom_put_u32(out->payload,++revision);
    } else if (in->command==CUSTOM_UPDATE_BEGIN) {
        out->transaction=99; out->length=1; out->payload[0]=1;
    }
    return CUSTOM_OK;
}
FUNCTIONS
static void clear_output(void) { output_size=0; output[0]=0; }
static message_t request(uint16_t command, uint16_t sequence, unsigned size) {
    custom_frame_t frame={.command=command,.sequence=sequence,.length=size};
    message_t msg={.connection=s_connection};
    custom_frame_encode(msg.wire,&frame); return msg;
}
static message_t binding_request(uint16_t seq) {
    custom_frame_t frame={.command=CUSTOM_SET_BINDING,.sequence=seq,.length=29};
    custom_put_u16(frame.payload+12,2);
    frame.payload[14]=7; frame.payload[15]=255; frame.payload[17]=128;
    frame.payload[18]=3; frame.payload[19]=6; frame.payload[20]=25;
    custom_put_u16(frame.payload+25,600); custom_put_u16(frame.payload+27,30);
    message_t msg={.connection=s_connection}; custom_frame_encode(msg.wire,&frame); return msg;
}
int main(void) {
    assert(!s_hid_logs_enabled);
    message_t silent=binding_request(60000);
    unsigned replies_before=reply_count;
    process(&silent);
    assert(!output_size && reply_count==replies_before+1 && revision==42);
    silent.wire[63]^=1; process(&silent); // Errors obey the same default-off gate.
    assert(!output_size && custom_get_u16(response+18)==CUSTOM_BAD_CRC);
    save_event(CUSTOM_OK);
    assert(!output_size && event_count==1); // HID SAVE events remain enabled independently of UART.
    s_transaction=99;
    process_console('u'); // Toggle is allowed during model maintenance.
    assert(s_hid_logs_enabled && !strcmp(output,"HID logs ON\n"));
    s_transaction=0; clear_output();
    memset(s_replay,0,sizeof(s_replay)); s_replay_pos=0; revision=41; event_count=0;
    message_t msg=request(CUSTOM_OPEN_SESSION,1,16);
    process(&msg);
    assert(s_session_valid && strstr(output,"HID OPEN_SESSION") && strstr(output,"status=OK(0)"));
    clear_output();
    msg=binding_request(2); process(&msg);
    assert(strstr(output,"HID SET_BINDING") && strstr(output,"class=2 flags=0x07 rgb=255,0,128"));
    assert(strstr(output,"modifier=0x03 keys=06,19,00,00,00,00 led_ms=600 hold_ms=30 revision=42"));
    assert(strstr(output,"status=OK(0) replay=0"));
    unsigned calls=execute_count;
    clear_output(); process(&msg);
    assert(execute_count==calls && strstr(output,"status=OK(0) replay=1"));
    clear_output();
    msg.wire[34]=5; custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60)); process(&msg);
    assert(execute_count==calls && strstr(output,"status=CONFLICT(14)"));
    clear_output();
    msg=binding_request(3); msg.wire[63]^=1; process(&msg);
    assert(execute_count==calls && strstr(output,"status=BAD_CRC(5)"));
    clear_output();
    msg=request(CUSTOM_GET_STATUS,4,0); msg.wire[2]=2; process(&msg);
    assert(execute_count==calls && strstr(output,"status=BAD_VERSION(3)"));
    clear_output();
    msg=request(CUSTOM_SAVE_CONFIG,5,12); custom_put_u32(msg.wire+28,42);
    custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60));
    next_status=CUSTOM_ACCEPTED; process(&msg);
    assert(strstr(output,"status=ACCEPTED(1)") && strstr(output,"revision=42 save_priority=1 stream_mask=0x06"));
    assert(!strstr(output,"complete"));
    clear_output(); save_event(CUSTOM_OK);
    assert(strstr(output,"HID SAVE_CONFIG complete status=OK(0) model_epoch=7 revision=42"));
    assert(event_count==1 && custom_get_u16(event+4)==CUSTOM_EVENT_SAVE && custom_get_u16(event+18)==CUSTOM_OK);
    clear_output(); save_event(CUSTOM_FLASH_ERROR);
    assert(strstr(output,"complete status=FLASH_ERROR(13)"));
    clear_output(); process_console('U');
    assert(!s_hid_logs_enabled && !strcmp(output,"HID logs OFF\n"));
    clear_output(); msg=binding_request(100); next_status=CUSTOM_OK;
    unsigned events_before=event_count; calls=execute_count;
    process(&msg); save_event(CUSTOM_FLASH_ERROR);
    assert(!output_size && execute_count==calls+1 && event_count==events_before+1);
    assert(custom_get_u16(event+18)==CUSTOM_FLASH_ERROR);
    process_console('u'); assert(s_hid_logs_enabled);
    clear_output();
    next_status=CUSTOM_OK;
    const uint16_t bulk[]={CUSTOM_READ_CHUNK,CUSTOM_READ_END,CUSTOM_OBJECT_DATA};
    for (unsigned i=0;i<3;++i) {
        clear_output(); msg=request(bulk[i],10+i,0); process(&msg); assert(!output_size);
        process(&msg); assert(!output_size); // Successful bulk retries also stay silent.
    }
    clear_output();
    msg=request(CUSTOM_OBJECT_DATA,20,1);
    custom_put_u32(msg.wire+8,99); custom_put_u32(msg.wire+12,400);
    custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60));
    next_status=CUSTOM_OFFSET_MISMATCH; process(&msg);
    assert(strstr(output,"HID OBJECT_DATA") && strstr(output,"status=OFFSET_MISMATCH(8)"));
    assert(strstr(output,"transaction=99 offset=400"));
    next_status=CUSTOM_OK;
    clear_output(); msg=request(CUSTOM_UPDATE_BEGIN,21,0); process(&msg);
    assert(strstr(output,"HID UPDATE_BEGIN") && strstr(output,"target_slot=1") && strstr(output,"transaction=99"));
    clear_output(); msg=request(CUSTOM_ACTIVATE,22,1); msg.wire[20]=255;
    custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60)); process(&msg);
    assert(strstr(output,"HID ACTIVATE") && strstr(output,"slot=embedded(255)"));
    clear_output(); msg=request(CUSTOM_READ_BEGIN,23,2); custom_put_u16(msg.wire+20,CUSTOM_OBJ_CLASSES);
    custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60)); process(&msg);
    assert(strstr(output,"HID READ_BEGIN") && strstr(output,"object=0x0004"));
    clear_output(); msg=request(CUSTOM_OBJECT_BEGIN,24,38);
    custom_put_u16(msg.wire+20,CUSTOM_OBJ_MODEL); custom_put_u32(msg.wire+22,138064);
    custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60)); process(&msg);
    assert(strstr(output,"HID OBJECT_BEGIN") && strstr(output,"object=0x0101 bytes=138064"));
    const uint16_t operations[]={CUSTOM_GET_INFO,CUSTOM_GET_CAPS,CUSTOM_GET_STATUS,CUSTOM_OBJECT_END,
        CUSTOM_UPDATE_COMMIT,CUSTOM_UPDATE_ABORT,CUSTOM_SUBSCRIBE};
    for (unsigned i=0;i<sizeof(operations)/sizeof(operations[0]);++i) {
        clear_output(); msg=request(operations[i],30+i,operations[i]==CUSTOM_SUBSCRIBE ? 1 : 0);
        process(&msg); assert(strstr(output,hid_command_name(operations[i])) && strstr(output,"status=OK(0)"));
    }
    clear_output(); msg=request(0x9999,40,0); next_status=CUSTOM_BAD_COMMAND; process(&msg);
    assert(strstr(output,"HID UNKNOWN cmd=0x9999") && strstr(output,"status=BAD_COMMAND(18)"));
    clear_output(); calls=execute_count; msg.connection=0; process(&msg);
    assert(execute_count==calls && !output_size); // Retired USB connection cannot execute/log an operation.
    /* Exercise the real save scheduler with every playback/capture combination. */
    next_status=CUSTOM_ACCEPTED;
    for (unsigned mask=0;mask<8;++mask) {
        clear_output(); stream_mask=mask; storage_calls=0; saved_revision=0; s_save_pending=true;
        s_pending_revision=42; revision=43; // A later RAM edit is not part of the frozen save.
        msg=request(CUSTOM_SAVE_CONFIG,50+mask,12); custom_put_u32(msg.wire+28,42);
        custom_put_u32(msg.wire+60,custom_crc32(msg.wire,60)); process(&msg);
        assert(strstr(output,"save_priority=1") && !strstr(output,"wait_playback_idle"));
        save_pending_config();
        assert(!s_save_pending && storage_calls==1 && saved_revision==42 && revision==43);
        assert(strstr(output,"complete status=OK(0)"));
        save_pending_config(); assert(storage_calls==1); // No duplicate Flash write.
    }
    clear_output(); stream_mask=6; s_save_pending=true; s_transaction=99; storage_calls=0;
    save_pending_config(); assert(s_save_pending && !storage_calls && !output_size);
    s_transaction=0; storage_result=1; saved_revision=0;
    save_pending_config();
    assert(!s_save_pending && storage_calls==1 && !saved_revision);
    assert(strstr(output,"complete status=FLASH_ERROR(13)"));
    clear_output(); process_console('U');
    assert(!s_hid_logs_enabled && !strcmp(output,"HID logs OFF\n"));
    clear_output(); s_save_pending=true; stream_mask=7; storage_result=ESP_OK;
    unsigned storage_before=storage_calls; events_before=event_count;
    save_pending_config();
    assert(!output_size && !s_save_pending && storage_calls==storage_before+1 && event_count==events_before+1);
    puts("PASS HID UART default off/toggle, replay, errors, all stream masks and save events while muted");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", nargs="+", required=True)
    args = parser.parse_args()
    control = (ROOT / "src/custom/custom_control.c").read_text(encoding="utf-8")
    names = ("hid_command_name", "hid_status_name", "log_hid_result", "log_hid_save_result", "save_event", "save_pending_config", "process_console", "process")
    source = HARNESS.replace("FUNCTIONS", "\n".join(function(control, name) for name in names))
    with tempfile.TemporaryDirectory(prefix="hid-logging-") as folder:
        folder = Path(folder)
        harness, binary = folder / "test.c", folder / "test.exe"
        harness.write_text(source, encoding="utf-8")
        subprocess.run(args.cc + ["-std=c11", "-Wall", "-Wextra", "-Werror", "-UNDEBUG",
            f"-I{ROOT / 'src/custom'}", str(harness), str(ROOT / "src/custom/custom_protocol.c"),
            "-o", str(binary)], check=True, timeout=60)
        subprocess.run([str(binary)], check=True, timeout=30)


if __name__ == "__main__":
    main()
