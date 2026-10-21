"""Exercise the real runtime bindings, action dispatcher and keyboard worker."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
from test_audio_diagnostics import HEADERS
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/usb_driver/tests'))
from test_capture2_stereo import function
HARNESS=r'''
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "custom_internal.h"
#include "keyword_actions.h"
#include "led_ring.h"
#include "tusb.h"
#include "freertos/queue.h"
#include "freertos/task.h"
struct MockQueue { unsigned count,head,length,bytes; unsigned char data[8][80]; };
static void (*actions_worker)(void *),(*hid_worker)(void *);
static jmp_buf done;
static unsigned ticks,led_count,report_count,event_count;
static bool fail_queue,fail_task,host_ready=true,mounted=true,disable_on_delay,disconnect_on_press,reconnect_on_delay;
static struct { uint8_t modifier,keys[6]; } reports[128];
static void disable_word(void) {
    custom_binding_t b; custom_binding_get(2,&b,NULL); b.actions_enabled=false;
    assert(custom_state_set_binding(custom_boot_id(),custom_model_epoch(),custom_binding_revision(),2,&b)==CUSTOM_OK);
}
QueueHandle_t xQueueCreate(unsigned length,unsigned bytes) {
    if (fail_queue) return NULL;
    assert(length==8 && bytes<=80);
    struct MockQueue *q=calloc(1,sizeof(*q)); q->length=length; q->bytes=bytes; return q;
}
void vQueueDelete(QueueHandle_t q) { free(q); }
BaseType_t xQueueSend(QueueHandle_t q,const void *data,TickType_t wait) {
    assert(wait==0);
    if (q->count==q->length) return pdFALSE;
    memcpy(q->data[(q->head+q->count++)%q->length],data,q->bytes); return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q,void *data,TickType_t wait) {
    assert(wait==portMAX_DELAY);
    if (!q->count) longjmp(done,1);
    memcpy(data,q->data[q->head],q->bytes); q->head=(q->head+1)%q->length; --q->count; return pdTRUE;
}
BaseType_t xTaskCreate(void (*entry)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *handle) {
    assert(stack==3072 && !arg && !handle);
    if (fail_task) return pdFALSE;
    if (!strcmp(name,"keyword_actions")) { assert(priority==2); actions_worker=entry; }
    else { assert(!strcmp(name,"usb_keyboard") && priority==5); hid_worker=entry; }
    return pdPASS;
}
static void drain(void (*worker)(void *)) { assert(worker); if (!setjmp(done)) worker(NULL); }
TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t delay) {
    ticks+=delay;
    if (disable_on_delay) { disable_on_delay=false; disable_word(); }
    if (reconnect_on_delay) { reconnect_on_delay=false; host_ready=mounted=true; }
}
uint32_t esp_random(void) { return 1234; }
void usb_audio_set_maintenance(bool active) { (void)active; }
void custom_keyword_event(int index) { assert(index>=2 && index<custom_class_count()); ++event_count; }
void custom_hid_receive(const uint8_t *data,uint16_t size) { (void)data; (void)size; }
bool tud_mounted(void) { return mounted; }
bool tud_hid_n_ready(uint8_t instance) { assert(instance==0); return host_ready; }
bool tud_hid_n_keyboard_report(uint8_t instance,uint8_t id,uint8_t modifier,const uint8_t *keys) {
    assert(instance==0 && id==0 && report_count<128);
    reports[report_count].modifier=modifier;
    if (keys) memcpy(reports[report_count].keys,keys,6); else memset(reports[report_count].keys,0,6);
    ++report_count;
    if (keys && disconnect_on_press) { disconnect_on_press=false; host_ready=mounted=false; reconnect_on_delay=true; }
    return true;
}
esp_err_t led_ring_trigger_keyword(led_ring_rgb_t color,const led_ring_keyword_options_t *options) {
    (void)color;
    assert(options && options->guarded && !options->has_next_mode);
    assert(custom_action_allowed(options->class_index,options->model_epoch,options->config_revision,false));
    ++led_count; return ESP_OK;
}
static void enable_word(unsigned hold) {
    custom_binding_t b; custom_binding_get(2,&b,NULL); b.actions_enabled=b.led_enabled=b.keyboard_enabled=true;
    b.modifier=1; b.keycodes[0]=6; b.key_hold_ms=hold;
    assert(custom_state_set_binding(custom_boot_id(),custom_model_epoch(),custom_binding_revision(),2,&b)==CUSTOM_OK);
}
typedef struct {
    uint32_t connection;
    uint8_t console_command,wire[CUSTOM_REPORT_BYTES];
} message_t;
static QueueHandle_t s_commands;
static uint32_t s_transaction;
static bool s_hid_logs_enabled;
static char console_reply[128];
void audio_diagnostics_write_kws_line(const char *line,size_t length) {
    assert(length<sizeof(console_reply)); memcpy(console_reply,line,length); console_reply[length]=0;
}
/* Actual UART queueing and execution functions from custom_control.c. */
CONSOLE_FUNCTIONS
static void drain_console(void) {
    message_t msg;
    assert(xQueueReceive(s_commands,&msg,portMAX_DELAY)==pdTRUE);
    process_console(msg.console_command);
}
int main(void) {
    custom_slot_meta_t meta={.class_count=8}; custom_state_init(NULL,&meta,-1);
    fail_queue=true; assert(usb_hid_keyboard_init()==ESP_ERR_NO_MEM); fail_queue=false;
    fail_task=true; assert(usb_hid_keyboard_init()==ESP_ERR_NO_MEM); fail_task=false;
    assert(usb_hid_keyboard_init()==ESP_OK);
    fail_task=true; assert(keyword_actions_init()==ESP_ERR_NO_MEM); fail_task=false;
    assert(keyword_actions_init()==ESP_OK);
    assert(!keyword_actions_submit(0) && !keyword_actions_submit(1) && !keyword_actions_submit(8));
    for (int i=2;i<8;++i) assert(keyword_actions_submit(i));
    drain(actions_worker); drain(hid_worker);
    assert(event_count==6 && report_count==0 && led_count==0); // Boot: recognition only.
    event_count=0;
    assert(!custom_control_console_command('p'));
    assert(!s_hid_logs_enabled);
    assert(custom_control_console_command('u') && strstr(console_reply,"HID logs command not queued"));
    assert(custom_control_console_command('a') && strstr(console_reply,"retry")); // Not initialized.
    s_commands=xQueueCreate(8,sizeof(message_t));
    uint32_t initial_revision=custom_binding_revision();
    assert(custom_control_console_command('u') && !s_hid_logs_enabled);
    drain_console(); assert(s_hid_logs_enabled && !strcmp(console_reply,"HID logs ON\n"));
    s_transaction=1; // Debug feedback can be toggled during a model update.
    assert(custom_control_console_command('U')); drain_console();
    assert(!s_hid_logs_enabled && !strcmp(console_reply,"HID logs OFF\n"));
    assert(custom_binding_revision()==initial_revision); // No binding edits or persistence.
    s_transaction=0;
    assert(custom_control_console_command('a'));
    custom_binding_t pending; custom_binding_get(2,&pending,NULL);
    assert(!pending.actions_enabled); // UART only queues; HID and UART share one writer.
    drain_console(); assert(strstr(console_reply,"ON"));
    for (int i=2;i<8;++i) assert(keyword_actions_submit(i));
    drain(actions_worker); drain(hid_worker);
    assert(event_count==6 && report_count==12);
#if CONFIG_WS2812_LED_RING_ENABLE
    assert(led_count==6);
#else
    assert(led_count==0);
#endif
    unsigned before=report_count,led_before=led_count;
    assert(keyword_actions_submit(2)); disable_word();
    drain(actions_worker); drain(hid_worker);
    assert(report_count==before && led_count==led_before);
    assert(keyword_actions_submit(2)); drain(actions_worker); drain(hid_worker);
    assert(event_count==8 && report_count==before); // Disabled words still produce recognition events.
    enable_word(0);
    assert(keyword_actions_submit(2)); drain(actions_worker); disable_word(); drain(hid_worker);
    assert(report_count==before); // A press already in the keyboard queue is also canceled.
    enable_word(30); disable_on_delay=true;
    assert(keyword_actions_submit(2)); drain(actions_worker); drain(hid_worker);
    assert(report_count==before+2 && reports[before].modifier==1 && reports[before].keys[0]==6);
    assert(!reports[before+1].modifier && !reports[before+1].keys[0]);
    before=report_count; enable_word(0); disconnect_on_press=true;
    assert(keyword_actions_submit(2)); drain(actions_worker); drain(hid_worker);
    assert(report_count==before+2 && reports[before+1].keys[0]==0); // Reconnect releases before any new press.
    before=report_count;
    assert(keyword_actions_submit(2));
    custom_state_set_actions_enabled(false);
    drain(actions_worker); drain(hid_worker);
    assert(report_count==before); // Serial bulk disable cancels queued actions.
    assert(custom_control_console_command('A')); drain_console(); assert(strstr(console_reply,"ON"));
    s_transaction=1;
    assert(custom_control_console_command('x')); drain_console(); assert(strstr(console_reply,"BUSY"));
    custom_binding_get(2,&pending,NULL); assert(pending.actions_enabled);
    s_transaction=0;
    assert(custom_control_console_command('X')); drain_console(); assert(strstr(console_reply,"OFF"));
    custom_binding_get(2,&pending,NULL);
    assert(!pending.actions_enabled && !pending.led_enabled && !pending.keyboard_enabled);
    for (int i=0;i<8;++i) assert(custom_control_console_command('a'));
    assert(custom_control_console_command('x') && strstr(console_reply,"retry")); // FIFO full.
    for (int i=0;i<8;++i) drain_console();
    assert(custom_maintenance_enter()==ESP_OK);
    assert(keyword_actions_submit(2)); drain(actions_worker); drain(hid_worker);
    assert(report_count==before);
    custom_maintenance_leave();
    // Action saturation cannot block or prevent recognition acknowledgement.
    for (unsigned i=0;i<20;++i) assert(keyword_actions_submit(2));
    drain(actions_worker); drain(hid_worker);
    assert(usb_hid_keyboard_barrier(1)==ESP_OK);
    puts("PASS boot disabled, UART control, runtime mappings, stale queues, release, reconnect, maintenance and saturation");
    return 0;
}
'''
def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--cc',nargs='+',required=True); args=parser.parse_args()
    headers=dict(HEADERS)
    headers['esp_err.h']+='\n#define ESP_ERR_INVALID_ARG 3\n#define ESP_ERR_TIMEOUT 4\n'
    headers['freertos/FreeRTOS.h']+='\n#define portMAX_DELAY 0xffffffffU\n'
    headers['freertos/task.h']+='\nBaseType_t xTaskCreate(void (*)(void *),const char *,unsigned,void *,unsigned,TaskHandle_t *);\n'
    headers['esp_partition.h']='#pragma once\ntypedef struct esp_partition_t esp_partition_t;\n'
    headers['esp_random.h']='#pragma once\n#include <stdint.h>\nuint32_t esp_random(void);\n'
    headers['tusb.h']='''#pragma once
#include <stdbool.h>
#include <stdint.h>
enum { HID_KEY_A=4,HID_KEY_C=6,HID_KEY_V=25,KEYBOARD_MODIFIER_LEFTCTRL=1 };
typedef enum { HID_REPORT_TYPE_INPUT=1,HID_REPORT_TYPE_OUTPUT=2 } hid_report_type_t;
bool tud_hid_n_ready(uint8_t);
bool tud_hid_n_keyboard_report(uint8_t,uint8_t,uint8_t,const uint8_t *);
bool tud_mounted(void);
'''
    with tempfile.TemporaryDirectory(prefix='keyword-actions-') as folder:
        folder=Path(folder)
        for name,text in headers.items():
            target=folder/name; target.parent.mkdir(parents=True,exist_ok=True); target.write_text(text,encoding='utf-8')
        control=(ROOT/'src/custom/custom_control.c').read_text(encoding='utf-8')
        console='\n'.join(function(control,name) for name in ('process_console','custom_control_console_command'))
        source=folder/'test.c'; source.write_text(HARNESS.replace('CONSOLE_FUNCTIONS',console),encoding='utf-8')
        for led in (1,0):
            binary=folder/f'test-led-{led}.exe'
            subprocess.run(args.cc+['-std=c11','-O2','-UNDEBUG',f'-I{folder}',f'-I{ROOT / "src"}',
                f'-I{ROOT / "src/custom"}',f'-I{ROOT / "src/usb_driver"}',f'-I{ROOT / "src/ws2812"}',
                f'-DCONFIG_WS2812_LED_RING_ENABLE={led}','-DCONFIG_USB_AUDIO_USB_TASK_PRIORITY=6',str(source),
                str(ROOT/'src/custom/custom_state.c'),str(ROOT/'src/custom/custom_protocol.c'),
                str(ROOT/'src/keyword_actions.c'),str(ROOT/'src/usb_driver/usb_hid_keyboard.c'),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True,timeout=30)
if __name__=='__main__': main()
