#include "custom_internal.h"
#include <assert.h>
#include <string.h>
static unsigned ticks;
static bool audio_maintenance;
uint32_t esp_random(void) { return 123456; }
unsigned xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(unsigned count) { ticks+=count; }
void usb_audio_set_maintenance(bool active) { audio_maintenance=active; }

int main(void) {
    custom_state_filter_init(NULL);
    custom_kws_filter_t filter;
    uint32_t filter_revision;
    custom_kws_filter_get(&filter, &filter_revision);
    assert(filter_revision==1 && filter.keyword_threshold_percent==80 &&
           filter.cooldown_ms==500 && filter.stable_frames==3 && !filter.need_vad_checked);
    filter.keyword_threshold_percent=0; filter.cooldown_ms=1;
    filter.stable_frames=1000; filter.need_vad_checked=1;
    assert(custom_filter_valid(&filter));
    custom_state_filter_set(&filter);
    custom_kws_filter_get(&filter, &filter_revision);
    assert(filter_revision==2 && filter.keyword_threshold_percent==0 && filter.need_vad_checked);
    filter.cooldown_ms=0; assert(!custom_filter_valid(&filter));
    filter.cooldown_ms=1; filter.stable_frames=0; assert(!custom_filter_valid(&filter));
    filter.stable_frames=1; filter.keyword_threshold_percent=101; assert(!custom_filter_valid(&filter));
    filter.keyword_threshold_percent=100; filter.need_vad_checked=2; assert(!custom_filter_valid(&filter));
    custom_slot_meta_t m={.class_count=34}; m.package_sha[0]=1;
    custom_state_init(NULL,&m,0);
    uint32_t boot=custom_boot_id(), epoch=custom_model_epoch(), revision=custom_binding_revision();
    custom_binding_t b={.actions_enabled=true,.led_enabled=true,.keyboard_enabled=true,
        .keycodes={6},.led_duration_ms=600};
    assert(custom_class_count()==34);
    assert(custom_state_set_binding(boot,epoch,revision,0,&b)==CUSTOM_OUT_OF_RANGE);
    assert(custom_state_set_binding(boot,epoch,revision,1,&b)==CUSTOM_OUT_OF_RANGE);
    assert(custom_state_set_binding(boot,epoch,revision,34,&b)==CUSTOM_OUT_OF_RANGE);
    assert(custom_state_set_binding(boot+1,epoch,revision,33,&b)==CUSTOM_MODEL_CHANGED);
    assert(custom_state_set_binding(boot,epoch+1,revision,33,&b)==CUSTOM_MODEL_CHANGED);
    assert(custom_state_set_binding(boot,epoch,revision+1,33,&b)==CUSTOM_CONFLICT);
    assert(custom_state_set_binding(boot,epoch,revision,33,&b)==CUSTOM_OK);
    revision=custom_binding_revision();
    assert(custom_action_allowed(33,epoch,revision,false));
    assert(custom_action_allowed(33,epoch,revision,true));
    assert(!custom_action_allowed(33,epoch,revision-1,true));
    assert(custom_state_set_binding(boot,epoch,revision,2,&b)==CUSTOM_OK);
    assert(custom_action_allowed(33,epoch,revision,true)); /* Another word's edit must not cancel this word. */
    revision=custom_binding_revision();
    b.actions_enabled=false;
    assert(custom_state_set_binding(boot,epoch,revision,33,&b)==CUSTOM_OK);
    assert(custom_class_count()==34); /* Disabling an action cannot change the model. */
    assert(!custom_action_allowed(33,epoch,custom_binding_revision(),false));
    assert(!custom_action_allowed(33,epoch,revision,true));
    b.actions_enabled=true; b.keycodes[1]=6;
    assert(custom_state_set_binding(boot,epoch,custom_binding_revision(),33,&b)==CUSTOM_OUT_OF_RANGE);
    b.keycodes[1]=0;
    assert(custom_state_set_binding(boot,epoch,custom_binding_revision(),33,&b)==CUSTOM_OK);
    custom_saved_config_t saved;
    custom_state_capture(&saved);
    assert(saved.bindings[33].actions_enabled);
    assert(custom_maintenance_enter()==ESP_OK && audio_maintenance);
    assert(!custom_action_allowed(33,epoch,custom_binding_revision(),true));
    custom_maintenance_leave(); assert(!audio_maintenance);
    m.package_sha[0]=2; m.class_count=3;
    custom_state_select(&m,1,&saved); /* Old model's bindings must not transfer by index. */
    assert(custom_class_count()==3 && custom_model_epoch()!=epoch);
    custom_binding_t current;
    assert(custom_binding_get(2,&current,NULL) && !current.actions_enabled);
    assert(!custom_binding_get(33,&current,NULL));
    m.package_sha[0]=1; m.class_count=34;
    custom_state_select(&m,0,&saved);
    assert(custom_binding_get(33,&current,NULL) && current.actions_enabled);
    /* A reboot restores mappings, never persisted enable flags. */
    custom_state_init(&saved,&m,0);
    assert(custom_config_dirty());
    for (int i=2;i<34;++i) {
        uint32_t fence;
        assert(custom_binding_get(i,&current,&fence));
        assert(!current.actions_enabled && !current.led_enabled && !current.keyboard_enabled);
        assert(!custom_action_allowed(i,custom_model_epoch(),fence,false));
        assert(!custom_action_allowed(i,custom_model_epoch(),fence,true));
    }
    assert(current.keycodes[0]==6 && current.led_duration_ms==600);
    revision=custom_binding_revision();
    custom_state_set_actions_enabled(false);
    assert(custom_binding_revision()==revision); /* Repeated off is idempotent. */
    custom_state_set_actions_enabled(true);
    uint32_t enabled_fence;
    assert(custom_binding_get(33,&current,&enabled_fence));
    assert(current.actions_enabled && current.led_enabled && current.keyboard_enabled);
    assert(custom_action_allowed(33,custom_model_epoch(),enabled_fence,true));
    assert(custom_binding_get(3,&current,NULL));
    assert(current.actions_enabled && current.led_enabled && !current.keyboard_enabled); /* No mapping. */
    revision=custom_binding_revision();
    custom_state_set_actions_enabled(true);
    assert(custom_binding_revision()==revision);
    custom_state_set_actions_enabled(false);
    custom_state_set_actions_enabled(true);
    assert(!custom_action_allowed(33,custom_model_epoch(),enabled_fence,true)); /* No stale replay. */
    /* Existing HID binding writes can selectively enable a word after reboot. */
    custom_state_init(&saved,&m,0);
    assert(custom_state_set_binding(custom_boot_id(),custom_model_epoch(),custom_binding_revision(),33,&b)==CUSTOM_OK);
    assert(custom_binding_get(33,&current,&revision) && current.keyboard_enabled);
    assert(custom_action_allowed(33,custom_model_epoch(),revision,true));
    assert(custom_binding_get(2,&current,NULL) && !current.actions_enabled);
    /* Factory defaults keep colors/keycodes, with all action switches off. */
    m.class_count=8;
    custom_state_init(NULL,&m,-1);
    assert(custom_binding_get(2,&current,NULL));
    assert(!current.actions_enabled && !current.led_enabled && !current.keyboard_enabled);
    assert(current.g==255 && current.keycodes[0]==28 && current.led_duration_ms==600);
    custom_state_capture(&saved);
    custom_state_mark_saved(custom_binding_revision());
    custom_state_init(&saved,&m,-1);
    assert(!custom_config_dirty()); /* Already-disabled storage still matches. */
    return 0;
}
