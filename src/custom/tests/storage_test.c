/* Real storage/state code with NOR-flash/NVS mocks and Windows native SHA256. */
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include <bcrypt.h>
#include "custom_internal.h"
#include "nvs.h"
#include "psa/crypto.h"

const uint8_t factory_blob[32] __asm__("_binary_kws_model_espdl_start")={1,2,3};
__asm__(".globl _binary_kws_model_espdl_end\n.set _binary_kws_model_espdl_end, _binary_kws_model_espdl_start+32");
static uint8_t flash_data[2][1024*1024];
static const esp_partition_t parts[2]={{"kws_a",1024*1024},{"kws_b",1024*1024}};
static unsigned ticks;
static bool fail_write;
static struct { char key[16]; size_t size; uint8_t bytes[4096]; } records[8];
static unsigned record_count;
static BCRYPT_ALG_HANDLE algorithm;
uint32_t esp_random(void) { return 1234; }
unsigned xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(unsigned delay) { ticks+=delay; }
void usb_audio_set_maintenance(bool active) { (void)active; }
const esp_partition_t *esp_partition_find_first(unsigned type,unsigned sub,const char *label) {
    assert(type==ESP_PARTITION_TYPE_DATA && sub==ESP_PARTITION_SUBTYPE_ANY);
    return !strcmp(label,"kws_a") ? &parts[0] : !strcmp(label,"kws_b") ? &parts[1] : NULL;
}
static uint8_t *address(const esp_partition_t *p,size_t off,size_t size) {
    assert(p>=parts && p<parts+2 && off<=p->size && size<=p->size-off);
    return flash_data[p-parts]+off;
}
esp_err_t esp_partition_read(const esp_partition_t *p,size_t off,void *data,size_t size) { memcpy(data,address(p,off,size),size); return ESP_OK; }
esp_err_t esp_partition_write(const esp_partition_t *p,size_t off,const void *data,size_t size) {
    if (fail_write) return ESP_FAIL;
    uint8_t *dst=address(p,off,size); const uint8_t *src=data;
    for (size_t i=0;i<size;++i) { assert((dst[i]&src[i])==src[i]); dst[i]&=src[i]; }
    return ESP_OK;
}
esp_err_t esp_partition_erase_range(const esp_partition_t *p,size_t off,size_t size) {
    assert(off%4096==0 && size%4096==0); memset(address(p,off,size),255,size); return ESP_OK;
}
esp_err_t nvs_flash_init_partition(const char *part) { assert(!strcmp(part,"kws_cfg")); return ESP_OK; }
esp_err_t nvs_open_from_partition(const char *part,const char *name,unsigned mode,nvs_handle_t *h) {
    assert(!strcmp(part,"kws_cfg") && !strcmp(name,"custom") && mode==NVS_READWRITE); *h=1; return ESP_OK;
}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *data,size_t *size) {
    assert(h==1);
    for (unsigned i=0;i<record_count;++i) if (!strcmp(records[i].key,key)) {
        assert(*size>=records[i].size); *size=records[i].size; memcpy(data,records[i].bytes,*size); return ESP_OK;
    }
    return ESP_FAIL;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t size) {
    assert(h==1 && size<=4096); unsigned i=0;
    for (;i<record_count;++i) if (!strcmp(records[i].key,key)) break;
    if (i==record_count) { assert(record_count<8); ++record_count; strcpy(records[i].key,key); }
    records[i].size=size; memcpy(records[i].bytes,data,size); return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) { assert(h==1); return ESP_OK; }
psa_status_t psa_crypto_init(void) {
    if (!algorithm && BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,NULL,0)<0) return -1;
    return PSA_SUCCESS;
}
psa_status_t psa_hash_setup(psa_hash_operation_t *h,unsigned alg) {
    assert(alg==PSA_ALG_SHA_256); psa_crypto_init();
    return BCryptCreateHash(algorithm,(BCRYPT_HASH_HANDLE *)&h->handle,NULL,0,NULL,0,0)<0 ? -1 : PSA_SUCCESS;
}
psa_status_t psa_hash_update(psa_hash_operation_t *h,const uint8_t *data,size_t size) {
    return BCryptHashData(h->handle,(PUCHAR)data,(ULONG)size,0)<0 ? -1 : PSA_SUCCESS;
}
psa_status_t psa_hash_abort(psa_hash_operation_t *h) {
    if (h->handle) BCryptDestroyHash(h->handle); h->handle=NULL; return PSA_SUCCESS;
}
psa_status_t psa_hash_finish(psa_hash_operation_t *h,uint8_t *sha,size_t cap,size_t *size) {
    assert(cap>=32); if (BCryptFinishHash(h->handle,sha,32,0)<0) return -1;
    *size=32; return psa_hash_abort(h);
}
psa_status_t psa_hash_compute(unsigned alg,const uint8_t *data,size_t size,uint8_t *sha,size_t cap,size_t *out) {
    psa_hash_operation_t h=PSA_HASH_OPERATION_INIT;
    if (psa_hash_setup(&h,alg) || psa_hash_update(&h,data,size)) return -1;
    return psa_hash_finish(&h,sha,cap,out);
}
static void sha256(const void *data,size_t size,uint8_t sha[32]) {
    size_t out; assert(psa_hash_compute(PSA_ALG_SHA_256,data,size,sha,32,&out)==PSA_SUCCESS && out==32);
}
static const char manifest[]="{\"format_version\":1,\"frontend_profile\":\"mfcc40_16k_int16_state6_v1\",\"classes\":[{\"class_index\":0,\"text\":\"silence\"},{\"class_index\":1,\"text\":\"unknown\"},{\"class_index\":2,\"text\":\"on\"}]}";
static void upload(unsigned object,const uint8_t *data,size_t size) {
    uint8_t sha[32]; sha256(data,size,sha);
    assert(custom_storage_object_begin(object,(uint32_t)size,sha)==CUSTOM_OK);
    assert(custom_storage_data(1,data,1)==CUSTOM_OFFSET_MISMATCH);
    for (size_t off=0;off<size;off+=40) {
        unsigned n=size-off>40 ? 40 : (unsigned)(size-off);
        assert(custom_storage_data((uint32_t)off,data+off,n)==CUSTOM_OK);
    }
    assert(custom_storage_object_end()==CUSTOM_OK);
}
static void download(const uint8_t *model,size_t size,bool finish) {
    assert(custom_storage_begin()==CUSTOM_OK);
    upload(CUSTOM_OBJ_MANIFEST,(const uint8_t *)manifest,strlen(manifest));
    upload(CUSTOM_OBJ_MODEL,model,size);
    if (finish) assert(custom_storage_finish()==CUSTOM_OK);
}
int main(void) {
    memset(flash_data,255,sizeof(flash_data));
    custom_saved_config_t saved; custom_slot_meta_t meta; int slot;
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==-1);
    custom_kws_filter_t filter={.keyword_threshold_percent=0,.cooldown_ms=1,
                                .stable_frames=1000,.need_vad_checked=1}, restored;
    assert(!custom_storage_filter_read(&restored));
    assert(custom_storage_filter_write(&filter)==ESP_OK);
    assert(custom_storage_filter_read(&restored));
    assert(restored.keyword_threshold_percent==0 && restored.cooldown_ms==1 &&
           restored.stable_frames==1000 && restored.need_vad_checked==1);
    custom_state_init(NULL,&meta,-1); custom_state_capture(&saved); assert(custom_storage_save(&saved)==ESP_OK);
    uint8_t model[5017]; for (unsigned i=0;i<sizeof(model);++i) model[i]=(uint8_t)(i*31);
    memcpy(model,"EDL2",4); custom_put_u32(model+4,0); custom_put_u32(model+8,sizeof(model)-16);
    download(model,sizeof(model),true);
    assert(custom_storage_slot_info(0,&meta));
    assert(!memcmp(flash_data[0],model,sizeof(model))); // Tail crossing the 4-KiB write boundary.
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==-1); // Download is not activation.
    assert(custom_storage_filter_read(&restored) && restored.keyword_threshold_percent==0 &&
           restored.cooldown_ms==1 && restored.stable_frames==1000 && restored.need_vad_checked==1);
    assert(custom_storage_slot_info(0,&meta)); custom_state_select(&meta,0,NULL);
    custom_state_capture(&saved); assert(custom_storage_save(&saved)==ESP_OK);
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==0 && meta.class_count==3);
    custom_state_select(&meta,slot,&saved);
    model[100]^=1; download(model,sizeof(model),false); // Power loss before valid metadata.
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==0);
    assert(!custom_storage_slot_info(1,NULL));
    download(model,sizeof(model),true);
    assert(custom_storage_slot_info(1,&meta)); custom_state_select(&meta,1,NULL);
    custom_state_capture(&saved); assert(custom_storage_save(&saved)==ESP_OK);
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==1);
    // Corrupting the new active model must select the previous valid active record.
    flash_data[1][100]^=1;
    assert(custom_storage_init(&saved,&meta,&slot)==ESP_OK && slot==0);
    custom_state_select(&meta,slot,&saved);
    assert(custom_storage_begin()==CUSTOM_OK);
    uint8_t sha[32]; sha256(manifest,strlen(manifest),sha); sha[0]^=1;
    assert(custom_storage_object_begin(CUSTOM_OBJ_MANIFEST,strlen(manifest),sha)==CUSTOM_OK);
    for (size_t off=0;off<strlen(manifest);off+=40) {
        unsigned n=strlen(manifest)-off>40 ? 40 : (unsigned)(strlen(manifest)-off);
        assert(custom_storage_data(off,(const uint8_t *)manifest+off,n)==CUSTOM_OK);
    }
    assert(custom_storage_object_end()==CUSTOM_HASH_MISMATCH);
    assert(custom_storage_finish()==CUSTOM_INVALID_STATE);
    custom_storage_abort();
    return 0;
}
