#include "custom_protocol.h"
#include <assert.h>
#include <string.h>

int main(void) {
    assert(custom_crc32("123456789",9)==0xcbf43926U);
    custom_frame_t request={.type=CUSTOM_REQUEST,.command=CUSTOM_OBJECT_DATA,.sequence=0xfffe,
        .transaction=0x12345678,.offset=4090,.length=40};
    for (unsigned i=0;i<40;++i) request.payload[i]=(uint8_t)(i*7);
    uint8_t wire[64]; custom_frame_encode(wire,&request);
    assert(!memcmp(wire,"KC\x01\x00",4));
    assert(wire[8]==0x78 && wire[9]==0x56 && wire[10]==0x34 && wire[11]==0x12);
    custom_frame_t decoded;
    assert(custom_frame_decode(&decoded,wire,64)==CUSTOM_OK);
    assert(decoded.offset==4090 && decoded.length==40 && !memcmp(decoded.payload,request.payload,40));
    for (unsigned i=0;i<60;++i) {
        uint8_t old=wire[i]; wire[i]^=0x40;
        assert(custom_frame_decode(&decoded,wire,64)!=CUSTOM_OK); wire[i]=old;
    }
    assert(custom_frame_decode(&decoded,wire,63)==CUSTOM_BAD_LENGTH);
    custom_frame_encode(wire,&(custom_frame_t){.command=CUSTOM_SET_BINDING});
    wire[59]=1; custom_put_u32(wire+60,custom_crc32(wire,60));
    assert(custom_frame_decode(&decoded,wire,64)==CUSTOM_BAD_LENGTH);
    wire[59]=0; wire[3]=CUSTOM_RESPONSE; custom_put_u32(wire+60,custom_crc32(wire,60));
    assert(custom_frame_decode(&decoded,wire,64)==CUSTOM_BAD_LENGTH);
    assert(custom_utf8_valid((const uint8_t *)"\xe5\xbc\x80\xe7\x81\xaf",6));
    assert(custom_utf8_valid((const uint8_t *)"\xf0\x9f\x92\xa1",4));
    assert(!custom_utf8_valid((const uint8_t *)"\xed\xa0\x80",3));
    assert(!custom_utf8_valid((const uint8_t *)"\xc0\xaf",2));
    assert(!custom_utf8_valid((const uint8_t *)"\xf4\x90\x80\x80",4));
    assert(!custom_utf8_valid((const uint8_t *)"\xe5\xbc",2));
    return 0;
}
