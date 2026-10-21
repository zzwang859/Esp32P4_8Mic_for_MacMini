#include <cassert>
#include "kws_stream_state.hpp"
int main() {
    kws_stream::Debouncer filter;
    float logits[KWS_MAX_CLASS_COUNT]{};
    logits[33]=10;
    int detected=-1;
    for (unsigned i=0;i<kws_model::warmup_chunks;++i) detected=filter.update(33,logits,1000000,34);
    assert(detected==33);
    filter.acknowledge(true,1000000);
    assert(filter.update(33,logits,1000000,8)==-1);
    assert(filter.update(33,logits,1000000,35)==-1);
    filter=kws_stream::Debouncer();
    logits[33]=1000; logits[2]=10;
    // Values beyond the active model's class count must not affect softmax.
    for (unsigned i=0;i<kws_model::warmup_chunks;++i) detected=filter.update(2,logits,1000000,3);
    assert(detected==2);
}
