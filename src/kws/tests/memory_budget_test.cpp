#include "kws_memory_budget.hpp"
#include <cassert>

int main()
{
    using kws_memory::tensor_budget;
    // Reproduce the board's post-reload heap: fragmented, but enough total
    // SRAM and one block large enough for this model's 57,904-byte arena.
    assert(tensor_budget(256 * 1024, 262259, 63488) >= 57904);
    assert(tensor_budget(256 * 1024, 262259, 63488) <= 63488 - 4096);
    // A genuine shortage must still preserve system headroom, even when
    // there is a large contiguous block. Configuration zero disables SRAM.
    assert(tensor_budget(256 * 1024, 65536, 60000) == 0);
    assert(tensor_budget(256 * 1024, 65536 + 1001, 60000) == 992);
    assert(tensor_budget(0, 262259, 63488) == 0);
    assert(tensor_budget(32768, 262259, 63488) == 32768);
    assert(tensor_budget(256 * 1024, 262259, 4096) == 0);
    assert(tensor_budget(256 * 1024, 0, 0) == 0);
}
