#pragma once

#include <algorithm>
#include <cstddef>

namespace kws_memory {

// The system reserve belongs to the whole internal heap, not to every
// contiguous block. Keep a separate, small margin for planner allocations
// made between this measurement and ESP-DL's aligned root allocation.
constexpr size_t system_reserve = 64 * 1024;
constexpr size_t block_margin = 4 * 1024;

inline size_t tensor_budget(size_t configured, size_t free_bytes, size_t largest)
{
    const size_t usable_total = free_bytes > system_reserve ? free_bytes - system_reserve : 0;
    const size_t usable_block = largest > block_margin ? largest - block_margin : 0;
    return std::min({configured, usable_total, usable_block}) & ~size_t(15);
}

} // namespace kws_memory
