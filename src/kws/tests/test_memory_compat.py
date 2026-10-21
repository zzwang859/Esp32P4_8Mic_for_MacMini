"""Exercise the actual XIP address wrapper without target MMU hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", nargs="+", required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="kws-memory-") as directory:
        folder = Path(directory)
        (folder / "dl_tool.hpp").write_text(r'''
#pragma once
#include <cstdint>
#define SOC_MEM_TCM_SUPPORTED 1
#define ESP_OK 0
using esp_paddr_t = uint32_t;
enum mmu_target_t { MMU_TARGET_PSRAM0, MMU_TARGET_FLASH0 };
namespace dl { enum memory_addr_type_t { MEMORY_ADDR_TCM, MEMORY_ADDR_INTERNAL,
    MEMORY_ADDR_PSRAM, MEMORY_ADDR_FLASH, MEMORY_ADDR_UKN }; }
bool esp_ptr_in_tcm(void *);
bool esp_ptr_internal(void *);
bool esp_ptr_external_ram(void *);
int esp_mmu_vaddr_to_paddr(void *, esp_paddr_t *, mmu_target_t *);
''', encoding="utf-8")
        (folder / "driver.cpp").write_text(r'''
#include "dl_tool.hpp"
#include <cassert>
static unsigned queries;
bool esp_ptr_in_tcm(void *p) { return (uintptr_t)p == 0x100; }
bool esp_ptr_internal(void *p) { return (uintptr_t)p == 0x200; }
bool esp_ptr_external_ram(void *p) { return (uintptr_t)p == 0x300; }
int esp_mmu_vaddr_to_paddr(void *p, esp_paddr_t *physical, mmu_target_t *target) {
    ++queries;
    *physical = 0;
    if ((uintptr_t)p == 0x400) { *target = MMU_TARGET_FLASH0; return ESP_OK; }
    if ((uintptr_t)p == 0x500) { *target = MMU_TARGET_PSRAM0; return ESP_OK; }
    return -1;
}
extern "C" dl::memory_addr_type_t __wrap__ZN2dl4tool16memory_addr_typeEPv(void *);
int main() {
    auto classify = __wrap__ZN2dl4tool16memory_addr_typeEPv;
    assert(classify((void *)0x100) == dl::MEMORY_ADDR_TCM);
    assert(classify((void *)0x200) == dl::MEMORY_ADDR_INTERNAL);
    assert(classify((void *)0x300) == dl::MEMORY_ADDR_PSRAM);
    assert(queries == 0); // SRAM must never reach the external MMU query.
    assert(classify((void *)0x400) == dl::MEMORY_ADDR_FLASH);
    assert(classify((void *)0x500) == dl::MEMORY_ADDR_PSRAM);
    assert(classify(nullptr) == dl::MEMORY_ADDR_UKN);
    assert(queries == 3);
}
''', encoding="utf-8")
        executable = folder / "memory-test.exe"
        subprocess.run(args.cxx + ["-std=c++17", "-Wall", "-Wextra", "-Werror",
                       "-I", str(folder), str(Path(__file__).parents[1] / "kws_memory_compat.cpp"),
                       str(folder / "driver.cpp"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
    print("XIP memory classification: passed")


if __name__ == "__main__":
    main()
