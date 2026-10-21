#include "dl_tool.hpp"

/* ESP-DL 3.3.x calls esp_mmu_vaddr_to_paddr even for internal SRAM when XIP
 * is enabled. SRAM has no external MMU mapping; the library then aborts.
 * Wrap only this symbol so managed/prebuilt AFE and KWS share the fix. */
extern "C" dl::memory_addr_type_t __wrap__ZN2dl4tool16memory_addr_typeEPv(void *address)
{
#if SOC_MEM_TCM_SUPPORTED
    if (esp_ptr_in_tcm(address)) return dl::MEMORY_ADDR_TCM;
#elif SOC_MEM_SPM_SUPPORTED
    if (esp_ptr_in_spm(address)) return dl::MEMORY_ADDR_TCM;
#endif
    if (esp_ptr_internal(address)) return dl::MEMORY_ADDR_INTERNAL;
    if (esp_ptr_external_ram(address)) return dl::MEMORY_ADDR_PSRAM;
    esp_paddr_t physical;
    mmu_target_t target;
    if (esp_mmu_vaddr_to_paddr(address, &physical, &target) == ESP_OK) {
        if (target == MMU_TARGET_PSRAM0) return dl::MEMORY_ADDR_PSRAM;
        if (target == MMU_TARGET_FLASH0) return dl::MEMORY_ADDR_FLASH;
    }
    return dl::MEMORY_ADDR_UKN;
}
