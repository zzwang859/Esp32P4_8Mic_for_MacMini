#include "kws_modules.hpp"

#include "dl_module_creator.hpp"
#include "kws_transpose.hpp"

// ESP-DL's registry includes every operator implementation. Keep that costly
// compilation separate from changes to model loading and SRAM budgeting.
void kws_register_modules()
{
    // Register defaults before overrides: an existing registry suppresses them.
    auto *creator = dl::module::ModuleCreator::get_instance();
    creator->register_dl_modules();
    creator->register_module("Transpose", KwsTranspose::deserialize);
}
