#include "kws_port.h"
#include "custom_control.h"
#include "esp_partition.h"
#include "kws_frontend.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iterator>
#include <new>
#include <vector>

#include "dl_model_base.hpp"
#include "kws_modules.hpp"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "kws_memory_budget.hpp"
#include "model_config.hpp"

namespace {

constexpr const char *kTag = "kws_model";
constexpr int kBackendBits = 16;
std::atomic<TaskHandle_t> arena_requester{nullptr};
extern const uint8_t kws_model_start[] asm("_binary_kws_model_espdl_start");

// The neural backend has seven inputs: MFCC plus six feedback states.
// onnx_next_state_6:0 is produced but its matching length-one input was removed.
constexpr auto &kStateInputs = kws_model::state_inputs;
constexpr auto &kStateOutputs = kws_model::state_outputs;
static_assert(std::size(kStateInputs) == std::size(kStateOutputs));

struct KwsContext {
    // Keep MFCC history/FFT in internal SRAM regardless of malloc's PSRAM
    // size threshold. All existing delete/error paths use the matching free.
    static void *operator new(size_t bytes, const std::nothrow_t &) noexcept {
        return heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    static void operator delete(void *ptr) noexcept { heap_caps_free(ptr); }
    static void operator delete(void *ptr, const std::nothrow_t &) noexcept { heap_caps_free(ptr); }

    void *model_blob = nullptr;
    unsigned class_count = 0;
    dl::Model *model = nullptr;
    kws_frontend_t frontend = {};
    dl::TensorBase *features = nullptr;
    dl::TensorBase *logits = nullptr;
    dl::TensorBase *state_inputs[std::size(kStateInputs)] = {};
    dl::TensorBase *state_outputs[std::size(kStateOutputs)] = {};
};

const char *memory_region(const void *p)
{
    if (esp_ptr_external_ram(p)) return "PSRAM";
    if (esp_ptr_internal(p)) return "SRAM";
    return "other";
}

void promote_parameters(dl::Model *model)
{
    auto *fbs = model->get_fbs_model();
    if (!fbs) return;
    fbs->load_map();
    size_t copied = 0;
    for (const auto &node : fbs->topological_sort()) {
        std::vector<std::string> inputs, outputs;
        fbs->get_operation_inputs_and_outputs(node, inputs, outputs);
        for (const auto &name : inputs) {
            if (name.empty() || !fbs->is_parameter(name)) continue;
            auto *tensor = model->get_intermediate(name);
            if (!tensor || !esp_ptr_external_ram(tensor->get_element_ptr())) continue;
            const size_t bytes = tensor->get_bytes();
            // Preserve the same startup reserve as activation allocation.
            if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < bytes + 65536) continue;
            void *data = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_SIMD);
            if (!data) continue;
            void *old = tensor->get_element_ptr();
            std::memcpy(data, old, bytes);
            tensor->set_element_ptr(data);
            // TensorBase now owns the aligned SRAM copy and releases it
            // with the model. Shared initializers already promoted are skipped.
            if (tensor->auto_free) heap_caps_free(old);
            tensor->auto_free = true;
            copied += bytes;
        }
    }
    fbs->clear_map();
    ESP_LOGI(kTag, "parameter promotion: %u bytes PSRAM -> SRAM", (unsigned)copied);
}

}  // namespace

// Scope the project-local planner path to the task constructing this KWS model.
extern "C" bool kws_model_tensor_arena_requested(void)
{
    return arena_requester.load(std::memory_order_acquire) == xTaskGetCurrentTaskHandle();
}

// Called by the project-local ESP-DL planner patch only for this KWS model.
// Graph/modules/tensor metadata exist here; the arena is reserved immediately.
extern "C" int kws_model_tensor_budget(void)
{
    // Audio/USB are already running. Preserve system headroom across the
    // whole heap, independently of the limit imposed by its largest block.
    const size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    const size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_SIMD);
    const int kInternalTensorBytes = static_cast<int>(kws_memory::tensor_budget(
        size_t(CONFIG_KWS_INTERNAL_TENSOR_KB) * 1024, free_bytes, largest));
    ESP_LOGI(kTag, "heap before tensor plan: internal_free=%u largest=%u psram_free=%u bytes",
             (unsigned)free_bytes,
             (unsigned)largest, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(kTag, "tensor internal SRAM budget=%d bytes (configured max=%d KiB)",
             kInternalTensorBytes, CONFIG_KWS_INTERNAL_TENSOR_KB);
    return kInternalTensorBytes;
}

extern "C" esp_err_t kws_create(kws_handle_t *handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    *handle = nullptr;
    auto *ctx = new (std::nothrow) KwsContext();
    if (!ctx) return ESP_ERR_NO_MEM;
    kws_register_modules();
    ESP_LOGI(kTag, "MFCC context=%u bytes region=%s history=%s fft=%s malloc_threshold=%d forced_internal=1",
             (unsigned)sizeof(*ctx), memory_region(ctx), memory_region(ctx->frontend.history),
             memory_region(ctx->frontend.fft), CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL);
    ctx->class_count = custom_class_count();
    const char *partition = custom_model_partition();
    const char *model_data=reinterpret_cast<const char *>(kws_model_start);
    if (partition) {
        const esp_partition_t *part=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_ANY,partition);
        uint8_t header[16];
        if (!part || esp_partition_read(part,0,header,sizeof(header))!=ESP_OK) { delete ctx; return ESP_FAIL; }
        size_t bytes=16U+uint32_t(header[8])+(uint32_t(header[9])<<8)+(uint32_t(header[10])<<16)+(uint32_t(header[11])<<24);
        if (bytes>part->size || bytes<32) { delete ctx; return ESP_FAIL; }
        ctx->model_blob=heap_caps_aligned_alloc(16,bytes,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
        if (!ctx->model_blob) { delete ctx; return ESP_ERR_NO_MEM; }
        if (esp_partition_read(part,0,ctx->model_blob,bytes)!=ESP_OK) { heap_caps_free(ctx->model_blob); delete ctx; return ESP_FAIL; }
        model_data=static_cast<const char *>(ctx->model_blob);
    }
    // No persistent flash mmap: PSRAM XIP must keep USB running during erase.
    // Only this task's constructor may use the reserved SRAM arena path.
    arena_requester.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
    ctx->model = new dl::Model(model_data,fbs::MODEL_LOCATION_IN_FLASH_RODATA,0);
    arena_requester.store(nullptr, std::memory_order_release);
    if (ctx->model) promote_parameters(ctx->model);
    ctx->features = ctx->model ? ctx->model->get_input(kws_model::feature_input) : nullptr;
    ctx->logits = ctx->model ? ctx->model->get_output(kws_model::logits_output) : nullptr;
    constexpr auto expected_dtype = dl::DATA_TYPE_INT16;
    if (!ctx->features || !ctx->logits || ctx->features->get_size() != KWS_FEATURE_COUNT ||
        ctx->logits->get_size() != ctx->class_count ||
        ctx->features->get_dtype() != expected_dtype || ctx->logits->get_dtype() != expected_dtype ||
        !ctx->features->get_element_ptr() || !ctx->logits->get_element_ptr()) {
        ESP_LOGE(kTag, "ESP-DL model inputs/outputs unavailable");
        delete ctx->model;
        heap_caps_free(ctx->model_blob);
        delete ctx;
        return ESP_FAIL;
    }
    for (size_t i = 0; i < std::size(kStateInputs); ++i) {
        ctx->state_inputs[i] = ctx->model->get_input(kStateInputs[i]);
        ctx->state_outputs[i] = ctx->model->get_output(kStateOutputs[i]);
        if (!ctx->state_inputs[i] || !ctx->state_outputs[i] ||
            ctx->state_inputs[i]->get_size() != ctx->state_outputs[i]->get_size() ||
            ctx->state_inputs[i]->get_dtype() != expected_dtype ||
            ctx->state_outputs[i]->get_dtype() != expected_dtype ||
            !ctx->state_inputs[i]->get_element_ptr() || !ctx->state_outputs[i]->get_element_ptr()) {
            ESP_LOGE(kTag, "Missing state pair %u", static_cast<unsigned>(i));
            delete ctx->model;
            heap_caps_free(ctx->model_blob);
            delete ctx;
            return ESP_FAIL;
        }
    }
    // Query once after allocation; never call profile_memory(), which also
    // writes directly to stdout. These lines use the existing UART queue.
    const auto memory = ctx->model->get_memory_info();
    for (const char *name : {"variable", "parameter", "parameter_copy"}) {
        const auto &allocation = memory.at(name);
        ESP_LOGI(kTag, "model %s: SRAM=%u PSRAM=%u flash=%u bytes", name,
                 (unsigned)allocation.internal, (unsigned)allocation.psram, (unsigned)allocation.flash);
    }
    unsigned internal_states = 0, external_states = 0;
    for (size_t i = 0; i < std::size(kStateInputs); ++i) {
        for (auto *tensor : {ctx->state_inputs[i], ctx->state_outputs[i]}) {
            internal_states += esp_ptr_internal(tensor->get_element_ptr());
            external_states += esp_ptr_external_ram(tensor->get_element_ptr());
        }
    }
    ESP_LOGI(kTag, "feedback state tensors: SRAM=%u PSRAM=%u total=%u",
             internal_states, external_states, unsigned(2 * std::size(kStateInputs)));
    ESP_LOGI(kTag, "tensor regions: MFCC_input=%s logits=%s internal_free_after=%u largest_after=%u bytes",
             memory_region(ctx->features->get_element_ptr()), memory_region(ctx->logits->get_element_ptr()),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    const esp_err_t err = kws_reset(ctx);
    if (err != ESP_OK) {
        delete ctx->model;
        heap_caps_free(ctx->model_blob);
        delete ctx;
        return err;
    }
    *handle = ctx;
    ESP_LOGI(kTag, "KWS model ready: FP32 audio frontend, INT%d backend, logits and %u state pairs", kBackendBits,
             static_cast<unsigned>(std::size(kStateInputs)));
    return ESP_OK;
}

extern "C" esp_err_t kws_reset(kws_handle_t handle)
{
    if (!handle) return ESP_ERR_INVALID_ARG;
    auto *ctx = static_cast<KwsContext *>(handle);
    kws_frontend_reset(&ctx->frontend);
    for (auto *input : ctx->state_inputs) {
        // All feedback inputs use symmetric quantization: real zero is byte zero.
        std::memset(input->get_element_ptr(), 0, input->get_bytes());
    }
    return ESP_OK;
}

extern "C" esp_err_t kws_process(kws_handle_t handle, const int32_t *pcm, size_t samples,
                                   float *logits, int *class_index)
{
    if (!handle || !pcm || !logits || !class_index || samples != KWS_CHUNK_SAMPLES) {
        return ESP_ERR_INVALID_ARG;
    }
    auto *ctx = static_cast<KwsContext *>(handle);
    float features[KWS_FEATURE_COUNT];
    kws_frontend_process(&ctx->frontend, pcm, features);
    if (!ctx->features->assign(
            {1, 1, KWS_FEATURE_COUNT}, features, 0, dl::DATA_TYPE_FLOAT)) {
        ESP_LOGE(kTag, "Failed to quantize MFCC input");
        return ESP_FAIL;
    }

    ctx->model->run(dl::RUNTIME_MODE_SINGLE_CORE);
    dl::TensorBase float_logits({1, static_cast<int>(ctx->class_count)}, logits, 0, dl::DATA_TYPE_FLOAT, false);
    if (!float_logits.assign(ctx->logits)) {
        ESP_LOGE(kTag, "Failed to dequantize logits");
        return ESP_FAIL;
    }
    for (size_t i = 0; i < ctx->class_count; ++i) {
        if (!std::isfinite(logits[i])) return ESP_FAIL;
    }
    for (size_t i = 0; i < std::size(kStateInputs); ++i) {
        if (!ctx->state_inputs[i]->assign(ctx->state_outputs[i])) {
            ESP_LOGE(kTag, "Failed to update state %u", static_cast<unsigned>(i));
            return ESP_FAIL;
        }
    }
    *class_index = static_cast<int>(std::max_element(logits, logits + ctx->class_count) - logits);
    return ESP_OK;
}

extern "C" void kws_destroy(kws_handle_t *handle)
{
    if (!handle || !*handle) return;
    auto *ctx = static_cast<KwsContext *>(*handle);
    delete ctx->model;
    heap_caps_free(ctx->model_blob);
    delete ctx;
    *handle = nullptr;
}
