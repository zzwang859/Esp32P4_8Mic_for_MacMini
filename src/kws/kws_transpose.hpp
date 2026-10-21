#pragma once

#include <cstring>

#include "dl_module_transpose.hpp"
#include "kws_transpose_copy.h"

// Preserve ESP-DL's shape planning and general fallback. Streaming state
// tensors have only two non-singleton axes, so avoid the generic rank-N walk.
class KwsTranspose : public dl::module::Transpose {
public:
    KwsTranspose(const char *name, dl::quant_type_t type, const std::vector<int> &perm) :
        dl::module::Transpose(name, dl::MODULE_NON_INPLACE, type, perm), perm_(perm)
    {}

    void forward(dl::ModelContext *context, dl::runtime_mode_t mode) override
    {
        auto *input = context->get_tensor(m_inputs_index[0]);
        auto *output = context->get_tensor(m_outputs_index[0]);
        const auto &shape = input->get_shape();
        const int element_bytes = input->get_dtype_bytes();
        if (perm_.size() == shape.size() && (element_bytes == 1 || element_bytes == 2) &&
            shape.size() <= 32 && output->get_dtype_bytes() == element_bytes &&
            input->get_size() == output->get_size() && compact(input) && compact(output)) {
            int axes[2] = {};
            size_t count = 0;
            for (size_t i = 0; i < shape.size(); ++i) {
                if (shape[i] > 1) {
                    if (count < 2) axes[count] = static_cast<int>(i);
                    ++count;
                }
            }
            int order[2] = {};
            size_t found = 0;
            bool valid = true;
            uint32_t seen = 0;
            for (int axis : perm_) {
                if (axis < 0) axis += static_cast<int>(shape.size());
                if (axis < 0 || axis >= static_cast<int>(shape.size())) { valid = false; break; }
                const uint32_t bit = uint32_t{1} << axis;
                if (seen & bit) { valid = false; break; }
                seen |= bit;
                if (shape[axis] > 1) {
                    if (found < 2) order[found] = axis;
                    ++found;
                }
            }
            if (valid && count <= 2 && found == count) {
                auto *src = static_cast<const uint8_t *>(input->get_element_ptr());
                auto *dst = static_cast<uint8_t *>(output->get_element_ptr());
                if (count < 2 || (order[0] == axes[0] && order[1] == axes[1])) {
                    if (src != dst) std::memcpy(dst, src, input->get_bytes());
                    return;
                }
                if (order[0] == axes[1] && order[1] == axes[0] && src != dst) {
                    if (element_bytes == 1) {
                        kws_transpose_u8(dst, src, shape[axes[0]], shape[axes[1]]);
                    } else {
                        kws_transpose_u16(reinterpret_cast<uint16_t *>(dst),
                                          reinterpret_cast<const uint16_t *>(src), shape[axes[0]], shape[axes[1]]);
                    }
                    return;
                }
            }
        }
        dl::module::Transpose::forward(context, mode);
    }

    static dl::module::Module *deserialize(fbs::FbsModel *model, std::string name)
    {
        dl::quant_type_t type = dl::QUANT_TYPE_NONE;
        std::vector<int> perm;
        model->get_operation_attribute(name, "quant_type", type);
        model->get_operation_attribute(name, "perm", perm);
        return new KwsTranspose(name.c_str(), type, perm);
    }

private:
    static bool compact(dl::TensorBase *tensor)
    {
        if (tensor->shape.size() != tensor->axis_offset.size()) return false;
        int stride = 1;
        for (size_t i = tensor->shape.size(); i > 0; --i) {
            if (tensor->shape[i - 1] > 1 && tensor->axis_offset[i - 1] != stride) return false;
            stride *= tensor->shape[i - 1];
        }
        return stride == tensor->get_size();
    }

    std::vector<int> perm_;
};
