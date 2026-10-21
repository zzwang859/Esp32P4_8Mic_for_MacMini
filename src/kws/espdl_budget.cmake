# ESP-DL's constructor normally receives its tensor budget before loading the
# graph. Loading creates modules/maps and can split the measured SRAM block.
# A task-scoped KWS request selects our late callback and reserves the arena before
# planning itself can fragment the block. root_alloc reuses that owned arena.
# Keep the managed component untouched and reject incompatible source updates.
idf_component_get_property(kws_espdl_lib espressif__esp-dl COMPONENT_LIB)
get_target_property(kws_espdl_sources ${kws_espdl_lib} SOURCES)
get_target_property(kws_espdl_source_dir ${kws_espdl_lib} SOURCE_DIR)
set(kws_espdl_patched_sources "")
set(kws_espdl_budget_patched FALSE)
set(kws_espdl_context_patched FALSE)
foreach(kws_espdl_source IN LISTS kws_espdl_sources)
    if(kws_espdl_source MATCHES "(^|/)dl/model/src/dl_(memory_manager_greedy|model_context)[.]cpp$")
        if(IS_ABSOLUTE "${kws_espdl_source}")
            set(kws_espdl_original "${kws_espdl_source}")
        else()
            set(kws_espdl_original "${kws_espdl_source_dir}/${kws_espdl_source}")
        endif()
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${kws_espdl_original}")
        file(READ "${kws_espdl_original}" kws_espdl_text)
        if(kws_espdl_source MATCHES "dl_memory_manager_greedy[.]cpp$")
            set(kws_espdl_old "    get_tensor_info_from_fbs(fbs_model, execution_plan, context, tensor_info, input_shapes);")
            set(kws_espdl_new [=[    get_tensor_info_from_fbs(fbs_model, execution_plan, context, tensor_info, input_shapes);

    if (kws_model_tensor_arena_requested()) {
        this->max_internal_size = kws_model_tensor_budget();
        // Own the SRAM block now: simulation's temporary allocations must
        // not steal it between measuring the heap and allocating the root.
        if (!context->root_alloc(this->max_internal_size, 0, this->alignment)) {
            ESP_LOGW(TAG, "KWS SRAM arena unavailable; planning in PSRAM");
            this->max_internal_size = 0;
        }
    }]=])
            set(kws_espdl_budget_patched TRUE)
        else()
            set(kws_espdl_old "    m_internal_size = internal_size;")
            set(kws_espdl_new [=[    if (m_internal_root && (internal_size != size_t(m_internal_size) || m_psram_root)) {
        ESP_LOGE(TAG, "Cannot resize a reserved tensor root");
        return false;
    }
    m_internal_size = internal_size;]=])
            set(kws_espdl_context_patched TRUE)
        endif()
        string(FIND "${kws_espdl_text}" "${kws_espdl_old}" kws_espdl_pos)
        if(kws_espdl_pos EQUAL -1)
            message(FATAL_ERROR "ESP-DL allocation source changed; update the KWS SRAM arena patch")
        endif()
        string(REPLACE "${kws_espdl_old}" "${kws_espdl_new}" kws_espdl_text "${kws_espdl_text}")
        if(kws_espdl_source MATCHES "dl_model_context[.]cpp$")
            set(kws_espdl_old "    if (m_internal_size > 0) {")
            string(FIND "${kws_espdl_text}" "${kws_espdl_old}" kws_espdl_pos)
            if(kws_espdl_pos EQUAL -1)
                message(FATAL_ERROR "ESP-DL root allocation changed; update the KWS SRAM arena patch")
            endif()
            string(REPLACE "${kws_espdl_old}" "    if (m_internal_size > 0 && !m_internal_root) {"
                kws_espdl_text "${kws_espdl_text}")
        endif()
        get_filename_component(kws_espdl_basename "${kws_espdl_source}" NAME)
        set(kws_espdl_generated "${CMAKE_CURRENT_BINARY_DIR}/kws_${kws_espdl_basename}")
        write_generated_source("${kws_espdl_generated}" "extern \"C\" bool kws_model_tensor_arena_requested(void);\nextern \"C\" int kws_model_tensor_budget(void);\n${kws_espdl_text}")
        list(APPEND kws_espdl_patched_sources "${kws_espdl_generated}")
    else()
        list(APPEND kws_espdl_patched_sources "${kws_espdl_source}")
    endif()
endforeach()
if(NOT kws_espdl_budget_patched OR NOT kws_espdl_context_patched)
    message(FATAL_ERROR "ESP-DL model source not found for the KWS late tensor-budget patch")
endif()
set_property(TARGET ${kws_espdl_lib} PROPERTY SOURCES "${kws_espdl_patched_sources}")
