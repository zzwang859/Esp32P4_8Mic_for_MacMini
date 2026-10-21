# All replaceable model assets live in kws/model. Use a fixed embed basename so
# replacing W16A16 weights never changes the linker interface.
set(kws_model_name "ds_tc_resnet_backend_w16a16_esp32p4.espdl")
set(kws_model_dir "${COMPONENT_DIR}/kws/model")
configure_file("${kws_model_dir}/${kws_model_name}"
               "${CMAKE_CURRENT_BINARY_DIR}/kws_model.espdl" COPYONLY)
target_add_binary_data(${COMPONENT_LIB}
    "${CMAKE_CURRENT_BINARY_DIR}/kws_model.espdl" BINARY)
target_include_directories(${COMPONENT_LIB} PRIVATE "${kws_model_dir}")
