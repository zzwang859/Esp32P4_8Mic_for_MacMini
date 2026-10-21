# Use the same calibration source for firmware and native DSP verification.
set(array_gain_generator "${CMAKE_CURRENT_LIST_DIR}/generate_gain_calibration.py")
get_filename_component(array_gain_json
    "${CMAKE_CURRENT_LIST_DIR}/gain_cali.json" ABSOLUTE)

function(generate_array_gain_header python output_directory)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${array_gain_json}" "${array_gain_generator}")
    execute_process(
        COMMAND "${python}" "${array_gain_generator}" "${array_gain_json}"
            "${output_directory}/input_gain_calibration.hpp"
        RESULT_VARIABLE result
        ERROR_VARIABLE error)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot generate array input gains: ${error}")
    endif()
endfunction()
