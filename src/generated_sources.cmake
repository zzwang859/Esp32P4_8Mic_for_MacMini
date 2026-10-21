# Preserve timestamps so an unrelated CMake reconfigure does not rebuild the
# patched dependencies and regenerate the application's linker script.
function(write_generated_source output content)
    if(EXISTS "${output}")
        file(READ "${output}" previous_content)
        if("${previous_content}" STREQUAL "${content}")
            return()
        endif()
    endif()
    file(WRITE "${output}" "${content}")
endfunction()
