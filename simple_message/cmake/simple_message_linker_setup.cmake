
function(simple_message_linker_setup)
    if (DEFINED simple_message_LIBRARY_DIRS)
    else()
        message(FATAL_ERROR "simple_message_LIBRARY_DIRS not set, "
            "have you find_package()-ed it?")
    endif()
    link_directories(${simple_message_LIBRARY_DIRS})
endfunction()
