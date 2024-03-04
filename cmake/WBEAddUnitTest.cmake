function(wbe_add_unit_test)
    set(options )
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES LINKS)

    set(prefix "wbe_add_unit_test")

    cmake_parse_arguments(${prefix}
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN})

    set(name ${${prefix}_NAME})
    set(unit_test_files ${${prefix}_SOURCES})

    set(targetname ${name}UnitTests)
    add_executable(${targetname}
        "${unit_test_files}")

    target_link_libraries(${targetname} PRIVATE Catch2::Catch2WithMain ${${prefix}_LINKS})
    target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

    add_test(NAME ${targetname} COMMAND ${targetname})
endfunction()
