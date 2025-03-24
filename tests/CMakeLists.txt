# Copyright (c) Zubax Robotics

cmake_minimum_required(VERSION 3.22)
project(ramen_tests CXX)
enable_testing()

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/lib/doctest)

function(gen_test name files compile_flags link_flags)
    add_executable(${name} ${common_sources} ${files})
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${compile_flags}" LINK_FLAGS "${link_flags}")
    add_test("run_${name}" "${name}")
endfunction()

function(gen_test_matrix name files)
    gen_test("${name}_x64"  "${files}"  "-m64"  "-m64")
    gen_test("${name}_x32"  "${files}"  "-m32"  "-m32")
    # Coverage is only available for GCC builds.
    if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        gen_test("${name}_cov" "${files}" "-g -O0 --coverage" "--coverage")
    endif ()
endfunction()

file(GLOB test_files ${CMAKE_CURRENT_SOURCE_DIR}/*.[ch]pp)
gen_test_matrix(test "${test_files}")
