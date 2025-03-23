# Copyright (c) Zubax Robotics

cmake_minimum_required(VERSION 3.22)
project(ramen_tests CXX)
enable_testing()

# Use -DNO_STATIC_ANALYSIS=1 to suppress static analysis.
# If not suppressed, the tools used here shall be available, otherwise the build will fail.
if (NOT NO_STATIC_ANALYSIS)
    # clang-tidy (separate config files per directory)
    find_program(clang_tidy NAMES clang-tidy)
    if (NOT clang_tidy)
        message(FATAL_ERROR "Could not locate clang-tidy")
    endif ()
    message(STATUS "Using clang-tidy: ${clang_tidy}")
    set(CMAKE_CXX_CLANG_TIDY ${clang_tidy})
endif ()

# clang-format
find_program(clang_format NAMES clang-format)
if (NOT clang_format)
    message(STATUS "Could not locate clang-format")
else ()
    file(GLOB format_files ${CMAKE_SOURCE_DIR}/ramen/*.[ch]pp ${CMAKE_SOURCE_DIR}/test/*.[ch]pp)
    message(STATUS "Using clang-format: ${clang_format}; files: ${format_files}")
    add_custom_target(format COMMAND ${clang_format} -i -fallback-style=none -style=file --verbose ${format_files})
endif ()

# C++ options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic -fstrict-aliasing")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch-enum -Wundef -Wconversion -Wsign-promo")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wtype-limits -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")

include_directories(SYSTEM lib/doctest)
include_directories(ramen/)

function(gen_test name files compile_definitions compile_features compile_flags link_flags)
    add_executable(${name} ${common_sources} ${files})
    target_compile_definitions(${name} PUBLIC ${compile_definitions})
    target_compile_features(${name} PUBLIC ${compile_features})
    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${compile_flags}" LINK_FLAGS "${link_flags}")
    add_test("run_${name}" "${name}" --rng-seed time)
endfunction()

function(gen_test_matrix name files defs)
    gen_test("${name}_c99_x64"      "${files}" "${defs}"                            c_std_99 "-m64" "-m64")
    gen_test("${name}_c99_x32"      "${files}" "${defs}"                            c_std_99 "-m32" "-m32")
    gen_test("${name}_c11_x64"      "${files}" "${defs}"                            c_std_11 "-m64" "-m64")
    gen_test("${name}_c11_x32"      "${files}" "${defs}"                            c_std_11 "-m32" "-m32")
    gen_test("${name}_c11_x64_ni"   "${files}" "${defs};O1HEAP_USE_INTRINSICS=0"    c_std_11 "-m64" "-m64")
    gen_test("${name}_c11_x32_ni"   "${files}" "${defs};O1HEAP_USE_INTRINSICS=0"    c_std_11 "-m32" "-m32")
    # Coverage is only available for GCC builds.
    if ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
        gen_test("${name}_cov" "${files}" "${defs}" c_std_11 "-g -O0 --coverage" "--coverage")
    endif ()
endfunction()

gen_test_matrix(test test/detail.cpp test/public.cpp)
