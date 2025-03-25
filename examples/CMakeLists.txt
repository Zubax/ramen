# Copyright (c) Zubax Robotics

cmake_minimum_required(VERSION 3.22)
project(ramen_examples CXX)
enable_testing()

file(GLOB example_files "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
foreach(src ${example_files})
    get_filename_component(exe_name ${src} NAME_WE)
    add_executable(${exe_name} ${src})
    add_test(NAME run_${exe_name} COMMAND ${exe_name})
endforeach()
