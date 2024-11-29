cmake_minimum_required(VERSION 3.1)

project(draft_loop_control_tests)

include_directories(../Core/Inc)
include_directories(../Core/Inc/modules)
include_directories(../Core/Inc/adapters)
include_directories(fakes/hal)

add_subdirectory(src)