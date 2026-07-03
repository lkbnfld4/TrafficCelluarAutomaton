cmake_minimum_required(VERSION 3.20)
project(traffic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(TRAFFIC_NATIVE "Optimize for the host CPU" ON)

function(traffic_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(TRAFFIC_NATIVE)
        target_compile_options(${target} PRIVATE -march=native)
    endif()
endfunction()

add_library(traffic_core STATIC src/traffic_sim.cpp)
target_include_directories(traffic_core PUBLIC include)
traffic_warnings(traffic_core)

if(SKBUILD)
    find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
    find_package(nanobind CONFIG REQUIRED)

    set_target_properties(traffic_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
    nanobind_add_module(core src/python_bindings.cpp)
    target_link_libraries(core PRIVATE traffic_core)
    install(TARGETS core LIBRARY DESTINATION traffic)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)

add_executable(traffic src/main.cpp)
target_link_libraries(traffic PRIVATE traffic_core argparse::argparse)
traffic_warnings(traffic)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
