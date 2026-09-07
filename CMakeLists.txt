# Copyright (c) 2018 - present, Lars Johannsmeier
# All rights reserved.
# contact: lars.johannsmeier@tum.de

cmake_minimum_required(VERSION 3.16...3.20)

set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 22)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
#set(CMAKE_FIND_DEBUG_MODE 1)

configure_file (
    "${CMAKE_SOURCE_DIR}/include/commons/config.hpp.in"
    "${CMAKE_SOURCE_DIR}/include/commons/config.hpp"
    )

project(mios VERSION ${PROJECT_VERSION}
    DESCRIPTION "Machine Intelligence Operating System (for Franka Emika Panda arms)"
    LANGUAGES CXX
    )

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE
        Release
        CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui, ccmake
    set_property(
        CACHE CMAKE_BUILD_TYPE
        PROPERTY STRINGS
        "Debug"
        "Release"
        "MinSizeRel"
        "RelWithDebInfo")
endif()


include(cmake/ApplyModuleOptions.cmake)
include(cmake/Conan.cmake)
include(cmake/Doxygen.cmake)
include(cmake/PreventInSourceBuilds.cmake)
include(cmake/StaticAnalyzers.cmake)

option(ONLY_COVERAGE "Build only tests necessary for coverage" OFF)
option(LIBCPP "Build with libc++" OFF)
option(ENABLE_COVERAGE "Enable coverage reporting for gcc/clang" OFF)
option(ENABLE_ASAN "Enable address sanitizer" OFF)
option(BUILD_SHARED_LIBS "Enable compilation of shared libraries" OFF)
option(ENABLE_TESTING "Enable the building of the test" OFF)
option(ENABLE_CLANG_TIDY "Enable testing with clang-tidy" OFF)
option(ENABLE_CPPCHECK "Enable testing with cppcheck" OFF)
option(ENABLE_INCLUDE_WHAT_YOU_USE "Enable static analysis with include-what-you-use" OFF)
option(SIMPLE_BUILD "Build the project as minimally as possible" OFF)
option(BUILD_DOC "Build the project's documentation" OFF)
option(FORCE_COLORED_OUTPUT "Always produce ANSI-colored output (GNU/Clang only)." OFF)
option(DEBUG_LOGGING "Enabling debug logging" OFF)
option(ENABLE_IPO "Enable Interprocedural Optimization, aka Link Time Optimization (LTO)" OFF)
option(ENABLE_PCH "Enable Precompiled Headers" ON)
option(ENABLE_UNITY_BUILD "Enable unity build" OFF)
option(MIOS_BUILD_LEGACY_PANDA
    "Build the legacy PandaBody/libfranka transport adapter" OFF)
option(MIOS_BUILD_LEGACY_MAIN
    "Build the legacy standalone MIOS executable" OFF)
option(MIOS_USE_SYSTEM_LIBFRANKA
    "Use an already installed libfranka package for the legacy Panda transport" OFF)
option(MIOS_BUILD_ROS2_CORE_RUNTIME
    "Build the ROS 2 executable that owns mios::Core" OFF)

if(ENABLE_TESTING)
    include(CTest)
endif()

# The legacy standalone executable ships shell/Python helpers and prebuilt
# plugins. The ROS-owned Core executable does not use them, so keep these
# file copies out of the ROS-only configuration path.
if(MIOS_BUILD_LEGACY_MAIN)
    include(cmake/CopyUtils.cmake)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/src/plugins)


add_library(${PROJECT_NAME}_project_warnings INTERFACE)
add_library(${PROJECT_NAME}_project_options INTERFACE)
add_library(${PROJECT_NAME}_project_deps INTERFACE)

target_compile_features(${PROJECT_NAME}_project_options
    INTERFACE
    cxx_std_17)

if(ENABLE_PCH)
    target_precompile_headers(
        ${PROJECT_NAME}_project_options
        INTERFACE
        ${CMAKE_SOURCE_DIR}/include/commons/pch.hpp
        )
endif()

target_compile_options(${PROJECT_NAME}_project_warnings
    INTERFACE
    -Wall
    -Wextra
    -Wshadow
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wpedantic
    -Wconversion
    -Wsign-conversion
    -Wnull-dereference
    -Wdouble-promotion
    -Wformat=2
    -Werror=return-type
    -Werror=uninitialized
    )

if(ONLY_COVERAGE OR ENABLE_COVERAGE)
    target_compile_options(${PROJECT_NAME}_project_options INTERFACE --coverage -O0 -g)
    target_link_libraries(${PROJECT_NAME}_project_options INTERFACE --coverage)
endif()

if(ENABLE_ASAN)
    target_compile_options(${PROJECT_NAME}_project_options INTERFACE -fsanitize=address)
    target_link_libraries(${PROJECT_NAME}_project_options INTERFACE -fsanitize=address)
endif()

# some GCC specific warnings. These flags are added only if the used compiler is GCC.
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_options(${PROJECT_NAME}_project_warnings
        INTERFACE
        #        -Wmisleading-indentation
        #        -Wduplicated-cond
        #        -Wlogical-op
        #        -Wuseless-cast
        )
    target_link_libraries(${PROJECT_NAME}_project_options INTERFACE stdc++fs)
endif()

if (${FORCE_COLORED_OUTPUT})
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        target_compile_options (${PROJECT_NAME}_project_options INTERFACE -fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        target_compile_options (${PROJECT_NAME}_project_options INTERFACE -fcolor-diagnostics)
    endif ()
endif ()

find_program(CCACHE ccache)
if(CCACHE)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
    set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()


include(cmake/FetchContent.cmake)


add_library(${PROJECT_NAME}::${PROJECT_NAME}_project_options ALIAS ${PROJECT_NAME}_project_options)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_project_warnings ALIAS ${PROJECT_NAME}_project_warnings)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_project_deps ALIAS ${PROJECT_NAME}_project_deps)

find_package(spdlog REQUIRED)
#find_package(Boost REQUIRED)
message("Boost Version:\t ${Boost_VERSION}")

target_link_libraries(${PROJECT_NAME}_project_deps
    INTERFACE
    spdlog::spdlog
    )

add_subdirectory(src)

message("#########################################")
    message("\t \t Summary")
    message("#########################################")
        message("Build type:       \t ${CMAKE_BUILD_TYPE}")
        message("Install prefix:   \t ${CMAKE_INSTALL_PREFIX}")
        message("Testing enabled:  \t ${ENABLE_TESTING}")
        message("Clang-tidy:       \t ${ENABLE_CLANG_TIDY}")
        message("Cppcheck:         \t ${ENABLE_CPPCHECK}")
        message("Compiler:         \t ${CMAKE_CXX_COMPILER_ID}")
        message("Sanitizers:       \t ${ENABLE_ASAN}")
        message("Shared libs:      \t ${BUILD_SHARED_LIBS}")
        message("Build libcpp:     \t ${LIBCPP}")
        message("CCache executable:\t ${CCACHE}")
        message("PCH:              \t ${ENABLE_PCH}")
        message("IPO:              \t ${ENABLE_IPO}")
        message("Unity build:      \t ${ENABLE_UNITY_BUILD}")
        message("Building doc:     \t ${BUILD_DOC}")
        message("------------------------------------------")
        message("Version:          \t ${PROJECT_VERSION}")
        message("########################################")
        #message("Boost_include_dir ${Boost_INCLUDE_DIRS}")
        #message("Boost_library_dir ${Boost_LIBRARY_DIRS}")
        #message("Boost_libraries linked ${Boost_LIBRARIES}")
