# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(microrosapp C CXX)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

zephyr_get_include_directories_for_lang_as_string(       C includes)
zephyr_get_system_include_directories_for_lang_as_string(C system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C definitions)
zephyr_get_compile_options_for_lang_as_string(           C options)

zephyr_get_include_directories_for_lang_as_string(       CXX includes_cxx)
zephyr_get_system_include_directories_for_lang_as_string(CXX system_includes_cxx)
zephyr_get_compile_definitions_for_lang_as_string(       CXX definitions_cxx)
zephyr_get_compile_options_for_lang_as_string(           CXX options_cxx)

# message(WARNING ${system_includes})
# message(FATAL_ERROR ${system_includes_cxx})

set(external_project_cflags
    "${includes} ${definitions} ${options} ${system_includes}"
    )

set(external_project_cxxflags
    "${includes_cxx} ${definitions_cxx} ${options_cxx} ${system_includes_cxx}"
    )

include(ExternalProject)

set(mylib_src_dir   ${CMAKE_CURRENT_SOURCE_DIR}/microros)
set(mylib_build_dir ${CMAKE_CURRENT_BINARY_DIR}/microros)

set(MYLIB_LIB_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/../mcu_ws/install)
set(MYLIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../mcu_ws/install/include)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
set(submake "$(MAKE)")
else()
set(submake "make")
endif()

ExternalProject_Add(
    mylib_project                 # Name for custom target
    PREFIX     ${mylib_build_dir} # Root dir for entire project
    SOURCE_DIR ${mylib_src_dir}
    BINARY_DIR ${mylib_src_dir} # This particular build system is invoked from the root
    CONFIGURE_COMMAND ""    # Skip configuring the project, e.g. with autoconf
    BUILD_COMMAND
        ${submake}
            PREFIX=${mylib_build_dir}
            CC=${CMAKE_C_COMPILER}
            AR=${CMAKE_AR}
            CFLAGS=${external_project_cflags}
            CXX=${CMAKE_CXX_COMPILER}
            CXXFLAGS=${external_project_cxxflags}

    INSTALL_COMMAND ""      # This particular build system has no install command
    BUILD_BYPRODUCTS ${MYLIB_LIB_DIR}/libmicroros.a
    )


add_library(mylib_lib STATIC IMPORTED GLOBAL)
add_dependencies(
    mylib_lib
    mylib_project
    )

add_dependencies(mylib_project offsets_h)

set_target_properties(mylib_lib PROPERTIES IMPORTED_LOCATION             ${MYLIB_LIB_DIR}/libmicroros.a)
set_target_properties(mylib_lib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${MYLIB_INCLUDE_DIR})

target_link_libraries(app PUBLIC mylib_lib)

