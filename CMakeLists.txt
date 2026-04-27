cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(boris_apartment_plugins)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_CLANG_TIDY "Run clang-tidy on every compiled file" OFF)
option(ENABLE_CPPCHECK    "Run cppcheck on every compiled file"   OFF)
option(ENABLE_ASAN  "Enable AddressSanitizer (memory errors)"      OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer"            ON)
option(ENABLE_TSAN  "Enable ThreadSanitizer (race conditions)"     OFF)

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
endif()

if(ENABLE_CPPCHECK)
    find_program(CPPCHECK_EXE NAMES cppcheck REQUIRED)
    set(CMAKE_CXX_CPPCHECK
        "${CPPCHECK_EXE}"
        "--enable=all"
        "--suppress=missingIncludeSystem"
        "--suppress=unusedFunction:script/logging.hh"
        "--inline-suppr"
        "--error-exitcode=1")
endif()

if(ENABLE_ASAN AND ENABLE_TSAN)
    message(FATAL_ERROR "ASan and TSan are mutually exclusive — enable only one.")
endif()

if(ENABLE_ASAN)
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
endif()

if(ENABLE_UBSAN)
    add_compile_options(-fsanitize=undefined)
    add_link_options(-fsanitize=undefined)
endif()

if(ENABLE_TSAN)
    add_compile_options(-fsanitize=thread)
    add_link_options(-fsanitize=thread)
endif()

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(hello_world SHARED script/hello_world.cpp)

target_include_directories(hello_world PRIVATE ${CMAKE_SOURCE_DIR}/script)

target_link_libraries(hello_world
    gz-sim8::gz-sim8
    gz-plugin2::register)

add_library(robot_plugin SHARED script/robot/robot_plugin.cpp)

target_include_directories(robot_plugin PRIVATE ${CMAKE_SOURCE_DIR}/script)

target_link_libraries(robot_plugin
    gz-sim8::gz-sim8
    gz-plugin2::register)
