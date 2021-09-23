cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(bootloader main.c )

pico_add_extra_outputs(bootloader)
pico_set_binary_type(bootloader copy_to_ram)

set_target_properties(bootloader PROPERTIES COMPILE_FLAGS "-Wall")

target_link_libraries(bootloader pico_stdlib pico_stdio_usb)
