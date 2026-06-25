cmake_minimum_required(VERSION 3.13)

# Specify ARM Toolchain
set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# Include Pico SDK
find_path(PICO_SDK_PATH
    NAMES "pico_sdk_version.cmake"
    PATHS
        $ENV{HOME}
        $ENV{PICO_SDK_PATH}
    PATH_SUFFIXES
        pico/pico-sdk
        pico-sdk
)

if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "Pico SDK not found!")
endif()

set(PICO_BOARD pico2)
set(PICO_CYW43_ARCH lwip_poll)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(gripper_rp2350 C CXX)

pico_sdk_init()

add_subdirectory(lib)

add_executable(gripper
    src/main.cpp
)

target_link_libraries(gripper
    pico_stdlib
    pico_multicore
    pico_time
    hardware_adc
    hardware_pwm
    hardware_gpio
    hardware_timer
    flash
    quadrature_encoder
    motor_controller
    protocol
)

# stdio transport
pico_enable_stdio_usb(gripper 1)
pico_enable_stdio_uart(gripper 0)

# Output files (.uf2, .hex, .bin, .map, .dis)
pico_add_extra_outputs(gripper)