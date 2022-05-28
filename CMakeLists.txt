# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/tylerm/projects/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(mat-clog C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(mat-clog mat-clog.c logger.c hw_i2c_bus.c hw_BME280.c hw_VEML6030.c hw_VL53L1X.c hw_DS1307.c hw_config.c)
add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

pico_set_program_name(mat-clog "mat-clog")
pico_set_program_version(mat-clog "0.1")

pico_enable_stdio_uart(mat-clog 1)
pico_enable_stdio_usb(mat-clog 0)

# Add the standard library to the build
target_link_libraries(mat-clog pico_stdlib)

# Add any user requested libraries
target_link_libraries(mat-clog
        hardware_spi
        hardware_i2c
        hardware_rtc
        FatFs_SPI)

pico_add_extra_outputs(mat-clog)
