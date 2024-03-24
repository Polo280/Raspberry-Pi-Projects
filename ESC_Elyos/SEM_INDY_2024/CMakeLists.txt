cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(myproject_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(bldcDriver
    BL_DRIVER.c
    BL_DRIVER.h
)

target_link_libraries(bldcDriver 
    pico_stdlib 
    hardware_pwm 
    hardware_adc 
    hardware_irq 
    hardware_gpio
    pico_time)

pico_enable_stdio_usb(bldcDriver 1)		# Enable USB printf output
pico_enable_stdio_uart(bldcDriver 0)	# Disable serial printf output

pico_add_extra_outputs(bldcDriver)

