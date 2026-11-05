cmake_minimum_required(VERSION 3.13)

project(firmware-native C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FIRMWARE ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware)
set(LIBOPENCM3_INCLUDE "$ENV{HOME}/.platformio/packages/framework-libopencm3/include"
    CACHE PATH "The include directory of libopencm3, for the headers the USB stack needs")

if(NOT EXISTS "${LIBOPENCM3_INCLUDE}/libopencm3/cm3/common.h")
    message(FATAL_ERROR "libopencm3 not found. Build the firmware once with PlatformIO, or set LIBOPENCM3_INCLUDE.")
endif()

add_compile_definitions(STM32F0 QSB_FSDEV_DBL_BUF)

# The firmware is written for a 32-bit MCU, so a host compiler has plenty to say about it, and it
# casts pointers to uint32_t for the DMA registers, which needs -fpermissive on a 64-bit host.
# gcc only accepts the USB stack's constants as case labels when optimizing.
add_compile_options(-w -O1 $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>)

enable_testing()

function(add_firmware_test name source)
    add_executable(${name} ${source} ${ARGN})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# Tests that run uart.cpp on stubs of the libopencm3 functions it calls
set(UART_INCLUDES stubs ${FIRMWARE}/include ${FIRMWARE}/src)

add_firmware_test(uart_rx_overrun uart_rx_overrun.cpp)
target_include_directories(uart_rx_overrun PRIVATE ${UART_INCLUDES})

add_firmware_test(uart_line_coding uart_line_coding.cpp)
target_include_directories(uart_line_coding PRIVATE ${UART_INCLUDES})

add_firmware_test(usb_serial_state usb_serial_state.cpp)
target_include_directories(usb_serial_state PRIVATE ${UART_INCLUDES} ${FIRMWARE}/lib/qsb ${LIBOPENCM3_INCLUDE})

# Tests of the USB stack, which only needs libopencm3's headers
add_firmware_test(qsb_driver_halt qsb_driver_halt.cpp)
target_include_directories(qsb_driver_halt PRIVATE ${FIRMWARE}/lib/qsb ${LIBOPENCM3_INCLUDE})

add_firmware_test(qsb_standard_requests qsb_standard_requests.c ${FIRMWARE}/lib/qsb/qsb_standard.c)
target_include_directories(qsb_standard_requests PRIVATE ${FIRMWARE}/lib/qsb ${LIBOPENCM3_INCLUDE})
