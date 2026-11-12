# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Mikrobus_Connection C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(Mikrobus_Connection
        Mikrobus_Connection.c
        lcd/lcd.c
        telemetry/telemetry.c
        webpage/webpage.c
        lib/dhcpserver/dhcpserver.c
        lib/dnsserver/dnsserver.c
)

pico_set_program_name(Mikrobus_Connection "Mikrobus_Connection")
pico_set_program_version(Mikrobus_Connection "0.1")

# UART off, USB serial on
pico_enable_stdio_uart(Mikrobus_Connection 0)
pico_enable_stdio_usb(Mikrobus_Connection 1)

target_include_directories(Mikrobus_Connection PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}            # lwipopts.h, htmlfile.h
        ${CMAKE_CURRENT_LIST_DIR}/lcd
        ${CMAKE_CURRENT_LIST_DIR}/telemetry
        ${CMAKE_CURRENT_LIST_DIR}/lib/dhcpserver
        ${CMAKE_CURRENT_LIST_DIR}/lib/dnsserver
        ${CMAKE_CURRENT_LIST_DIR}/webpage
)

target_link_libraries(Mikrobus_Connection
        pico_stdlib
        pico_multicore
        pico_low_power
        hardware_uart
        hardware_spi
        hardware_gpio
        hardware_dma
        hardware_irq
        pico_cyw43_arch_lwip_poll
)

pico_add_extra_outputs(Mikrobus_Connection)