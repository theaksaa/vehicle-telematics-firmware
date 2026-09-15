cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(vehicle_telematics_firmware)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
    src/main.c

    src/gnss/gnss_service.c

    src/shell/app_shell.c
    src/shell/gps_shell.c
)