cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/../../modules/vl53l1x")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(measuring_tape)

target_sources(app PRIVATE src/main.c) 