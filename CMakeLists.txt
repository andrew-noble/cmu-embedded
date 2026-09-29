# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

# Keep the course's programmer choice in source control for clean F401RE builds.
if(BOARD MATCHES "^nucleo_f401re($|/)")
  set(BOARD_FLASH_RUNNER stm32cubeprogrammer CACHE STRING "F401RE flash runner")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2_sensors_actuators)

option(MOTOR_CALIBRATION "Build fixed-duty motor calibration firmware" OFF)

target_sources(app PRIVATE src/encoder.c src/motor_driver.c)
if(MOTOR_CALIBRATION)
  target_include_directories(app PRIVATE src tools/motor_calibration)
  target_sources(app PRIVATE tools/motor_calibration/motor_calibration.c)
else()
  target_sources(app PRIVATE src/main.c src/motor_control.c src/blinker.c)
endif()
