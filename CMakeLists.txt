# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

# Keep the course's programmer choice in source control for clean F401RE builds.
if(BOARD MATCHES "^nucleo_f401re($|/)")
  set(BOARD_FLASH_RUNNER stm32cubeprogrammer CACHE STRING "F401RE flash runner")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2_sensors_actuators)

option(MOTOR_CALIBRATION "Build fixed-duty motor calibration firmware" OFF)
option(SERVO_CALIBRATION "Build manual servo calibration firmware" OFF)

if(MOTOR_CALIBRATION AND SERVO_CALIBRATION)
  message(FATAL_ERROR "Select only one calibration firmware")
endif()

target_sources(app PRIVATE src/motor_driver.c)
if(SERVO_CALIBRATION)
  target_include_directories(app PRIVATE src tools/servo_calibration)
  target_sources(app PRIVATE src/servo_driver.c tools/servo_calibration/servo_calibration.c)
elseif(MOTOR_CALIBRATION)
  target_include_directories(app PRIVATE src tools/motor_calibration)
  target_sources(app PRIVATE src/encoder.c tools/motor_calibration/motor_calibration.c)
else()
  target_sources(app PRIVATE src/encoder.c src/main.c src/motor_control.c src/blinker.c
                             src/status_tx.c src/servo_driver.c)
endif()
