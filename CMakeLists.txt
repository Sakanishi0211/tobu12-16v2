cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(tobu C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-O 2")
#set(CMAKE_C_FLAGS "-O 2")
set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})
pico_sdk_init()


include_directories(/home/sakanishi/pico/eigen)
#include_directories(/Users/itoukouhei/pico/eigen)

add_executable(tobu
  tobu.cpp
  ekf_sensor.cpp
  lsm9ds1_reg.c
  pwm_uart.cpp)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(tobu 
  pico_stdlib 
  hardware_uart 
  hardware_spi 
  hardware_pwm 
  pico_multicore)
#target_compile_options(tobu PRIVATE -DEIGEN_NO_DEBUG)

# enable usb output, disable uart output
pico_enable_stdio_usb(tobu 1)
pico_enable_stdio_uart(tobu 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(tobu)
