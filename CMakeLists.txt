cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

include(pico_extras_import_optional.cmake)

project(robotic_car C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

include(example_auto_set_url.cmake)

add_executable(robotic_car
main.c
components/ultrasonic/ultrasonic.c
components/wheel_encoder/wheel_encoder.c
)

target_link_libraries(robotic_car 
pico_stdlib 
hardware_pwm 
hardware_adc 
hardware_gpio 
hardware_timer
)

pico_enable_stdio_usb(robotic_car 1)
pico_enable_stdio_uart(robotic_car 1)

pico_add_extra_outputs(robotic_car) 

example_auto_set_url(robotic_car)