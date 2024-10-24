cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

include(pico_extras_import_optional.cmake)

project(robotic_car C CXX ASM)
project(robotic_car_wifi C CXX ASM)

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
)

pico_enable_stdio_usb(robotic_car 1)
pico_enable_stdio_uart(robotic_car 1)

pico_add_extra_outputs(robotic_car) 

example_auto_set_url(robotic_car)

add_subdirectory(components)

# wifi cmake stuff
add_executable(robotic_car_wifi
        components/wifi/wifi.c
        )

target_compile_definitions(robotic_car_wifi PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )

target_include_directories(robotic_car_wifi PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )

target_link_libraries(robotic_car_wifi
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )

pico_enable_stdio_usb(robotic_car_wifi 1)
pico_enable_stdio_uart(robotic_car_wifi 1)

pico_add_extra_outputs(robotic_car_wifi)

example_auto_set_url(robotic_car_wifi)


# freertos check
if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping Pico W FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
else()
    include(FreeRTOS_Kernel_import.cmake)

endif()
