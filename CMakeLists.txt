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
components/motor_control/motor_control.c
components/ir_sensor/ir_sensor.c
components/ir_sensor/ir_line_following.c
)

target_compile_definitions(robotic_car PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )

target_include_directories(robotic_car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
        )

target_link_libraries(robotic_car 
pico_stdlib 
hardware_pwm 
hardware_adc 
hardware_gpio 
pico_cyw43_arch_lwip_sys_freertos
FreeRTOS-Kernel-Heap4
hardware_i2c
)

pico_enable_stdio_usb(robotic_car 1)
pico_enable_stdio_uart(robotic_car 1)

pico_add_extra_outputs(robotic_car) 

example_auto_set_url(robotic_car)

# car wifi cmake stuff
add_executable(robotic_car_wifi
        components/wifi/car/wifi.c
        )

target_compile_definitions(robotic_car_wifi PRIVATE
        WIFI_SSID=\"picow_p5a\"
        WIFI_PASSWORD=\"password\"
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
        hardware_i2c
        )

pico_enable_stdio_usb(robotic_car_wifi 1)
pico_enable_stdio_uart(robotic_car_wifi 1)

pico_add_extra_outputs(robotic_car_wifi)

example_auto_set_url(robotic_car_wifi)


# remote wifi cmake stuff
add_executable(remote_wifi
        components/wifi/remote/remote_wifi.c
        components/remote/remote.c
        )

target_compile_definitions(remote_wifi PRIVATE
        STATIC_IP=1    # uncomment this to disable dhcp client and use static ip addr. must be used in conjunction with PICO_ROLE
        PICO_ROLE=1    # uncomment this to set the appropriate static ip. refer to lwipopts_examples_common.h for more info
        WIFI_SSID=\"picow_p5a\"
        WIFI_PASSWORD=\"password\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )

target_include_directories(remote_wifi PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        #${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts (seems like dunnid anymore?? idk)
        )

target_link_libraries(remote_wifi
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_i2c
        )

pico_enable_stdio_usb(remote_wifi 1)
pico_enable_stdio_uart(remote_wifi 0)

pico_add_extra_outputs(remote_wifi)

example_auto_set_url(remote_wifi)

# dashboard cmake stuff
add_executable(dashboard
        components/wifi/dashboard/dashboard.c
        )

target_compile_definitions(dashboard PRIVATE
        # STATIC_IP=1    # uncomment this to disable dhcp client and use static ip addr. must be used in conjunction with PICO_ROLE
        # PICO_ROLE=1    # uncomment this to set the appropriate static ip. refer to lwipopts_examples_common.h for more info
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )

target_include_directories(dashboard PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        #${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts (seems like dunnid anymore?? idk)
        )

target_link_libraries(dashboard
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )

pico_enable_stdio_usb(dashboard 1)

pico_add_extra_outputs(dashboard)

example_auto_set_url(dashboard)

# freertos check
if (NOT FREERTOS_KERNEL_PATH AND NOT DEFINED ENV{FREERTOS_KERNEL_PATH})
    message("Skipping Pico W FreeRTOS examples as FREERTOS_KERNEL_PATH not defined")
else()
    include(FreeRTOS_Kernel_import.cmake)

endif()

# remote controller cmake stuff
add_executable(remote
        components/remote/remote.c
        )

target_link_libraries(remote pico_stdlib hardware_i2c)

# Enable USB output, disable UART output
pico_enable_stdio_usb(remote 1)
pico_enable_stdio_uart(remote 0)

pico_add_extra_outputs(remote)

example_auto_set_url(remote)
