add_executable(proj_motor
        proj_motor.c
        )

# pull in common dependencies and additional pwm hardware support
target_link_libraries(proj_motor pico_stdlib hardware_pwm)

pico_enable_stdio_usb(proj_motor 1)
pico_enable_stdio_uart(proj_motor 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(proj_motor)

# add url via pico_set_program_url
example_auto_set_url(proj_motor)
