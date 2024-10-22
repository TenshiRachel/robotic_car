add_executable(project_ir_contrast
project_ir_contrast.c
        )

target_link_libraries(project_ir_contrast pico_stdlib hardware_adc)

# create map/bin/hex file etc.
pico_add_extra_outputs(project_ir_contrast)

pico_enable_stdio_usb(project_ir_contrast 1)

# add url via pico_set_program_url
example_auto_set_url(project_ir_contrast)

