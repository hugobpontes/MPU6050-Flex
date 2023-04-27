add_executable(rpi_pico_example
        rpi_pico_example.c
        Mpu6050Flex.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(rpi_pico_example pico_stdlib hardware_i2c)

set_target_properties(rpi_pico_example PROPERTIES
    LANGUAGE C
)

# enable usb output, disable uart output
pico_enable_stdio_usb(rpi_pico_example 1)
pico_enable_stdio_uart(rpi_pico_example 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(rpi_pico_example)

# add url via pico_set_program_url
example_auto_set_url(rpi_pico_example)
