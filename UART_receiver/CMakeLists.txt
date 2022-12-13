if (TARGET tinyusb_device)
        add_executable(UART_receiver
                UART_receiver.c
                )

        # Pull in basic dependencies
        target_link_libraries(UART_receiver pico_stdlib hardware_uart)

        # enable usb output, disable uart output
        pico_enable_stdio_usb(UART_receiver 1)
        pico_enable_stdio_uart(UART_receiver 0)

        # create map/bin/hex file etc.
        pico_add_extra_outputs(UART_receiver)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

