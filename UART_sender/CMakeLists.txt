if (TARGET tinyusb_device)
        add_executable(UART_sender
                UART_sender.c
                )

        # Pull in basic dependencies
        target_link_libraries(UART_sender pico_stdlib hardware_uart)

            # enable usb output, disable uart output
        pico_enable_stdio_usb(UART_sender 1)
        pico_enable_stdio_uart(UART_sender 0)

        # create map/bin/hex file etc.
        pico_add_extra_outputs(UART_sender)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
