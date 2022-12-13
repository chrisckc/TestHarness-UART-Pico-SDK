# Test Harness for debugging UART communication issues between 2 Raspberry Pi Pico's (RP2040)

### This is based on the SPI master-slave example from the Pico examples Repo:
https://github.com/raspberrypi/pico-examples/tree/master/spi/spi_master_slave
and this UART example:
https://github.com/raspberrypi/pico-examples/blob/master/uart/uart_advanced/uart_advanced.c


### I have modified the way it operates to first send a separate, single byte, data transfer before the buffer is transferred. The output and input buffers have also been reduced from 256 bytes to 255.
I increased the send rate from 1 per second to 10 per second.

There is also a lot of extra serial output and error capturing and reporting has been added.

In order to properly view the serial output you will need to use a proper terminal emulator, ie. one that supports ANSI control characters. I use iTerm2 on MacOS with a command to launch screen against the usb tty.

### What it does
The Sender Pico sends a single value to the Receiver Pico via the UART, in this case the length of the buffer to be sent next.
Immediately after this it sends the 255 byte buffer, (this was 256 bytes in the original SPI master-slave example).

In response to this, the Receiver Pico, sends back it own buffer to the sender in the same way.


### The issue

There are 2 issues:

#### Issue 1:
The receiver always sees an extraneous interrupt after startup, well before any data is actually sent to it. Inside the interrupt, uart_is_readable(uart0) returns true and then uart_getc(uart0) returns a null byte. This happens regardless of baud rate or whether the FIFO is enabled or not.
This results in the receiver initially reporting an empty page being received before data is actually sent to it.
Data only starts to be sent to the receiver 1 second after it is ready and waiting to receive it (refer to setup code).

#### Issue 2:
The sender is unable to reliably receive a response from the receiver at baud rates above 115200, the higher the baud rate the higher the error rate.

At 230400 baud the error rate is around 1.1%

At 460800 baud the error rate is around 95%

At 921600 baud the error rate is 100%

The error seems to be related to the use of the USB serial during the reception of data on UART0, if is disable the part of the test code which is outputting some results from the UART0 send operation while the UART0 receive operation is in progress, the error rate drops significantly, but not completely at all supported baud rates.

For UART0 receive without any simultaneous USB serial output:

At 230400 baud the error rate is 0%

At 460800 baud the error rate is around 0%

At 921600 baud the error rate is 32%

#### Notes:

The above tested were conducted with the FIFO disabled as in the uart_advanced example.

With the FIFO enabled, issue 1 still occurs at all baud rates.

With the FIFO enabled, the error rate for issue 2 is zero for all of the above scenarios and baud rates.

Having the FIO disabled can be desirable or essential for certain use cases and there is nothing in the documentation that mentions any limits on the supported baud rates with the FIFO disabled.
