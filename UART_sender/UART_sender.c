// UART Test Harness by Chris Claxton
// Based on the SPI master-slave example from the Pico examples repo

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

// Debug Signal outputs
#define LED_BUILTIN (25u)
#define DEBUG_PIN2 (2u)
#define DEBUG_PIN3 (3u)
#define DEBUG_PIN4 (4u)
#define DEBUG_PIN5 (5u)
#define DEBUG_PIN_INITIAL_STATE (1)

// Serial data output and debugging options
#define DEBUG_SERIAL_OUTPUT_SCROLLING (false) // If not scrolling the terminal position is reset using escape sequences, proper terminal emulator required
#define DEBUG_SERIAL_OUTPUT_PAGE_LIMIT (0) // Set to zero to show all pages
#define DEBUG_SERIAL_OUTPUT_DURING_UART_RECEIVE (true) // Set to false to prevent USB Serial debug output during UART data reception

#define UART_ID uart0
//#define BAUD_RATE (115200)
//#define BAUD_RATE (230400)
//#define BAUD_RATE (460800)
//#define BAUD_RATE (576000)
#define BAUD_RATE (921600)
#define DATA_BITS (8)
#define PARITY    UART_PARITY_NONE
#define STOP_BITS (1)
#define FIFO_ENABLED (true)

// Default for UART0 is GP0 and GP1, see the GPIO function select table in the datasheet for information on which other pins can be used.
#define UART_TX_PIN (0)
#define UART_RX_PIN (1)

#define BUF_LEN         0xFF // 255 byte buffer
uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

bool ledState = false;
unsigned int sendCounter = 0, lastSendCount = 0, sendRate = 0;
unsigned int lastSendMillis = 0, sendInterval = 100; // send every 100 milliseconds

unsigned int seconds = 0, lastSeconds = 0;
unsigned int loopCounter = 0, lastLoopCounter = 0;
unsigned int receiveCounter = 0, lastReceiveCount = 0, receiveRate = 0, errorCount = 0, incompleteReceiveCount = 0, sendErrorCount = 0;

volatile  unsigned int _startByte = 0; // A pre-agreed start byte between the sender and receiver, not implemented yet.
volatile  unsigned int _byteIndex = 0, _expectedByteCount = 0, _bytesReceived = 0;
volatile bool uartDataReady = false;
volatile unsigned int  bytesAvailable = 0, bytesExpected = 0;
unsigned int lastBytesAvailable = 0, lastBytesExpected = 0;


void printBuffer(uint8_t buf[], size_t len) {
    int i;
    for (i = 0; i < len; ++i) {
        if (i % 16 == 15)
            printf("%02X \r\n", buf[i]);
        else
            printf("%02X ", buf[i]);
    }

    // append trailing newline if there isn't one
    if (i % 16) {
        printf("   \r\n");
    }
}

bool verifyInBuffer(unsigned int page) {
    bool success = true;
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        uint8_t inverted = (uint8_t)~i;
        if (in_buf[i] != inverted) {
            printf("Error page: %07u index: %03u expected: 0x%02X received 0x%02X    \r\n", page, i, inverted, in_buf[i]);
            errorCount++;
            success = false;
        }
    }
    return success;
}

void printFirstMissingByte(unsigned int page) {
    for (uint8_t i = 0; i < BUF_LEN; ++i) {
        uint8_t inverted = (uint8_t)~i;
        if (in_buf[i] != inverted) {
            printf("ERROR! page: %07u first missing byte at index: %03u expected: 0x%02X received 0x%02X    \r\n", page, i, inverted, in_buf[i]);
            break;
        }
    }
}

void clearBuffer(uint8_t buf[], size_t len) {
    for (int i = 0; i < len; ++i) {
        buf[i] = 0;
    }
}

// RX interrupt handler
void on_uart_rx() {
    gpio_put(DEBUG_PIN2, 0);
    while (uart_is_readable(UART_ID)) {
        if (_byteIndex == 0) {
            _expectedByteCount = uart_getc(UART_ID);
        } else {
            in_buf[_byteIndex - 1] = uart_getc(UART_ID);
        }
        _byteIndex++;
    }
    // Output a low pulse on a debug pin if we received something unexpected
    if (_byteIndex == 0 && _expectedByteCount < BUF_LEN) {
        gpio_put(DEBUG_PIN4, 0);
        //printf("ebc: %u \r\n", expectedByteCount);
        gpio_put(DEBUG_PIN4, 1);
    }
    _bytesReceived = _byteIndex > 0 ? _byteIndex - 1 : 0; // account for the prefix byte, expectedByteCount
    // Once we have all the expected data, mark as ready
    if (_bytesReceived >= _expectedByteCount) {
        uartDataReady = true;
        bytesExpected = _expectedByteCount;
        bytesAvailable = _bytesReceived;
        _byteIndex = 0;
    }
    gpio_put(DEBUG_PIN2, 1);
}

void resetAllRxVars() {
    _byteIndex = 0;
    _expectedByteCount = 0;
    _bytesReceived = 0;
    uartDataReady = false;
    bytesExpected = 0;
    bytesAvailable = 0;
}

void sendTestData(uint8_t length) {
    printf("UART Sender says: Sending Output buffer...  (page %u, buffer size: %03u) \r\n", sendCounter, length);
    gpio_put(DEBUG_PIN3, 0);
    // Send the data length value on the UART so the receiver knows what to expect next
    if (uart_is_writable(UART_ID)) {
        uart_putc_raw(UART_ID, length); // Send the buffer length on the UART first
        // Write the output buffer to the UART
        for (uint8_t x = 0; x < length; ++x) {
            uart_putc_raw(UART_ID, out_buf[x]);
        }
        sendCounter++;
        gpio_put(DEBUG_PIN3, 1);
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            if (DEBUG_SERIAL_OUTPUT_DURING_UART_RECEIVE) {
                printf("UART Sender says: Output buffer page %u sent, buffer size: %03u \r\n", sendCounter, length);
            }
        }
    } else {
        gpio_put(DEBUG_PIN3, 1);
        sendErrorCount++;
        if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
            printf("UART Sender says: ERROR!!! Could not send Output buffer page %u, uart_is_writable returned false \r\n", sendCounter);
        }
    }
}

int main() {
    // Enable UART so we can print
    stdio_init_all();

    int startupDelay = 10;
    for (int i = 1; i <= startupDelay; ++i) {
        printf("Waiting %d seconds to start: %d\r\n", startupDelay, i);
        sleep_ms(1000);
    }
    printf("\e[2J\e[H"); // clear screen and go to home position

    printf("UART Sender example using baud rate: %d \r\n", BAUD_RATE);
    printf("rp2040_chip_version: %u \r\n", rp2040_chip_version());
    printf("rp2040_rom_version: %u \r\n", rp2040_rom_version());
    printf("get_core_num: %u \r\n\r\n", get_core_num());

    // Init the onboard LED
    gpio_set_function(LED_BUILTIN, GPIO_FUNC_SIO);
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, GPIO_OUT);
    gpio_put(LED_BUILTIN, ledState);

    // Init the debug pins
    gpio_set_function(DEBUG_PIN2, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_put(DEBUG_PIN2, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN3, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);
    gpio_put(DEBUG_PIN3, DEBUG_PIN_INITIAL_STATE);

    gpio_set_function(DEBUG_PIN4, GPIO_FUNC_SIO);
    gpio_init(DEBUG_PIN4);
    gpio_set_dir(DEBUG_PIN4, GPIO_OUT);
    gpio_put(DEBUG_PIN4, DEBUG_PIN_INITIAL_STATE);

    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    printf("Requested UART Baud Rate: %d \r\n", BAUD_RATE);
    int actualBaudRate = uart_set_baudrate(UART_ID, BAUD_RATE);
    printf("Actual UART Baud Rate: %d \r\n", actualBaudRate);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    printf("FIFO Enabled: %s \r\n\r\n", FIFO_ENABLED ? "true" : "false");
    uart_set_fifo_enabled(UART_ID, FIFO_ENABLED);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // Initialize output buffer
    for (size_t i = 0; i < BUF_LEN; ++i) {
        // The values should be: {0x01, 0x02, 0x03...}
        out_buf[i] = i + 1;
    }
    clearBuffer(in_buf, BUF_LEN);

    printf("UART Sender says: The value: 0x%02X (%u) (buffer size) followed immediately by the buffer printed below will be sent to the receiver every: %u ms\r\n", BUF_LEN, BUF_LEN, sendInterval);
    printBuffer(out_buf, BUF_LEN);
    printf("\r\n");
    printf("The value 0x%02X (%u) is expected to be returned followed by a reversed version of the above buffer\r\n\r\n", BUF_LEN, BUF_LEN);

    unsigned long startMillis = to_ms_since_boot(get_absolute_time());
    unsigned long currentMillis = 0;

    // Loop for ever...
    for (size_t i = 0; ; ++i) {
        loopCounter++;
        // Keep track of seconds since start
        currentMillis = to_ms_since_boot(get_absolute_time());
        seconds = (currentMillis - startMillis) / 1000;
        if (seconds - lastSeconds > 0) {
            lastSeconds = seconds;
            // calculate the loop rate, per second
            lastLoopCounter = loopCounter;
            loopCounter = 0;
            // calculate the send rate, per second
            sendRate = sendCounter - lastSendCount;
            lastSendCount = sendCounter;
            //calculate the receive rate, per second
            receiveRate = receiveCounter - lastReceiveCount;
            lastReceiveCount = receiveCounter;
        }

        // Send data every sendInterval
        if (currentMillis >= lastSendMillis + sendInterval) {
            lastSendMillis = currentMillis;
            gpio_put(LED_BUILTIN, 1); // turn on the LED
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                if ((receiveCounter + incompleteReceiveCount) < sendCounter) {
                    incompleteReceiveCount++;
                    errorCount++;
                    printf("ERROR!!! The page %u response was incomplete!!! _expectedByteCount: %03u and _bytesReceived: %03u should equal the Buffer Length: %03u\r\n", sendCounter, _expectedByteCount, _bytesReceived, BUF_LEN);
                    printBuffer(in_buf, BUF_LEN);
                    printf("UART Sender says: ERROR!!! Received incomplete buffer!!! (printed above) \r\n");
                    printFirstMissingByte(sendCounter);
                    clearBuffer(in_buf, BUF_LEN);
                    resetAllRxVars(); // recover from this anomaly
                }
                // Reset the previous terminal position if we are not scrolling the output
                if (!DEBUG_SERIAL_OUTPUT_SCROLLING) {
                    printf("\e[H"); // move to the home position, at the upper left of the screen
                    printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                }
                // Print the header info
                printf("\r\nSeconds: %07u.%03u       \r\n", seconds, currentMillis - startMillis - (seconds * 1000));
                printf("LoopRate: %07u         \r\n", lastLoopCounter); // how many loops per second
                printf("sendCounter: %07u         \r\n", sendCounter);
                printf("sendRate: %07u         \r\n", sendRate);
                printf("Send errorCount: %03u         \r\n", sendErrorCount);
                printf("receiveCounter: %07u         \r\n", receiveCounter);
                printf("receiveRate: %07u         \r\n", receiveRate);
                printf("Receive errorCount: %03u         \r\n", errorCount);
                printf("Receive incompleteReceiveCount: %03u         \r\n", incompleteReceiveCount);
                printf("Receive FailureRate: %11.7f percent  \r\n", 100.0f * errorCount / (sendCounter > 0 ? sendCounter : 1));
            }
            // Send the test data
            sendTestData(BUF_LEN);
            gpio_put(LED_BUILTIN, 0); // turn off the LED
        }

        // Check if we have all the expected data gathered by the receive interrupt
        if (uartDataReady) {
            receiveCounter++;
            gpio_put(DEBUG_PIN3, 0);
            if ((DEBUG_SERIAL_OUTPUT_PAGE_LIMIT == 0) || (receiveCounter <= DEBUG_SERIAL_OUTPUT_PAGE_LIMIT)) { // optionally only show the results up to DEBUG_SERIAL_OUTPUT_PAGE_LIMIT
                // print data to the serial port
                printf("UART Sender says: read page %u back from the receiver, received page size: %03u expected: %03u lastExpected: %03u                               \r\n", receiveCounter, bytesAvailable, bytesExpected, lastBytesExpected);
                // Write the input buffer out to the USB serial port
                printBuffer(in_buf, BUF_LEN);
                printf("UART Sender says: Verifying received data...                                         \r\n");
                if (bytesExpected != BUF_LEN) {
                    printf("ERROR!!! page: %u bytesExpected: %03u should equal the Buffer Length: %03u\r\n", receiveCounter, bytesExpected, BUF_LEN);
                }
                verifyInBuffer(receiveCounter);
            }
            clearBuffer(in_buf, BUF_LEN);
            lastBytesExpected = bytesExpected;
            uartDataReady = false;
            bytesAvailable = 0;
            gpio_put(DEBUG_PIN3, 1);
        }
    }
}
