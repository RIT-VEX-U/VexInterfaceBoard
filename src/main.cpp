#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <iostream>
#include <cstring>

#define UART_ID uart0  // Use UART0
#define TX_PIN 0       // UART TX Pin
#define RX_PIN 1       // UART RX Pin
#define BAUD_RATE 115200 // Match with VEX Brain

// Initialize UART
void setup_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
}

// COBS Decoding
size_t cobs_decode(const uint8_t* input, size_t length, uint8_t* output) {
    size_t read_index = 0, write_index = 0, code = 0;

    while (read_index < length) {
        code = input[read_index++];
        for (size_t i = 1; i < code; i++) {
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFF && read_index < length) {
            output[write_index++] = 0;
        }
    }

    return write_index;
}

// Receive and process COBS-encoded data
void receive_and_process_data() {
    const size_t max_encoded_size = 10;  // Maximum size of COBS-encoded uint32_t
    uint8_t rx_buffer[max_encoded_size];
    uint8_t decoded_buffer[4];          // uint32_t is 4 bytes
    size_t received = 0;

    // Wait for enough data to arrive
    while (uart_is_readable(UART_ID)) {
        rx_buffer[received++] = uart_getc(UART_ID);
        if (rx_buffer[received - 1] == 0) {
            break;  // COBS encoding uses 0 as a delimiter
        }
    }

    if (received > 0) {
        // Decode the received data
        size_t decoded_length = cobs_decode(rx_buffer, received, decoded_buffer);

        if (decoded_length == 4) {
            // Convert the decoded bytes into a uint32_t
            uint32_t value = 0;
            memcpy(&value, decoded_buffer, sizeof(uint32_t));

            // Print the value
            std::cout << "Received uint32_t: " << value << std::endl;
        } else {
            std::cerr << "Error: Decoded length mismatch" << std::endl;
        }
    }
}

int main() {

    stdio_init_all();  // Initialize standard I/O
    setup_uart();      // Initialize UART

    while (true) {
        receive_and_process_data();  // Handle incoming data
        sleep_ms(100);  // Small delay to prevent overwhelming the system
    }

    return 0;
}
