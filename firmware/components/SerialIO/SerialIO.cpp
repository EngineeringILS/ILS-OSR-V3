#include "SerialIO.hpp"

SerialIO::SerialIO(size_t usb_buffer_size, size_t io_buffer_size) :
    usb_serial_buf_size_(usb_buffer_size),
    io_buffer_size_(io_buffer_size),
    usb_config_{
        .tx_buffer_size = usb_buffer_size, // Use the constructor parameter directly
        .rx_buffer_size = usb_buffer_size  // Use the constructor parameter directly
    }
    {}

SerialIO::~SerialIO() {
    deinit();
}

esp_err_t SerialIO::init() {
    esp_err_t err = usb_serial_jtag_driver_install(&usb_config_);
    is_initialized_ = (err == ESP_OK);
    return err;
}

void SerialIO::deinit() {
    if (is_initialized_) {
        usb_serial_jtag_driver_uninstall();
        is_initialized_ = false;
    }
}

std::string SerialIO::serial_in(const std::string& prompt) {
    if (!is_initialized_) return "";

    serial_out(prompt);

    std::string input_str;
    input_str.reserve(io_buffer_size_);
    char input_char;

    while (true) {
        int len = usb_serial_jtag_read_bytes(&input_char, 1, portMAX_DELAY);
        if (len > 0) {
            if (input_char == '\r') {
                continue;
            }
            if (input_char == '\n') {
                serial_out("\r\n");
                break;
            }
            // --- MODIFIED SECTION: Handle backspace ---
            // Note: 127 is the ASCII code for the 'DEL' key, which some terminals send for backspace.
            if (input_char == '\b' || input_char == 127) {
                // Only process backspace if the input string is not empty.
                if (!input_str.empty()) {
                    input_str.pop_back();
                    // Send backspace, space, backspace to visually erase the character on the terminal.
                    serial_out("\b \b");
                }
            }
            // Only add other characters if they are printable and fit in the buffer.
            else if (input_str.length() < io_buffer_size_ - 1) {
                input_str += input_char;
                usb_serial_jtag_write_bytes(&input_char, 1, portMAX_DELAY);
            }
        }
    }
    return input_str;
}

void SerialIO::serial_out(const std::string& message) {
    if (!is_initialized_ || message.empty()) return;
    if (message.size() < io_buffer_size_) {
        usb_serial_jtag_write_bytes(message.c_str(), message.length(), portMAX_DELAY);
    }
}