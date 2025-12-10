#include "SerialIO.hpp"

SerialIO::SerialIO(size_t usb_buffer_size = 1024, size_t io_buffer_size = 128) :
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
    if (!is_initialized_) return ""; // Don't process input if the JTAG driver is not initialized.
    serial_out(prompt);  // Output the prompt using the serial_out() method defined in this class.

    std::string input_str;  // Declare the input_str object to hold inputted chars, with its specified buffer space to avoid overflow.
    input_str.reserve(io_buffer_size_);
    char input_char;
    // While loop that dynamically emulates a terminal session for the user. During this loop the user can dynamically enter and remove characters, 
    // Before confirming their choice using the enter key (entering a '\n' char).
    // Additionally there are guardrails that ensure that edge-case inputs, and backspaces, are handled correctly.
    // During each iteration of the loop, all entered characters so far are presented back to the user, thus establishing the dynamic entry & removal of chars.
    while (true) {
        int len = usb_serial_jtag_read_bytes(&input_char, 1, portMAX_DELAY);
        if (len > 0) {
            // Ignore carraige returns.
            if (input_char == '\r') {
                continue;
            }
            // Break and store when user presses enter.
            if (input_char == '\n') {
                serial_out("\r\n");
                break;
            }

            //  
            if (input_char == '\b' || input_char == io_buffer_size_ - 1) {
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