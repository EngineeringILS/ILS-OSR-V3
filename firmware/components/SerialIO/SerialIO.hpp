#include <string>
#include <common/drivers/SerialInterface.hpp>
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"

#ifndef SERIAL_IO_HPP
#define SERIAL_IO_HPP

enum class LineResult { None, Line, Overflow };

/**
 * TODO: Migrate SerialIO to SerialInterface:
 */
// class SerialIO : public SerialInterface {
    
// };


/// @brief A class for managing serial input/output operations using the ESP32's JTAG interface.
/// @details This class follows the RAII pattern, installing the driver on initialization
///          and uninstalling it upon destruction.
class SerialIO {
public:
    /// @brief Constructs a SerialIO object and allows for custom buffer sizes.
    /// @param usb_buffer_size The size in bytes for the underlying TX and RX driver buffers.
    /// @param io_buffer_size The max number of characters for a single line read operation.
    SerialIO(size_t usb_buffer_size = 1024, size_t io_buffer_size = 128);

    /// @brief Destructor that cleans up and uninstalls the USB serial JTAG driver.
    ~SerialIO();

    // Rule of 3: Copy Forbid:
    SerialIO(const SerialIO&) = delete;
    SerialIO& operator=(const SerialIO&) = delete;

    /// @brief Installs the USB serial JTAG driver with the configured settings.
    /// @return ESP_OK on success, or an error code on failure.
    esp_err_t init();

    /// @brief Uninstalls the USB serial JTAG driver.
    void deinit();

    /// @brief Gets a line of input from the serial console.
    /// @param prompt The prompt to display to the user before waiting for input.
    /// @return The user's input as an std::string, without the trailing newline.
    /// @note This function basically loops through and echoes back a users input, then returing the unified string whenever enter is pressed.
    /// @note Due to the design of this class, the input prompt will only accept up to a number of chars equal to io_buffer_size - 1 
     std::string serial_in(const std::string& prompt);

    /// @brief Sends a string to the serial console.
    /// @param message The message to send.
    /// @note Long messages are written in chunks; io_buffer_size_ limits input lines only.
    void serial_out(const std::string& message);

    /// Nonblocking, non-echoing input; preserves partial lines between calls.
    /// Consumes at most io_buffer_size bytes per call. CR is ignored; LF ends
    /// a line. Invalid/overlong lines are discarded through LF, then Overflow
    /// is returned once. Do not mix this method with serial_in on one instance.
    LineResult poll_line(std::string& line);

private: 
    size_t usb_serial_buf_size_;  // Use size_t objects to hold the USB buffer value, char limiter.
    size_t io_buffer_size_;
    bool is_initialized_ = false; 
    usb_serial_jtag_driver_config_t usb_config_;
    std::string pending_line_;
    bool discard_line_ = false;
};

#endif
