#include <string>
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"


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
    /// @note There is a guardrail to ensure that the outputted message object does not exceed the set maximum number of chars set in io_buffer_size_
    void serial_out(const std::string& message);

private: 
    size_t usb_serial_buf_size_;  // Use size_t objects to hold the USB buffer value, char limiter.
    size_t io_buffer_size_;
    bool is_initialized_ = false; 
    usb_serial_jtag_driver_config_t usb_config_;
};