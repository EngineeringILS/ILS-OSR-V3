#pragma once

#include "i2c_master.h"
#include <algorithm>
#include <string>

struct usb_serial_jtag_driver_config_t {
    size_t tx_buffer_size, rx_buffer_size;
};

namespace TestSerial {
inline int installations = 0;
inline std::string output;
}

inline esp_err_t usb_serial_jtag_driver_install(const usb_serial_jtag_driver_config_t*) {
    ++TestSerial::installations;
    return ESP_OK;
}
inline void usb_serial_jtag_driver_uninstall() { --TestSerial::installations; }
inline int usb_serial_jtag_read_bytes(void*, size_t, int) { return 0; }
inline int usb_serial_jtag_write_bytes(const void* data, size_t size, int) {
    // Short writes exercise the console's output retry loop.
    const size_t count = std::min(size, size_t{7});
    TestSerial::output.append(static_cast<const char*>(data), count);
    return static_cast<int>(count);
}
