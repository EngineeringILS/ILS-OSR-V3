#include "i2c_tiny_usb_bridge.hpp"
#include <algorithm>
#include <cstring>
#include "esp_log.h"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

static const char* TAG = "i2c_tiny_usb";

bool I2CTinyUSBBridge::acceptsToDevice(const Setup& setup) {
    return isI2CIO(setup.bRequest) || setup.bRequest == CMD_SET_DELAY;
}

bool I2CTinyUSBBridge::controlToDevice(const Setup& setup, const uint8_t* data) {
    if (isI2CIO(setup.bRequest)) {
        I2CTinyUSBMessage* msg = append(setup, false);
        if (msg == nullptr) {
            // The failure is reported through CMD_GET_STATUS.
            return true;
        }

        if (msg->len != 0) {
            std::memcpy(msg->data, data, msg->len);
        }

        if (setup.bRequest & CMD_I2C_IO_END) {
            run();
        } else {
            // Execution is deferred until the last message.
            status_ = STATUS_ADDRESS_ACK;
        }
        return true;
    }

    if (setup.bRequest == CMD_SET_DELAY) {
        setDelay(setup.wValue);
        return true;
    }

    ESP_LOGE(TAG, "Vendor request 0x%02x to device not supported", setup.bRequest);
    return false;
}

bool I2CTinyUSBBridge::controlToHost(const Setup& setup, uint8_t* data, size_t capacity, size_t& len) {
    len = 0;

    if (isI2CIO(setup.bRequest)) {
        // Always return the requested length, the host driver treats a short transfer as a fatal error.
        if (setup.wLength > capacity) {
            return false;
        }

        // Read data is needed now, so the read ends the collected transaction.
        I2CTinyUSBMessage* msg = append(setup, true);
        if (msg != nullptr) {
            run();
        }

        if (msg != nullptr && status_ == STATUS_ADDRESS_ACK) {
            std::memcpy(data, msg->data, msg->len);
        } else {
            std::memset(data, 0, setup.wLength);
        }
        len = setup.wLength;
        return true;
    }

    switch (setup.bRequest) {
        case CMD_ECHO: {
            const uint8_t echo[2] = {
                static_cast<uint8_t>(setup.wValue), static_cast<uint8_t>(setup.wValue >> 8)
            };
            len = std::min({static_cast<size_t>(setup.wLength), sizeof(echo), capacity});
            std::memcpy(data, echo, len);
            return true;
        }
        case CMD_GET_FUNC: {
            const uint32_t func = FUNC_I2C | FUNC_SMBUS_EMUL;
            const uint8_t le_func[4] = {
                static_cast<uint8_t>(func), static_cast<uint8_t>(func >> 8),
                static_cast<uint8_t>(func >> 16), static_cast<uint8_t>(func >> 24)
            };
            len = std::min({static_cast<size_t>(setup.wLength), sizeof(le_func), capacity});
            std::memcpy(data, le_func, len);
            return true;
        }
        case CMD_GET_STATUS:
            len = std::min({static_cast<size_t>(setup.wLength), sizeof(status_), capacity});
            std::memcpy(data, &status_, len);
            return true;
        default:
            break;
    }

    ESP_LOGE(TAG, "Vendor request 0x%02x to host not supported", setup.bRequest);
    return false;
}

I2CTinyUSBMessage* I2CTinyUSBBridge::append(const Setup& setup, bool read) {
    if (setup.bRequest & CMD_I2C_IO_BEGIN) {
        clear();
    }

    // A write announced as a read, or the reverse, would put stale buffer contents on the bus.
    if (((setup.wValue & M_RD) != 0) != read) {
        ESP_LOGE(TAG, "Message direction does not match the request direction");
        fail();
        return nullptr;
    }

    if (num_msgs_ >= MAX_MSGS || setup.wLength > MAX_DATA - used_) {
        ESP_LOGE(TAG, "Transaction exceeds message or data limits");
        fail();
        return nullptr;
    }

    if (num_msgs_ == 0) {
        addr_ = setup.wIndex;
    } else if (addr_ != setup.wIndex) {
        ESP_LOGE(TAG, "Multiple target addresses in a transaction are not supported");
        fail();
        return nullptr;
    }

    I2CTinyUSBMessage* msg = &msgs_[num_msgs_];
    msg->data = &data_[used_];
    msg->len = setup.wLength;
    msg->read = read;

    num_msgs_++;
    used_ += setup.wLength;

    return msg;
}

void I2CTinyUSBBridge::run() {
    if (transport_.transfer(addr_, msgs_, num_msgs_)) {
        status_ = STATUS_ADDRESS_ACK;
    } else {
        // Expected while the host scans for targets, so it is not a warning.
        ESP_LOGD(TAG, "Transfer to address 0x%02x failed", addr_);
        status_ = STATUS_ADDRESS_NAK;
    }

    clear();
}

void I2CTinyUSBBridge::clear() {
    num_msgs_ = 0;
    used_ = 0;
}

void I2CTinyUSBBridge::fail() {
    clear();
    status_ = STATUS_ADDRESS_NAK;
}

void I2CTinyUSBBridge::setDelay(uint16_t delay_us) {
    // The host sends the clock half-period of the original bit-banging firmware;
    // map it to the closest standard bus speed.
    const uint32_t freq_hz = 1000000UL / (2UL * std::max<uint32_t>(delay_us, 1U));
    uint32_t speed_hz = 100000;
    if (freq_hz >= 1000000) {
        speed_hz = 1000000;
    } else if (freq_hz >= 400000) {
        speed_hz = 400000;
    }

    if (!transport_.setFrequency(speed_hz)) {
        // Not all controllers support runtime configuration.
        ESP_LOGW(TAG, "Failed to configure bus speed, continue anyway");
    }
}

}
}
}
