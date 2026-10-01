#include <i2c_driver.hpp>
#include "i2c_tiny_usb_bridge.hpp"

#ifndef LUNABOTICS_I2C_TINY_USB_TRANSPORT_HPP
#define LUNABOTICS_I2C_TINY_USB_TRANSPORT_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

/**
 * @brief Executes i2c-tiny-usb transactions on an I2CBus.
 *
 * Every target shares one device registration without an address. Each message starts with an
 * explicit address byte inside one i2c_master defined-operations list, which keeps the repeated
 * starts between messages and ends the transaction with a single stop.
 *
 * @note The controller executes at most SOC_I2C_CMD_REG_NUM operations per transaction. A read must be
 * the last message, and a transaction with several messages must fit the transmit FIFO. Transactions
 * outside these limits fail with ESP_ERR_NOT_SUPPORTED instead of being split.
 */
class I2CBusTransport : public I2CTinyUSBTransport {
public:
    /**
     * @brief Constructs a transport at the bus frequency, limited to max_frequency_hz.
     * @param bus The initialized bus, which must outlive the transport.
     * @param max_frequency_hz The highest SCL frequency the host may select, in Hz.
     * @param timeout_ms The timeout of one transaction, in ms.
     */
    explicit I2CBusTransport(I2CBus* bus, uint32_t max_frequency_hz = 400000, int timeout_ms = 100);

    /**
     * @brief Removes the device registration from the bus.
     */
    ~I2CBusTransport() override;

    // Rule of 3: Copy Forbid:
    I2CBusTransport(const I2CBusTransport&) = delete;
    I2CBusTransport& operator=(const I2CBusTransport&) = delete;

    bool transfer(uint16_t address, I2CTinyUSBMessage* msgs, size_t count) override;

    /**
     * @brief Sets the SCL frequency of the next transaction, limited to the configured maximum.
     */
    bool setFrequency(uint32_t frequency_hz) override;

    uint32_t getFrequency() const { return frequency_hz_; }
    esp_err_t getErr() const { return err_; }

private:
    bool attach();

    I2CBus* bus_;
    const uint32_t max_frequency_hz_;
    const int timeout_ms_;
    uint32_t frequency_hz_;
    uint32_t device_frequency_hz_ = 0;
    i2c_master_dev_handle_t device_handle_ = nullptr;
    esp_err_t err_ = ESP_OK;

    // Address byte and payload of each message, as written to the bus.
    uint8_t tx_[I2CTinyUSBBridge::MAX_DATA + 1]{};
    // Receives the byte clocked by a zero-length read.
    uint8_t discard_ = 0;
};

}
}
}

#endif
