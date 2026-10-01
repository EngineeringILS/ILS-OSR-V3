#include <i2c_driver.hpp>
#include "i2c_tiny_usb_bridge.hpp"
#include "i2c_tiny_usb_transport.hpp"

#ifndef LUNABOTICS_I2C_TINY_USB_HPP
#define LUNABOTICS_I2C_TINY_USB_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

/**
 * @brief USB identity and bus limits of the I2C-Tiny-USB bridge.
 * @note The Linux i2c-tiny-usb driver does not know the default IDs; see README.md for new_id.
 */
struct I2CTinyUSBConfig {
    uint16_t vid = 0x303A;              // Espressif vendor ID.
    uint16_t pid = 0x4020;              // esp_tinyusb default product ID of a vendor-class device.
    uint16_t bcd_device = 0x0100;       // Reported by the Linux driver as the adapter version.
    const char* manufacturer = "Engineering ILS Foundation";
    const char* product = "OSR V3 i2c-tiny-usb bridge";
    const char* serial = nullptr;       // nullptr uses the base MAC address.
    uint32_t max_frequency_hz = 400000; // Highest SCL frequency the host may select.
    int timeout_ms = 100;               // Timeout of one I2C transaction.
};

/**
 * @brief Exposes an I2CBus to a USB host as an I2C-Tiny-USB adapter.
 *
 * Port of the Zephyr i2c_tiny_usb sample to ESP-IDF and TinyUSB. The USB function is a vendor-specific
 * interface without endpoints. Every command is a vendor request on the default control pipe, handled by
 * I2CTinyUSBBridge in the TinyUSB task. The Linux i2c-tiny-usb driver registers the bus as /dev/i2c-N.
 *
 * @warning init() moves the internal USB PHY from the USB Serial/JTAG controller to USB-OTG. SerialIO and the
 * USB Serial/JTAG console stop working, and flashing needs the ROM download mode (hold BOOT, press RESET).
 * @note This component defines the TinyUSB callbacks tud_vendor_control_xfer_cb() and usbd_app_driver_get_cb().
 */
class I2CTinyUSB {
public:
    /**
     * @brief Constructs the bridge without touching the USB peripheral.
     * @param bus The initialized bus to expose, which must outlive the bridge.
     * @param config The USB identity and bus limits. String pointers are copied by init().
     */
    explicit I2CTinyUSB(I2CBus* bus, const I2CTinyUSBConfig& config = I2CTinyUSBConfig());

    /**
     * @brief Uninstalls the USB device stack.
     */
    ~I2CTinyUSB();

    // Rule of 3: Copy Forbid:
    I2CTinyUSB(const I2CTinyUSB&) = delete;
    I2CTinyUSB& operator=(const I2CTinyUSB&) = delete;

    /**
     * @brief Installs the TinyUSB device stack and connects to the host.
     * @return True on success, false on failure; check getErr().
     * @note Only one instance can be initialized at a time.
     */
    bool init();

    /**
     * @brief Disconnects from the host and uninstalls the TinyUSB device stack.
     */
    void deinit();

    /**
     * @brief Checks whether the host has configured the device.
     */
    bool isMounted() const;

    bool isInitialized() const { return initialized_; }
    esp_err_t getErr() const { return err_; }

private:
    friend struct I2CTinyUSBCallbacks;

    I2CBus* bus_;
    I2CTinyUSBConfig config_;
    I2CBusTransport transport_;
    I2CTinyUSBBridge bridge_;
    bool initialized_ = false;
    esp_err_t err_ = ESP_OK;

    // Data stages of control transfers.
    uint8_t out_[I2CTinyUSBBridge::MAX_DATA]{};
    uint8_t in_[I2CTinyUSBBridge::MAX_DATA]{};
};

}
}
}

#endif
