#include <cstddef>
#include <cstdint>

#ifndef LUNABOTICS_I2C_TINY_USB_BRIDGE_HPP
#define LUNABOTICS_I2C_TINY_USB_BRIDGE_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

/**
 * @brief One I2C message of an i2c-tiny-usb transaction.
 */
struct I2CTinyUSBMessage {
    uint8_t* data = nullptr; // Payload to write, or destination of the read data.
    uint16_t len = 0;        // Payload length in bytes. Zero only addresses the target.
    bool read = false;       // True to read from the target, false to write to it.
};

/**
 * @brief I2C controller used by I2CTinyUSBBridge to execute transactions.
 */
class I2CTinyUSBTransport {
public:
    virtual ~I2CTinyUSBTransport() = default;

    /**
     * @brief Executes messages to one target as a single transaction.
     * Messages are separated by repeated starts, and the transaction ends with a stop.
     * @param address The 7-bit target address.
     * @param msgs The messages in bus order. Read messages receive their data.
     * @param count The number of messages, at least one.
     * @return True if the target acknowledged its address and every written byte.
     */
    virtual bool transfer(uint16_t address, I2CTinyUSBMessage* msgs, size_t count) = 0;

    /**
     * @brief Requests the SCL frequency of subsequent transactions.
     * @param frequency_hz The requested SCL frequency in Hz.
     * @return True if the frequency will be applied.
     */
    virtual bool setFrequency(uint32_t frequency_hz) = 0;
};

/**
 * @brief Handles the vendor requests of the I2C-Tiny-USB protocol, independent of the USB device stack.
 *
 * Ported from the Zephyr i2c_tiny_usb sample. The protocol is the one of https://github.com/harbaum/I2C-Tiny-USB,
 * as used by the Linux i2c-tiny-usb bus driver. All communication is done through vendor requests on the
 * default control pipe. bRequest encodes the command. For CMD_I2C_IO, wValue carries the Linux I2C message
 * flags, wIndex the target address, and the data stage the payload. The host marks the first and last message
 * of a transaction with the BEGIN and END bits.
 *
 * Each message arrives in a separate request. The messages are collected and executed as one transaction to
 * preserve the repeated starts between them. Execution happens when the host announces the last message or
 * requests read data, so a NAK from a deferred write is reported with the status of the whole transaction.
 */
class I2CTinyUSBBridge {
public:
    // Commands in bRequest:
    static constexpr uint8_t CMD_ECHO = 0;
    static constexpr uint8_t CMD_GET_FUNC = 1;
    static constexpr uint8_t CMD_SET_DELAY = 2;
    static constexpr uint8_t CMD_GET_STATUS = 3;
    static constexpr uint8_t CMD_I2C_IO = 4;
    static constexpr uint8_t CMD_I2C_IO_BEGIN = 1 << 0;
    static constexpr uint8_t CMD_I2C_IO_END = 1 << 1;

    // Status reported by CMD_GET_STATUS:
    static constexpr uint8_t STATUS_IDLE = 0;
    static constexpr uint8_t STATUS_ADDRESS_ACK = 1;
    static constexpr uint8_t STATUS_ADDRESS_NAK = 2;

    // Linux I2C message flag in the wValue of CMD_I2C_IO:
    static constexpr uint16_t M_RD = 1 << 0;

    // Linux I2C adapter functionality reported by CMD_GET_FUNC:
    static constexpr uint32_t FUNC_I2C = 0x00000001UL;
    static constexpr uint32_t FUNC_SMBUS_EMUL = 0x0eff0008UL;

    // Limits of one transaction:
    static constexpr size_t MAX_MSGS = 8;
    static constexpr size_t MAX_DATA = 256;

    /**
     * @brief The SETUP packet fields of a vendor request.
     */
    struct Setup {
        uint8_t bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;
    };

    /**
     * @brief Constructs a bridge with an idle status.
     * @param transport The I2C controller, which must outlive the bridge.
     */
    explicit I2CTinyUSBBridge(I2CTinyUSBTransport& transport) : transport_(transport) {}

    // Rule of 3: Copy Forbid:
    I2CTinyUSBBridge(const I2CTinyUSBBridge&) = delete;
    I2CTinyUSBBridge& operator=(const I2CTinyUSBBridge&) = delete;

    /**
     * @brief Checks a host-to-device request before its data stage is received.
     * @return True if controlToDevice() handles the request.
     */
    static bool acceptsToDevice(const Setup& setup);

    /**
     * @brief Handles a host-to-device vendor request after its data stage.
     * @param setup The request.
     * @param data The wLength bytes of the data stage, or nullptr when wLength is zero.
     * @return False if the request is not supported and must be stalled.
     * @note Transaction failures are reported through CMD_GET_STATUS, not through the return value.
     */
    bool controlToDevice(const Setup& setup, const uint8_t* data);

    /**
     * @brief Handles a device-to-host vendor request.
     * @param setup The request.
     * @param data Destination for the data stage.
     * @param capacity The size of data in bytes.
     * @param len Set to the number of bytes to send.
     * @return False if the request is not supported and must be stalled.
     */
    bool controlToHost(const Setup& setup, uint8_t* data, size_t capacity, size_t& len);

    /**
     * @brief Gets the status reported by CMD_GET_STATUS.
     */
    uint8_t getStatus() const { return status_; }

private:
    static bool isI2CIO(uint8_t request) {
        return (request & ~(CMD_I2C_IO_BEGIN | CMD_I2C_IO_END)) == CMD_I2C_IO;
    }

    I2CTinyUSBMessage* append(const Setup& setup, bool read);
    void run();
    void clear();
    void fail();
    void setDelay(uint16_t delay_us);

    I2CTinyUSBTransport& transport_;
    I2CTinyUSBMessage msgs_[MAX_MSGS]{};
    uint8_t data_[MAX_DATA]{};
    uint16_t used_ = 0;
    uint8_t num_msgs_ = 0;
    uint16_t addr_ = 0;
    uint8_t status_ = STATUS_IDLE;
};

}
}
}

#endif
