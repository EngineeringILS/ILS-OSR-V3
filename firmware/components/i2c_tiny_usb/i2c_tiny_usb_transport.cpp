#include "i2c_tiny_usb_transport.hpp"
#include <algorithm>
#include <cstring>
#include "esp_log.h"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

static const char* TAG = "i2c_tiny_usb";

I2CBusTransport::I2CBusTransport(I2CBus* bus, uint32_t max_frequency_hz, int timeout_ms) :
    bus_(bus), max_frequency_hz_(max_frequency_hz), timeout_ms_(timeout_ms), frequency_hz_(100000) {
    if (bus_ != nullptr && bus_->getI2CPort().frequency != 0) {
        frequency_hz_ = bus_->getI2CPort().frequency;
    }
    frequency_hz_ = std::min(frequency_hz_, max_frequency_hz_);
}

I2CBusTransport::~I2CBusTransport() {
    if (device_handle_ != nullptr) {
        i2c_master_bus_rm_device(device_handle_);
    }
}

bool I2CBusTransport::setFrequency(uint32_t frequency_hz) {
    if (frequency_hz == 0) {
        return false;
    }
    frequency_hz_ = std::min(frequency_hz, max_frequency_hz_);
    return true;
}

bool I2CBusTransport::transfer(uint16_t address, I2CTinyUSBMessage* msgs, size_t count) {
    if (bus_ == nullptr || !bus_->isInitialized()) {
        err_ = ESP_ERR_INVALID_STATE;
        return false;
    }
    if (msgs == nullptr || count == 0 || address > 0x7F) {
        err_ = ESP_ERR_INVALID_ARG;
        return false;
    }

    // Check the transaction against the controller limits before touching the bus.
    bool read_last = true;
    size_t num_ops = 1; // Final stop.
    size_t tx_len = 0;
    for (size_t i = 0; i < count; i++) {
        if (msgs[i].len != 0 && msgs[i].data == nullptr) {
            err_ = ESP_ERR_INVALID_ARG;
            return false;
        }
        if (msgs[i].read) {
            // The driver collects the data of a single read, at the end of the command list.
            read_last = read_last && (i + 1 == count);
            // Start, address, reads with ACK, last read with NACK.
            num_ops += (msgs[i].len > 1) ? 4 : 3;
            tx_len += 1;
        } else {
            // Start, then address and payload.
            num_ops += 2;
            tx_len += 1 + msgs[i].len;
        }
    }

    // The driver queues all messages in the transmit FIFO before starting the bus.
    // Only a single message is split across several FIFO fills.
    if (!read_last || num_ops > SOC_I2C_CMD_REG_NUM || tx_len > sizeof(tx_) ||
        (count > 1 && tx_len > SOC_I2C_FIFO_LEN)) {
        ESP_LOGW(TAG, "Transaction of %u messages to 0x%02x exceeds the controller limits",
                 static_cast<unsigned>(count), address);
        err_ = ESP_ERR_NOT_SUPPORTED;
        return false;
    }

    i2c_operation_job_t ops[SOC_I2C_CMD_REG_NUM] = {};
    size_t op = 0;
    uint8_t* tx = tx_;
    for (size_t i = 0; i < count; i++) {
        I2CTinyUSBMessage& msg = msgs[i];
        const size_t len = msg.read ? 1 : 1 + msg.len;

        tx[0] = static_cast<uint8_t>((address << 1) | (msg.read ? 1 : 0));
        if (!msg.read && msg.len != 0) {
            std::memcpy(tx + 1, msg.data, msg.len);
        }

        ops[op++].command = I2C_MASTER_CMD_START;
        ops[op].command = I2C_MASTER_CMD_WRITE;
        ops[op].write.ack_check = true;
        ops[op].write.data = tx;
        ops[op].write.total_bytes = len;
        op++;
        tx += len;

        if (msg.read) {
            // A zero-length read still clocks one byte, so the target releases SDA before the stop.
            uint8_t* data = (msg.len != 0) ? msg.data : &discard_;
            const size_t n = (msg.len != 0) ? msg.len : 1;
            if (n > 1) {
                ops[op].command = I2C_MASTER_CMD_READ;
                ops[op].read.ack_value = I2C_ACK_VAL;
                ops[op].read.data = data;
                ops[op].read.total_bytes = n - 1;
                op++;
            }
            // The last byte before the stop is NACKed.
            ops[op].command = I2C_MASTER_CMD_READ;
            ops[op].read.ack_value = I2C_NACK_VAL;
            ops[op].read.data = data + n - 1;
            ops[op].read.total_bytes = 1;
            op++;
        }
    }
    ops[op++].command = I2C_MASTER_CMD_STOP;

    if (!attach()) {
        return false;
    }

    err_ = i2c_master_execute_defined_operations(device_handle_, ops, op, timeout_ms_);
    // ESP_ERR_INVALID_STATE includes the NACK of an absent target, which is routine during a scan.
    if (err_ != ESP_OK && err_ != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "Transfer to 0x%02x failed: %s", address, esp_err_to_name(err_));
    }
    return err_ == ESP_OK;
}

bool I2CBusTransport::attach() {
    if (device_handle_ != nullptr && device_frequency_hz_ == frequency_hz_) {
        return true;
    }

    if (device_handle_ != nullptr) {
        // A new SCL frequency needs a new registration. Removal is refused while another device is
        // mid-transaction, so keep the previous frequency and retry with the next transaction.
        if (i2c_master_bus_rm_device(device_handle_) != ESP_OK) {
            return true;
        }
        device_handle_ = nullptr;
    }

    i2c_device_config_t device_cfg = {};
    device_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_cfg.device_address = I2C_DEVICE_ADDRESS_NOT_USED;
    device_cfg.scl_speed_hz = frequency_hz_;

    err_ = i2c_master_bus_add_device(bus_->getI2CBus(), &device_cfg, &device_handle_);
    if (err_ != ESP_OK) {
        device_handle_ = nullptr;
        return false;
    }
    device_frequency_hz_ = frequency_hz_;
    return true;
}

}
}
}
