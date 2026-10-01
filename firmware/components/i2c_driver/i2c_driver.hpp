#include <common/protocols/InterfaceProtocols.hpp>
#include "driver/i2c_master.h"

#ifndef LUNABOTICS_I2C_DRIVER_HPP
#define LUNABOTICS_I2C_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {
using namespace Lunabotics::Common;

/**
 * @brief Hardware abstracted C++ Class for creating an I2C bus using ESP-IDF's i2c_master driver. 
 * Provides initialization, destructor, and public status data for the I2C Bus. 
 */
class I2CBus {
public:
    /**
     * @brief I2C Bus constructor that attemps initialization on construction.
     * @param port The reference I2CPort struct to initialize.
     */
    explicit I2CBus(const Protocols::I2CPort &port) : port_(port) {
        init();
    }

    // Rule of 3: Copy Forbid:
    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    ~I2CBus() {
        if (initialized_) {
            i2c_del_master_bus(bus_handle_);
        }
    }

    /**
     * @brief Attemps to initialize the I2C Bus. 
     * Suceeds if err = ESP_OK and initialized_ = true. 
     * Fails gracefully specifying the error code in err, and setting initialized_ to false.
     */
    bool init() {
        if (!port_.exists) {
            initialized_ = false;
            // Port cannot be found:
            err_ = ESP_ERR_NOT_FOUND;
        

        // Skip re-initialization (the I2C Bus is expected to either never fail or catastrophically fail, if the I2C Bus catastrophically fails, its a bigger problem.):
        } else if (!initialized_) {
            i2c_master_bus_config_t i2c_bus_config = {};
            i2c_bus_config.i2c_port = port_.i2c_port;
            i2c_bus_config.sda_io_num = static_cast<gpio_num_t>(port_.sda_pin);
            i2c_bus_config.scl_io_num = static_cast<gpio_num_t>(port_.scl_pin);
            i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
            i2c_bus_config.glitch_ignore_cnt = 7;
            i2c_bus_config.flags.enable_internal_pullup = true;

            err_ = i2c_new_master_bus(&i2c_bus_config, &bus_handle_);
            if (err_ == ESP_OK) {
                initialized_ = true;
            } else {
                initialized_ = false;
            }
        }

    // Return the current initialization state:
    return initialized_;
    }

    i2c_master_bus_handle_t getI2CBus() {
        return bus_handle_;
    }

    Protocols::I2CPort getI2CPort() const {
        return port_;
    }

    esp_err_t getErr() const {
        return err_;
    }

    bool isInitialized() const {
        return initialized_;
    }
    
private:
    const Protocols::I2CPort port_;
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    esp_err_t err_ = ESP_OK;
    bool initialized_ = false;
};

/**
 * @brief Registers an I2C device and provides byte and word register access.
 */
class I2CDevice {
public:
    /**
     * @brief Construct a new I2CDevice object.
     * @param address The 7-bit hex address of the I2C Device (e.g. 0x36).
     * @param bus A pointer to the I2CBus this sensor will operate on.
     * @param sclFreq The desired SCL frequency in Hz, within the device and board limits.
     */
    explicit I2CDevice(const uint8_t &address, I2CBus* bus,  const uint32_t &sclFreq = 100000) : bus_(bus), address_(address), device_scl_freq_(sclFreq)  {
        init();
    }

    /**
     * @brief Destroy the existing I2CDevice object.
     */
    virtual ~I2CDevice() {
        if (device_handle_ != nullptr) {
            i2c_master_bus_rm_device(device_handle_);
        }
    }

    // Rule of 3: Copy Forbid:
    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;
    
    /**
     * @brief Initializes the I2C Device with the provided configuration, sets internal err_ upon sucesss or failure.
     * @return true if successful, false if failed.
     */
    bool init() {
        // Bus DNE, or uninitialized:
        if (!bus_ || !bus_->isInitialized()) {
            // Set Invalid state due to the bus being invalid or uninitialized:
            err_ =  ESP_ERR_INVALID_STATE;
            initialized_ = false;
        }

        // Forbid re-initialization (consider I2C devices crashing?):
        else if (address_ > 0x7F || device_scl_freq_ == 0) {
            err_ = ESP_ERR_INVALID_ARG;
            initialized_ = false;
        }
        else if (device_handle_ == nullptr && !initialized_) {
            // Define the I2C config:
            i2c_device_config_t device_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = address_,
                .scl_speed_hz = device_scl_freq_,
                .scl_wait_us = 0,
                .flags = { .disable_ack_check = false}
            };

            // Register the device, set error, and check:
            err_ = i2c_master_bus_add_device(bus_->getI2CBus(), &device_cfg, &device_handle_); 
            if (err_ == ESP_OK) {
                initialized_ = true;
            } else {
                initialized_ = false;
            }
        }

        return initialized_;
    }

    /**
     * @brief Deinitializes the I2C device, allowing for re-initialization at a later time without destruction.
     */
    void deinit() {
        if (device_handle_ != nullptr) {
            i2c_master_bus_rm_device(device_handle_);
            device_handle_ = nullptr;
        }
        initialized_ = false;
    }

    
    /**
     * @brief Writes a raw buffer to the device (useful for commands without registers).
     */
    bool write(const uint8_t* data, size_t len) {
        if (!device_handle_ || !initialized_ ) {
            reg_rx_err_ = ESP_ERR_INVALID_STATE;
            return false;
        }
        reg_rx_err_ = i2c_master_transmit(device_handle_, data, len, -1);
        if (reg_rx_err_ == ESP_OK) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * @brief Writes a byte of data to a specific I2C Device's register.
     * @param reg_addr The hex address of the register.
     * @param data The data byte to write.
     * @returns True if the write operation returns ESP_OK, false if an error occurs (check getRXerr() for more info).
     */
    bool writeRegister(const uint8_t& reg_addr, const uint8_t& data) {
        uint8_t write_buf[2] = {reg_addr, data};
        return write(write_buf, 2);
    }

     /**
     * @brief Writes a 16-bit word, most-significant byte first.
     * @param reg_addr The hex address of the register.
     * @param data The 2 data bytes to write.
     * @returns True if the write operation returns ESP_OK, false if an error occurs (check getRXerr() for more info).
     */
    bool writeRegister(const uint8_t& reg_addr, const uint16_t& data) {
        uint8_t write_buf[3] = {
            reg_addr, static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)
        };
        return write(write_buf, 3);
    }


    /**
     * @brief Reads a byte of data from a specific I2C Device's register.
     * @param reg_addr The hex address of the register.
     * @param data The data byte to be written to.
     * @returns True if the read operation returns ESP_OK, false if an error occurs (check getRXerr() for more info).
     */
    bool readRegister(const uint8_t& reg_addr, uint8_t &data) {
        if (!device_handle_ || !initialized_) {
            reg_rx_err_ = ESP_ERR_INVALID_STATE;
            return false;
        }
        reg_rx_err_ = i2c_master_transmit_receive(device_handle_, &reg_addr, 1, &data, 1, -1);
        if (reg_rx_err_ == ESP_OK) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * @brief Reads two bytes of data from a specific I2C Device's register.
     * @param reg_addr The hex address of the register.
     * @param data The 2 data bytes to be written to.
     * @returns True if the read operation returns ESP_OK, false if an error occurs (check getRXerr() for more info).
     */
    bool readRegister(const uint8_t& reg_addr, uint16_t &data) {
        uint8_t read_buf[2];
        if (!device_handle_ || !initialized_) {
            reg_rx_err_ = ESP_ERR_INVALID_STATE;
            return false;
        }
        reg_rx_err_ = i2c_master_transmit_receive(device_handle_, &reg_addr,  1, read_buf, 2, -1);
        if (reg_rx_err_ == ESP_OK) {
            // read from the buffer into data.
            data = (read_buf[0] << 8) | read_buf[1];
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * @brief Reads two bytes of data from a specific I2C Device's register.
     * @param reg_addr The hex address of the register.
     * @param data The 2 data bytes (signed) to be written to.
     * @returns True if the read operation returns ESP_OK, false if an error occurs (check getRXerr() for more info).
     */
    bool readRegister(const uint8_t& reg_addr, int16_t &data) {
        uint8_t read_buf[2];
        if (!device_handle_ || !initialized_) {
            reg_rx_err_ = ESP_ERR_INVALID_STATE;
            return false;
        }
        reg_rx_err_ = i2c_master_transmit_receive(device_handle_, &reg_addr,  1, read_buf, 2, -1);
        if (reg_rx_err_ == ESP_OK) {
            // read from the buffer into data.
            uint16_t raw_data = (read_buf[0] << 8) | read_buf[1];
            data = static_cast<int16_t>(raw_data);
            return true;
        }
        else {
            return false;
        }
    }
    
    


    // Public Getter Methods:
    /**
     * @brief Reads a device-specific register block without interpreting byte order.
     * @param reg_addr Register command, including device-specific increment flags.
     * @param data Destination buffer.
     * @param len Number of bytes to read.
     * @return True on success; check getRXErr() on failure.
     */
    bool readRegisters(uint8_t reg_addr, uint8_t* data, size_t len) {
        if (!device_handle_ || !initialized_) {
            reg_rx_err_ = ESP_ERR_INVALID_STATE;
            return false;
        }
        if (!data || len == 0) {
            reg_rx_err_ = ESP_ERR_INVALID_ARG;
            return false;
        }
        reg_rx_err_ = i2c_master_transmit_receive(
            device_handle_, &reg_addr, 1, data, len, 100
        );
        return reg_rx_err_ == ESP_OK;
    }

    I2CBus* getBus() {return bus_; }
    uint8_t getAddress() const { return address_; }
    uint32_t getFreq() const { return device_scl_freq_; }
    bool isInitialized() const { return initialized_; }
    esp_err_t getErr() const { return err_; }
    esp_err_t getRXErr() const { return reg_rx_err_; }
    

private:
    I2CBus* bus_;
    const uint8_t address_;
    const uint32_t device_scl_freq_;
    bool initialized_ = false;
    esp_err_t err_ = ESP_OK;
    esp_err_t reg_rx_err_ = ESP_OK;
    i2c_master_dev_handle_t device_handle_ = nullptr;

};

}
}
}

#endif
