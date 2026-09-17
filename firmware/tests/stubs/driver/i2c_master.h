#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

// Host test substitute: emulate register transactions without ESP32 hardware.
using esp_err_t = int;
using gpio_num_t = int;
using i2c_port_num_t = int;
using i2c_master_bus_handle_t = void*;
using i2c_master_dev_handle_t = void*;
constexpr int ESP_OK = 0;
constexpr int ESP_ERR_NOT_FOUND = 1;
constexpr int ESP_ERR_INVALID_STATE = 2;
constexpr int ESP_ERR_INVALID_ARG = 3;
constexpr int I2C_CLK_SRC_DEFAULT = 0;
constexpr int I2C_ADDR_BIT_LEN_7 = 0;

struct i2c_master_bus_config_t {
    int i2c_port, sda_io_num, scl_io_num, clk_source, glitch_ignore_cnt;
    struct { bool enable_internal_pullup; } flags;
};
struct i2c_device_config_t {
    int dev_addr_length, device_address;
    uint32_t scl_speed_hz, scl_wait_us;
    struct { bool disable_ack_check; } flags;
};

namespace TestI2C {
inline uint16_t registers[256]{};
inline int fail_register = -1;
inline int buses = 0;
inline int devices = 0;
inline uint8_t written[3]{};
}

inline esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t* bus) {
    *bus = &TestI2C::buses;
    ++TestI2C::buses;
    return ESP_OK;
}
inline esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t) {
    --TestI2C::buses;
    return ESP_OK;
}
inline esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t* device) {
    *device = &TestI2C::devices;
    ++TestI2C::devices;
    return ESP_OK;
}
inline esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t) {
    --TestI2C::devices;
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t* data, size_t size, int) {
    if (size > sizeof(TestI2C::written)) return ESP_ERR_INVALID_ARG;
    std::memcpy(TestI2C::written, data, size);
    return ESP_OK;
}
inline esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t* reg, size_t, uint8_t* data, size_t size, int) {
    if (*reg == TestI2C::fail_register) return ESP_ERR_NOT_FOUND;
    const uint16_t value = TestI2C::registers[*reg];
    data[0] = size == 2 ? value >> 8 : value & 0xFF;
    if (size == 2) data[1] = value & 0xFF;
    return ESP_OK;
}
