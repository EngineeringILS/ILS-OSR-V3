#pragma once

#include "driver/i2c_master.h"
#include <cstring>

// Host test substitute: a fixed base MAC address.
enum esp_mac_type_t { ESP_MAC_BASE };
inline esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t) {
    const uint8_t base[6] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56};
    std::memcpy(mac, base, sizeof(base));
    return ESP_OK;
}
