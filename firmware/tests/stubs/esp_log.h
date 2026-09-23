#pragma once
#include "driver/i2c_master.h"
constexpr int ESP_LOG_NONE = 0;
inline void esp_log_level_set(const char*, int) {}
inline const char* esp_err_to_name(esp_err_t error) {
    switch (error) {
    case ESP_OK: return "ESP_OK";
    case ESP_ERR_NOT_FOUND: return "ESP_ERR_NOT_FOUND";
    case ESP_ERR_NOT_FINISHED: return "ESP_ERR_NOT_FINISHED";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    default: return "ESP_FAIL";
    }
}
