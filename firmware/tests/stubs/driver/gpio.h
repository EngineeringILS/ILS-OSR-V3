#pragma once
#include "i2c_master.h"
constexpr int GPIO_MODE_OUTPUT = 1;
inline esp_err_t gpio_set_direction(gpio_num_t, int) { return ESP_OK; }
inline esp_err_t gpio_set_level(gpio_num_t, int) { return ESP_OK; }
