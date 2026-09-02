#include "Neopixel.hpp"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

Neopixel::~Neopixel() {
    deinit();
}

bool Neopixel::configurePower() {
    if (!config_.has_power_pin) {
        return true;
    }

    const gpio_num_t power_pin =
        static_cast<gpio_num_t>(config_.power.gpio_pin);

    if (!GPIO_IS_VALID_OUTPUT_GPIO(power_pin) ||
        config_.power.gpio_pin == config_.data.gpio_pin) {
        setErr(ESP_ERR_INVALID_ARG);
        return false;
    }

    gpio_config_t power_config = {};
    power_config.pin_bit_mask = 1ULL << config_.power.gpio_pin;
    power_config.mode = GPIO_MODE_OUTPUT;
    power_config.pull_up_en = GPIO_PULLUP_DISABLE;
    power_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_config.intr_type = GPIO_INTR_DISABLE;

    setErr(gpio_config(&power_config));
    if (getErr() != ESP_OK) {
        return false;
    }

    power_initialized_ = true;
    setErr(gpio_set_level(
        power_pin,
        config_.power_active_high ? 1 : 0
    ));

    if (getErr() != ESP_OK) {
        gpio_reset_pin(power_pin);
        power_initialized_ = false;
        return false;
    }

    return true;
}

void Neopixel::disablePower() {
    if (!power_initialized_) {
        return;
    }

    gpio_set_level(
        static_cast<gpio_num_t>(config_.power.gpio_pin),
        config_.power_active_high ? 0 : 1
    );
    gpio_reset_pin(static_cast<gpio_num_t>(config_.power.gpio_pin));
    power_initialized_ = false;
}

bool Neopixel::init() {
    if (_State == SensorState::CONNECTED) {
        return true;
    }

    const gpio_num_t data_pin =
        static_cast<gpio_num_t>(config_.data.gpio_pin);

    if (!GPIO_IS_VALID_OUTPUT_GPIO(data_pin) ||
        config_.pixel_count == 0) {
        setErr(ESP_ERR_INVALID_ARG);
        _State = SensorState::FAILED;
        return false;
    }

    _State = SensorState::INITIALIZED;

    if (!configurePower()) {
        _State = SensorState::FAILED;
        return false;
    }

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = config_.data.gpio_pin;
    strip_config.max_leds = config_.pixel_count;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format =
        LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.flags.invert_out = config_.invert_out;

    led_strip_spi_config_t spi_config = {};
    spi_config.clk_src = SPI_CLK_SRC_DEFAULT;
    spi_config.spi_bus = config_.spi_host;
    spi_config.flags.with_dma = config_.with_dma;

    setErr(led_strip_new_spi_device(
        &strip_config,
        &spi_config,
        &strip_handle_
    ));

    if (getErr() != ESP_OK) {
        strip_handle_ = nullptr;
        disablePower();
        _State = SensorState::FAILED;
        return false;
    }

    setErr(led_strip_clear(strip_handle_));
    if (getErr() != ESP_OK) {
        led_strip_del(strip_handle_);
        strip_handle_ = nullptr;
        disablePower();
        _State = SensorState::FAILED;
        return false;
    }

    setLogicalState(false);
    _State = SensorState::CONNECTED;
    return true;
}

void Neopixel::deinit() {
    LED::deinit();

    if (strip_handle_ != nullptr) {
        led_strip_clear(strip_handle_);
        led_strip_del(strip_handle_);
        strip_handle_ = nullptr;
    }

    disablePower();
    setLogicalState(false);
    _State = SensorState::UNINITIALIZED;
}

bool Neopixel::writeColors() {
    if (strip_handle_ == nullptr) {
        setErr(ESP_ERR_INVALID_STATE);
        _State = SensorState::ERROR;
        return false;
    }

    for (size_t index = 0; index < colors_.size(); index++) {
        const NeopixelColor &color = colors_[index];
        setErr(led_strip_set_pixel(
            strip_handle_,
            index,
            color.red,
            color.green,
            color.blue
        ));

        if (getErr() != ESP_OK) {
            _State = SensorState::ERROR;
            return false;
        }
    }

    setErr(led_strip_refresh(strip_handle_));
    if (getErr() != ESP_OK) {
        _State = SensorState::ERROR;
        return false;
    }

    return true;
}

bool Neopixel::setLevel(const bool &is_on) {
    if (strip_handle_ == nullptr) {
        setErr(ESP_ERR_INVALID_STATE);
        _State = SensorState::ERROR;
        return false;
    }

    if (is_on) {
        if (!writeColors()) {
            return false;
        }
    } else {
        setErr(led_strip_clear(strip_handle_));
        if (getErr() != ESP_OK) {
            _State = SensorState::ERROR;
            return false;
        }
    }

    setLogicalState(is_on);
    _State = SensorState::CONNECTED;
    return true;
}

bool Neopixel::setPixelColor(
    const size_t &index,
    const uint8_t &red,
    const uint8_t &green,
    const uint8_t &blue
) {
    if (index >= colors_.size()) {
        setErr(ESP_ERR_INVALID_ARG);
        return false;
    }

    colors_[index] = {red, green, blue};

    if (isOn()) {
        return writeColors();
    }

    return true;
}

bool Neopixel::setColor(
    const uint8_t &red,
    const uint8_t &green,
    const uint8_t &blue
) {
    for (NeopixelColor &color : colors_) {
        color = {red, green, blue};
    }

    if (isOn()) {
        return writeColors();
    }

    return true;
}

bool Neopixel::getPixelColor(
    const size_t &index,
    NeopixelColor &color
) const {
    if (index >= colors_.size()) {
        return false;
    }

    color = colors_[index];
    return true;
}

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics
