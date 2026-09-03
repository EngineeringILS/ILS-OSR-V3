#include "LED.hpp"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

LED::~LED() {
    deinit();
}

bool LED::init() {
    if (_State == PeripheralState::CONNECTED) {
        return true;
    }

    const gpio_num_t gpio_pin = static_cast<gpio_num_t>(config_.gpio_pin);
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio_pin)) {
        err_ = ESP_ERR_INVALID_ARG;
        _State = PeripheralState::FAILED;
        return false;
    }

    _State = PeripheralState::INITIALIZED;

    gpio_config_t gpio_config_data = {};
    gpio_config_data.pin_bit_mask = 1ULL << config_.gpio_pin;
    gpio_config_data.mode = GPIO_MODE_OUTPUT;
    gpio_config_data.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config_data.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config_data.intr_type = GPIO_INTR_DISABLE;

    err_ = gpio_config(&gpio_config_data);
    if (err_ != ESP_OK) {
        _State = PeripheralState::FAILED;
        return false;
    }
    initialized_ = true;

    err_ = gpio_set_level(gpio_pin, active_high_ ? 0 : 1);
    if (err_ != ESP_OK) {
        gpio_reset_pin(gpio_pin);
        initialized_ = false;
        _State = PeripheralState::FAILED;
        return false;
    }

    is_on_.store(false);
    _State = PeripheralState::CONNECTED;
    return true;
}

void LED::deinit() {
    if (blink_timer_ != nullptr) {
        if (esp_timer_is_active(blink_timer_)) {
            esp_timer_stop(blink_timer_);
        }
        esp_timer_delete(blink_timer_);
        blink_timer_ = nullptr;
    }

    if (initialized_) {
        gpio_reset_pin(static_cast<gpio_num_t>(config_.gpio_pin));
        initialized_ = false;
    }

    is_on_.store(false);
    _State = PeripheralState::UNINITIALIZED;
}

bool LED::setLevel(const bool &is_on) {
    const uint32_t level = (is_on == active_high_) ? 1 : 0;
    err_ = gpio_set_level(
        static_cast<gpio_num_t>(config_.gpio_pin),
        level
    );

    if (err_ != ESP_OK) {
        _State = PeripheralState::ERROR;
        return false;
    }

    is_on_.store(is_on);
    return true;
}

bool LED::on() {
    if (_State != PeripheralState::CONNECTED &&
        _State != PeripheralState::ERROR) {
        return false;
    }

    if (!stopBlink()) {
        return false;
    }

    return setLevel(true);
}

bool LED::off() {
    if (_State != PeripheralState::CONNECTED &&
        _State != PeripheralState::ERROR) {
        return false;
    }

    if (!stopBlink()) {
        return false;
    }

    return setLevel(false);
}

bool LED::blink(const uint32_t &interval_ms) {
    if ((_State != PeripheralState::CONNECTED &&
         _State != PeripheralState::ERROR) ||
        interval_ms == 0) {
        err_ = ESP_ERR_INVALID_ARG;
        return false;
    }

    if (blink_timer_ == nullptr) {
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = &LED::blinkCallback;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "led_blink";
        timer_args.skip_unhandled_events = true;

        err_ = esp_timer_create(&timer_args, &blink_timer_);
        if (err_ != ESP_OK) {
            _State = PeripheralState::ERROR;
            return false;
        }
    }

    if (!stopBlink()) {
        return false;
    }

    err_ = esp_timer_start_periodic(
        blink_timer_,
        static_cast<uint64_t>(interval_ms) * 1000ULL
    );

    if (err_ != ESP_OK) {
        _State = PeripheralState::ERROR;
        return false;
    }

    _State = PeripheralState::CONNECTED;
    return true;
}

bool LED::stopBlink() {
    if (blink_timer_ == nullptr ||
        !esp_timer_is_active(blink_timer_)) {
        return true;
    }

    err_ = esp_timer_stop(blink_timer_);
    if (err_ != ESP_OK) {
        _State = PeripheralState::ERROR;
        return false;
    }

    return true;
}

bool LED::isBlinking() const {
    return blink_timer_ != nullptr &&
        esp_timer_is_active(blink_timer_);
}

void LED::blinkCallback(void* arg) {
    LED* led = static_cast<LED*>(arg);
    if (led != nullptr) {
        led->setLevel(!led->isOn());
    }
}

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics
