#include <common/drivers/PeripheralInterface.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <atomic>
#include <cstdint>

#ifndef LUNABOTICS_LED_DRIVER_HPP
#define LUNABOTICS_LED_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;

/**
 * @brief Generic ESP32-S3 GPIO LED driver.
 *
 * This class implements the PeripheralInterface and provides direct on/off
 * control and nonblocking blinking through an ESP timer.
 */
class LED : public PeripheralInterface {
public:

    // Constructor and Destructor:
    /**
     * @brief Constructor for a GPIO LED.
     *
     * @param config The GPIO configuration for the LED.
     * @param active_high True when a high GPIO level turns the LED on.
     */
    explicit LED(
        const Protocols::GPIOConfig &config,
        const bool &active_high = true
    ) :
        PeripheralInterface(
            Protocols::InterfaceType::GPIO,
            PeripheralInterface::HostController::ESP32
        ),
        config_(config),
        active_high_(active_high)
    {}

    /**
     * @brief Stops the blink timer and releases the GPIO.
     */
    virtual ~LED();

    // Rule of 3: Copy Forbid:
    LED(const LED&) = delete;
    LED& operator=(const LED&) = delete;

    // Implementation of the pure virtual functions:

    /**
     * @brief Initializes the LED GPIO as an output and turns the LED off.
     * @return True on successful initialization, false on failure.
     */
    bool init() override;

    /**
     * @brief Deinitializes the LED and releases the GPIO.
     */
    void deinit();

    /**
     * @brief Turns the LED on and stops any active blink timer.
     * @return True on success, false on failure.
     */
    bool on();

    /**
     * @brief Turns the LED off and stops any active blink timer.
     * @return True on success, false on failure.
     */
    bool off();

    /**
     * @brief Starts blinking without blocking the calling task.
     * @param interval_ms Time between each LED state change in milliseconds.
     * @return True if the periodic timer starts successfully, false otherwise.
     */
    bool blink(const uint32_t &interval_ms = 500);

    /**
     * @brief Stops blinking and leaves the LED in its current state.
     * @return True if the timer is stopped or was already inactive.
     */
    bool stopBlink();

    /**
     * @brief Gets the logical LED state.
     */
    bool isOn() const { return is_on_.load(); }

    /**
     * @brief Gets whether the nonblocking blink timer is active.
     */
    bool isBlinking() const;

    /**
     * @brief Gets the most recent ESP-IDF error.
     */
    esp_err_t getErr() const { return err_; }

    /**
     * @brief Gets the LED GPIO configuration.
     */
    Protocols::GPIOConfig getConfig() const { return config_; }

private:
    static void blinkCallback(void* arg);
    bool setLevel(const bool &is_on);

    const Protocols::GPIOConfig config_;
    const bool active_high_;
    std::atomic<bool> is_on_{false};
    bool initialized_ = false;
    esp_timer_handle_t blink_timer_ = nullptr;
    esp_err_t err_ = ESP_OK;
};

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics

#endif
