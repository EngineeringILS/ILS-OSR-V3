#include "LED.hpp"
#include <driver/spi_master.h>
#include <led_strip.h>
#include <led_strip_spi.h>
#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef LUNABOTICS_NEOPIXEL_DRIVER_HPP
#define LUNABOTICS_NEOPIXEL_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

/**
 * @brief ESP32-S3 SPI-backed NeoPixel configuration.
 */
struct NeopixelConfig {
    Protocols::GPIOConfig data;
    size_t pixel_count = 1;
    spi_host_device_t spi_host = SPI2_HOST;
    bool with_dma = true;
    bool invert_out = false;
    bool has_power_pin = false;
    Protocols::GPIOConfig power{0};
    bool power_active_high = true;
};

/**
 * @brief RGB color for a NeoPixel.
 */
struct NeopixelColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

/**
 * @brief Generic ESP32-S3 NeoPixel driver using the SPI peripheral.
 *
 * This class derives from LED so direct on/off and nonblocking blink behavior
 * can be reused. Turning the NeoPixel back on restores its configured colors.
 *
 * @note The Espressif SPI LED-strip backend reserves the entire selected SPI
 * bus because NeoPixels do not use a chip-select signal.
 */
class Neopixel : public LED {
public:

    // Constructor and Destructor:
    /**
     * @brief Constructor for one NeoPixel or a NeoPixel strip.
     * @param config The data GPIO, pixel count, SPI, and optional power setup.
     */
    explicit Neopixel(const NeopixelConfig &config) :
        LED(config.data),
        config_(config),
        colors_(config.pixel_count, NeopixelColor{16, 16, 16})
    {}

    /**
     * @brief Releases the SPI LED-strip driver and optional power GPIO.
     */
    virtual ~Neopixel();

    /**
     * @brief Initializes the optional power pin and SPI LED-strip backend.
     * @return True on successful initialization, false on failure.
     */
    bool init() override;

    /**
     * @brief Deinitializes the NeoPixel and releases its hardware resources.
     */
    void deinit() override;

    /**
     * @brief Sets one pixel's cached RGB color.
     *
     * If the NeoPixel is on, the new color is sent immediately. If it is off,
     * the color is restored the next time on() or blink() turns it on.
     */
    bool setPixelColor(
        const size_t &index,
        const uint8_t &red,
        const uint8_t &green,
        const uint8_t &blue
    );

    /**
     * @brief Sets every pixel to the same RGB color.
     */
    bool setColor(
        const uint8_t &red,
        const uint8_t &green,
        const uint8_t &blue
    );

    /**
     * @brief Gets the cached color for one pixel.
     */
    bool getPixelColor(
        const size_t &index,
        NeopixelColor &color
    ) const;

    /**
     * @brief Gets the number of configured pixels.
     */
    size_t getPixelCount() const { return config_.pixel_count; }

protected:
    bool setLevel(const bool &is_on) override;

private:
    bool writeColors();
    bool configurePower();
    void disablePower();

    const NeopixelConfig config_;
    std::vector<NeopixelColor> colors_;
    led_strip_handle_t strip_handle_ = nullptr;
    bool power_initialized_ = false;
};

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics

#endif
