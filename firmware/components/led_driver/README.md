# LED Driver

Generic ESP32-S3 GPIO LED driver implemented as a Lunabotics `PeripheralInterface`.

## Features

- Active-high and active-low LEDs
- Direct on/off control
- Nonblocking periodic blink using `esp_timer`
- Derived SPI-backed WS2812 NeoPixel support
- Optional NeoPixel power-pin control
- ESP-IDF error and peripheral state reporting

## GPIO LED Example

```cpp
#include <LED.hpp>

using namespace Lunabotics::ESP32::Drivers;

Lunabotics::Common::Protocols::GPIOConfig led_config{
    .gpio_pin = 13
};

LED led(led_config);

if (led.init()) {
    led.blink(500);
}
```

`blink()` returns immediately. The supplied interval is the time between LED
state changes, so `blink(500)` produces a one-second complete on/off cycle.

## NeoPixel Example

The Adafruit ESP32-S3 TFT Feather uses GPIO 33 for NeoPixel data and GPIO 34
for NeoPixel power:

```cpp
#include <Neopixel.hpp>

using namespace Lunabotics::ESP32::Drivers;

NeopixelConfig neopixel_config{
    .data = {.gpio_pin = 33},
    .pixel_count = 1,
    .spi_host = SPI2_HOST,
    .with_dma = true,
    .invert_out = false,
    .has_power_pin = true,
    .power = {.gpio_pin = 34},
    .power_active_high = true
};

Neopixel neopixel(neopixel_config);

if (neopixel.init()) {
    neopixel.setColor(32, 0, 32);
    neopixel.on();
    neopixel.blink(500);
}
```

The SPI LED-strip backend uses only MOSI for the NeoPixel waveform, but it
reserves the complete selected SPI bus. Do not place another device on the
same SPI host.
