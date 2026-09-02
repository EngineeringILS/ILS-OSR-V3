# LED Driver

Generic ESP32-S3 GPIO LED driver implemented as a Lunabotics `PeripheralInterface`.

## Features

- Active-high and active-low LEDs
- Direct on/off control
- Nonblocking periodic blink using `esp_timer`
- ESP-IDF error and peripheral state reporting

## Example

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
