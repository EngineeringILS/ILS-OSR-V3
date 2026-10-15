#include <Platforms.hpp>
#include <i2c_driver.hpp>
#include <i2c_tiny_usb.hpp>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

// I2C breakout firmware for the Feather ESP32-S2 Reverse TFT: exposes I2C0 to the USB host
// as an i2c-tiny-usb adapter, unless the header pins labelled 13 and 12 are shorted.

using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;

extern "C" {
    void app_main(void);
};

namespace {

const char *TAG = "breakout";

// Event bits shared by app_main and the abort watch task.
constexpr EventBits_t JUMPER_CHECKED = BIT0;   // First jumper reading is done.
constexpr EventBits_t JUMPER_SHORTED = BIT1;   // Pins 13 and 12 are shorted; the breakout must stop.
constexpr EventBits_t BREAKOUT_RUNNING = BIT2; // The breakout is connected to USB.

struct AbortWatch {
    gpio_num_t drive;
    gpio_num_t sense;
    EventGroupHandle_t events;
};

/**
 * @brief Tests for a short between the drive and sense pins.
 * The sense pin is pulled against the driven level in each phase, so only a real short follows
 * both levels; a floating, pulled or stuck sense pin reads as open.
 * @param rest_level Level the drive pin is left at, which shows on the LED behind pin 13.
 */
bool pinsShorted(gpio_num_t drive, gpio_num_t sense, int rest_level) {
    gpio_set_pull_mode(sense, GPIO_PULLUP_ONLY);
    gpio_set_level(drive, 0);
    esp_rom_delay_us(20);
    const bool follows_low = gpio_get_level(sense) == 0;

    gpio_set_pull_mode(sense, GPIO_PULLDOWN_ONLY);
    gpio_set_level(drive, 1);
    esp_rom_delay_us(20);
    const bool follows_high = gpio_get_level(sense) == 1;

    gpio_set_level(drive, rest_level);
    return follows_low && follows_high;
}

/**
 * @brief Watches the abort jumper for as long as the firmware runs.
 * Reports the first reading, then any short that lasts 100 ms. The LED behind pin 13 is on while
 * the breakout runs, and blinks once the breakout is aborted, until reset.
 */
void abortWatchTask(void *arg) {
    const AbortWatch &watch = *static_cast<const AbortWatch *>(arg);

    gpio_config_t drive_cfg = {};
    drive_cfg.pin_bit_mask = 1ULL << watch.drive;
    drive_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&drive_cfg);

    gpio_config_t sense_cfg = {};
    sense_cfg.pin_bit_mask = 1ULL << watch.sense;
    sense_cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&sense_cfg);

    // First reading: three samples 10 ms apart must all see the short.
    int hits = 0;
    for (int i = 0; i < 3; i++) {
        hits += pinsShorted(watch.drive, watch.sense, 0) ? 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    bool shorted = (hits == 3);
    xEventGroupSetBits(watch.events, JUMPER_CHECKED | (shorted ? JUMPER_SHORTED : 0));

    // Later, a short must last five samples (100 ms), which rides out a brushed or bouncing jumper.
    int run = 0;
    while (!shorted) {
        vTaskDelay(pdMS_TO_TICKS(20));
        const int rest = (xEventGroupGetBits(watch.events) & BREAKOUT_RUNNING) ? 1 : 0;
        run = pinsShorted(watch.drive, watch.sense, rest) ? run + 1 : 0;
        shorted = (run >= 5);
    }
    xEventGroupSetBits(watch.events, JUMPER_SHORTED);

    gpio_set_pull_mode(watch.sense, GPIO_FLOATING);
    for (int level = 1;; level ^= 1) {
        gpio_set_level(watch.drive, level);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

}

void app_main(void) {
    static Boards::FeatherS2RevTFT board;

    // 1. Abort jumper watch, before anything touches USB.
    static EventGroupHandle_t events = xEventGroupCreate();
    static const AbortWatch watch{
        gpio_num_t(board.breakout_abort_drive.gpio_pin),
        gpio_num_t(board.breakout_abort_sense.gpio_pin),
        events
    };
    xTaskCreate(abortWatchTask, "abort_watch", 2048, (void *)&watch, 2, nullptr);
    xEventGroupWaitBits(events, JUMPER_CHECKED, pdFALSE, pdTRUE, portMAX_DELAY);
    if (xEventGroupGetBits(events) & JUMPER_SHORTED) {
        ESP_LOGW(TAG, "Pins 13 and 12 are shorted: I2C breakout aborted");
        return;
    }

    // 2. Setup Hardware
    board.enableI2C();
    Protocols::I2CPort i2cPort0;
    board.I2C(0, i2cPort0);
    // Static, so the bus and the breakout outlive app_main and the breakout's buffers stay off the main task stack.
    static Drivers::I2CBus i2cBus0(i2cPort0);

    // 3. Expose I2C0 to the USB host.
    static Drivers::I2CTinyUSB breakout(&i2cBus0);
    if (!breakout.init()) {
        ESP_LOGE(TAG, "I2C breakout failed to start: %s", esp_err_to_name(breakout.getErr()));
        return;
    }
    xEventGroupSetBits(events, BREAKOUT_RUNNING);
    ESP_LOGI(TAG, "I2C breakout running. Short pins 13 and 12 to abort it.");

    // 4. A later short disconnects the breakout until reset.
    xEventGroupWaitBits(events, JUMPER_SHORTED, pdFALSE, pdTRUE, portMAX_DELAY);
    breakout.deinit();
    ESP_LOGW(TAG, "Pins 13 and 12 shorted: I2C breakout aborted");
}
