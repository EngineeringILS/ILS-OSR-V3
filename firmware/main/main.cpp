#include <SerialIO.hpp>
#include <Platforms.hpp>
#include <lsm9ds1.hpp>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include "imu_protocol.hpp"

using namespace Lunabotics::Common;
using namespace Lunabotics::ESP32;

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    SerialIO serial;
    if (serial.init() != ESP_OK) return;

    Boards::FeatherS3TFT board;
    Protocols::I2CPort port{};
    if (!board.enableI2C() || !board.I2C(0, port)) {
        serial.serial_out(ImuProtocol::error(0, "ESP_ERR_INVALID_STATE"));
        return;
    }
    Drivers::I2CBus bus(port);
    Drivers::LSM9DS1 imu(0x6B, &bus);
    bool initialized = imu.init();
    bool streaming = initialized;
    uint64_t sequence = 0;
    if (!initialized) serial.serial_out(ImuProtocol::error(0, esp_err_to_name(imu.getErr())));

    auto read_sample = [&](uint32_t id) -> esp_err_t {
        if (!imu.read()) return imu.getErr();
        // Acquisition completion on the MCU clock, not host/UTC time.
        const int64_t timestamp_us = esp_timer_get_time();
        DataTypes::LSM9DS1Data data;
        imu.getData(data);
        serial.serial_out(ImuProtocol::format_sample(data, id, sequence++, timestamp_us));
        return ESP_OK;
    };

    std::string line;
    while (true) {
        const auto result = serial.poll_line(line);
        if (result == LineResult::Overflow) {
            serial.serial_out(ImuProtocol::error(0, "BAD_COMMAND"));
        } else if (result == LineResult::Line) {
            const auto command = ImuProtocol::parse_command(line);
            using ImuProtocol::Action;
            switch (command.action) {
            case Action::Stop:
                streaming = false;
                serial.serial_out(ImuProtocol::ack(command.id, "STOP"));
                break;
            case Action::Start:
                if (initialized) {
                    streaming = true;
                    serial.serial_out(ImuProtocol::ack(command.id, "START"));
                } else {
                    serial.serial_out(ImuProtocol::error(command.id, "ESP_ERR_INVALID_STATE"));
                }
                break;
            case Action::Init:
                streaming = false;
                initialized = imu.init();
                serial.serial_out(initialized ? ImuProtocol::ack(command.id, "INIT") :
                    ImuProtocol::error(command.id, esp_err_to_name(imu.getErr())));
                break;
            case Action::Read: {
                const auto error = read_sample(command.id);
                if (error != ESP_OK) serial.serial_out(ImuProtocol::error(command.id, esp_err_to_name(error)));
                break;
            }
            case Action::Invalid:
                serial.serial_out(ImuProtocol::error(command.id, "BAD_COMMAND"));
                break;
            }
        }
        if (streaming) {
            const auto error = read_sample(0);
            if (error != ESP_OK && error != ESP_ERR_NOT_FINISHED) {
                serial.serial_out(ImuProtocol::error(0, esp_err_to_name(error)));
                streaming = false;
            }
        }
        vTaskDelay(std::max<TickType_t>(pdMS_TO_TICKS(10), 1));
    }
}
