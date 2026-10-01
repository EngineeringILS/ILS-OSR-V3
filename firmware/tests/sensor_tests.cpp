#include <ina3221.hpp>
#include <Max1704x.hpp>
#include <SerialIO.hpp>
#include <common/drivers/PeripheralInterface.hpp>
#include <cmath>
#include <iostream>
#include <type_traits>

using namespace Lunabotics::Common;
using namespace Lunabotics::ESP32::Drivers;

static_assert(!std::is_copy_constructible<I2CBus>::value);
static_assert(!std::is_copy_assignable<I2CBus>::value);
static_assert(!std::is_copy_constructible<I2CDevice>::value);
using PeripheralState = Sensors::PeripheralInterface::PeripheralState;

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
    return 1; } } while (false)

int main() {
    Protocols::I2CPort port{42, 41, 0, 100000, true};
    {
        I2CBus bus(port);
        CHECK(bus.isInitialized());
        I2CDevice invalid(0x80, &bus);
        CHECK(!invalid.isInitialized());
        uint16_t value = 0;
        CHECK(!invalid.readRegister(0, value));
        CHECK(invalid.getRXErr() == ESP_ERR_INVALID_STATE);

        // Distinct channel values catch channel-selection and conversion errors.
        TestI2C::registers[0xFE] = 0x5449;
        TestI2C::registers[0xFF] = 0x3220;
        TestI2C::registers[1] = 80;
        TestI2C::registers[2] = 8000;
        TestI2C::registers[3] = static_cast<uint16_t>(-80);
        TestI2C::registers[4] = 16000;
        TestI2C::registers[5] = 160;
        TestI2C::registers[6] = 24000;
        INA3221 power(0x40, &bus);
        CHECK(power.init());
        CHECK(TestI2C::written[0] == 0 && TestI2C::written[1] == 0x71 && TestI2C::written[2] == 0x27);
        CHECK(power.read());
        DataTypes::INA3221Data sample;
        power.getData(sample);
        CHECK(std::abs(sample.channel_1.bus_voltage.in(Units::volts) - 8.0) < 1e-9);
        CHECK(std::abs(sample.channel_2.current.in(Units::amperes) + 0.008) < 1e-9);
        CHECK(std::abs(sample.channel_3.power.in(Units::watts) - 0.384) < 1e-9);
        const auto timestamp = sample.timestamp;
        TestI2C::fail_register = 5;
        CHECK(!power.read());
        power.getData(sample);
        CHECK(sample.timestamp == timestamp);
        TestI2C::fail_register = -1;
        CHECK(power.read());

        // A failed battery read must preserve the sample and permit recovery.
        Max1704x battery(0x36, &bus);
        CHECK(battery.init());
        CHECK(battery.read());
        DataTypes::BatteryData before, after;
        battery.getData(before);
        TestI2C::registers[2] = 0;
        TestI2C::fail_register = 4;
        CHECK(!battery.read());
        battery.getData(after);
        CHECK(after.voltage == before.voltage);
        TestI2C::fail_register = -1;
        CHECK(battery.read());
        CHECK(battery.getState() == Sensors::SensorInterface::SensorState::CONNECTED);
    }
    CHECK(TestI2C::buses == 0 && TestI2C::devices == 0);
    {
        SerialIO invalid(1024, 0);
        CHECK(invalid.init() == ESP_ERR_INVALID_ARG);
        SerialIO terminal;
        CHECK(terminal.init() == ESP_OK);
        CHECK(terminal.init() == ESP_OK);
        CHECK(TestSerial::installations == 1);
        const std::string message(512, 'x');
        terminal.serial_out(message);
        CHECK(TestSerial::output == message);
    }
    CHECK(TestSerial::installations == 0);
    std::cout << "Sensor recovery, register conversion, and resource tests passed.\n";
}
