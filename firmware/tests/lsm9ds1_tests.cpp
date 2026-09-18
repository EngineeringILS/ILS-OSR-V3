#include <lsm9ds1.hpp>
#include <cmath>
#include <iostream>

using namespace Lunabotics::Common;
using namespace Lunabotics::ESP32::Drivers;
using State = Sensors::SensorInterface::SensorState;

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
    return 1; } } while (false)

bool near(double actual, double expected) { return std::abs(actual - expected) < 1e-9; }

void word(uint8_t address, uint8_t reg, int16_t value) {
    const uint16_t raw = static_cast<uint16_t>(value);
    TestI2C::banks[address][reg] = raw & 0xFF;
    TestI2C::banks[address][reg + 1] = raw >> 8;
}

void sample(uint8_t ag, uint8_t mag) {
    TestI2C::banks[ag][0x17] = 7;
    TestI2C::banks[mag][0x27] = 8;
    word(ag, 0x28, 1000); word(ag, 0x2A, -1000); word(ag, 0x2C, -32768);
    word(ag, 0x18, -100); word(ag, 0x1A, 200); word(ag, 0x1C, 32767);
    word(mag, 0x28, 100); word(mag, 0x2A, -200); word(mag, 0x2C, 300);
    word(ag, 0x15, -160);
}

int main() {
    TestI2C::banked = true;
    Protocols::I2CPort port{42, 41, 0, 100000, true};
    {
        I2CBus bus(port);
        for (const uint8_t ag : {0x6A, 0x6B}) {
            for (const uint8_t mag : {0x1C, 0x1E}) {
                TestI2C::banks[ag][0x0F] = 0x68;
                TestI2C::banks[mag][0x0F] = 0x3D;
                LSM9DS1 imu(ag, &bus, mag);
                CHECK(!imu.read());
                CHECK(imu.init());
                CHECK(imu.getState() == State::CONNECTED);
                CHECK(TestI2C::banks[ag][0x22] == 0x44);
                CHECK(TestI2C::banks[mag][0x24] == 0x40);
                CHECK(!imu.read() && imu.getErr() == ESP_ERR_NOT_FINISHED);
                sample(ag, mag);
                CHECK(imu.read());
                DataTypes::LSM9DS1Data data;
                imu.getData(data);
                CHECK(near(data.acceleration.a_x.in(Units::meters / Units::squared(Units::seconds)), 0.59820565));
                CHECK(near(data.acceleration.a_y.in(Units::meters / Units::squared(Units::seconds)), -0.59820565));
                CHECK(near(data.angular_velocity.y.in(Units::radians / Units::seconds), 1.75 * 3.14159265358979323846 / 180));
                CHECK(near(data.magnetic_field.z.in(Units::tesla), 300 * 0.14e-7));
                CHECK(near(data.temperature.in(Units::kelvins), 288.15));
                CHECK(data.acceleration.timestamp == data.timestamp);

                // Every acquisition stage must preserve the previous sample on failure.
                const auto stamp = data.timestamp;
                for (const auto reg : {0x20, 0x17, 0x28, 0x18, 0x15}) {
                    TestI2C::fail_address = ag;
                    TestI2C::fail_register = reg;
                    CHECK(!imu.read());
                    CHECK(imu.getState() == State::ERROR);
                    imu.getData(data);
                    CHECK(data.timestamp == stamp);
                }
                TestI2C::fail_address = mag;
                TestI2C::fail_register = 0x28;
                CHECK(!imu.read());
                imu.getData(data);
                CHECK(data.timestamp == stamp);
                TestI2C::fail_register = -1;
                TestI2C::fail_address = -1;
                CHECK(imu.read());
                CHECK(imu.getState() == State::CONNECTED);

                // Emulate changed hardware ranges to verify conversion decoding.
                constexpr double scales[] = {0.061, 0.732, 0.122, 0.244};
                for (uint8_t range = 0; range < 4; ++range) {
                    TestI2C::banks[ag][0x20] = 0x60 | (range << 3);
                    CHECK(imu.read());
                    imu.getData(data);
                    CHECK(near(data.acceleration.a_x.in(Units::meters / Units::squared(Units::seconds)), scales[range] * 9.80665));
                    TestI2C::banks[mag][0x21] = range << 5;
                    CHECK(imu.read());
                    imu.getData(data);
                    constexpr double magnetic[] = {0.14, 0.29, 0.43, 0.58};
                    CHECK(near(data.magnetic_field.x.in(Units::tesla), 100 * magnetic[range] * 1e-7));
                }
                TestI2C::banks[ag][0x22] = 0x46;
                CHECK(!imu.read() && imu.getErr() == ESP_ERR_INVALID_STATE);
                CHECK(imu.init());
                sample(ag, mag);
                for (const uint8_t range : {0, 1, 3}) {
                    TestI2C::banks[ag][0x10] = 0x60 | (range << 3);
                    CHECK(imu.read());
                    imu.getData(data);
                    constexpr double gyroscope[] = {8.75, 17.5, 0, 70};
                    CHECK(near(data.angular_velocity.x.in(Units::radians / Units::seconds),
                        -100 * gyroscope[range] * 0.001 * 3.14159265358979323846 / 180));
                }
                TestI2C::banks[ag][0x23] = 0x02;
                CHECK(!imu.read() && imu.getErr() == ESP_ERR_INVALID_STATE);
                TestI2C::banks[ag][0x23] = 0;
                TestI2C::banks[mag][0x24] = 0xC0;
                CHECK(!imu.read() && imu.getErr() == ESP_ERR_INVALID_STATE);
                CHECK(imu.init());
            }
        }

        LSM9DS1 imu(0x6B, &bus);
        TestI2C::banks[0x1E][0x0F] = 0;
        CHECK(!imu.init() && imu.getState() == State::FAILED);
        TestI2C::banks[0x1E][0x0F] = 0x3D;
        TestI2C::reset_stuck = true;
        CHECK(!imu.init() && imu.getErr() == ESP_ERR_TIMEOUT);
        TestI2C::reset_stuck = false;
        TestI2C::fail_write_register = 0x24;
        TestI2C::fail_address = 0x1E;
        CHECK(!imu.init() && imu.getState() == State::FAILED);
        CHECK(!imu.read());
        TestI2C::fail_write_register = -1;
        TestI2C::fail_address = -1;
        CHECK(imu.init());

        // Block reads reject invalid buffers before entering the transport.
        I2CDevice raw(0x6B, &bus);
        const auto transactions = TestI2C::transactions;
        CHECK(!raw.readRegisters(0x28, nullptr, 6));
        CHECK(raw.getRXErr() == ESP_ERR_INVALID_ARG);
        CHECK(TestI2C::transactions == transactions);

    }
    CHECK(TestI2C::devices == 0 && TestI2C::buses == 0);
    std::cout << "LSM9DS1 register, conversion, lifecycle, and failure tests passed.\n";
}
