#include <SerialIO.hpp>
#include <freertos/task.h>
#include <iostream>
#include <stdexcept>

extern "C" void app_main();
struct Finished {};
void require(bool condition) { if (!condition) throw std::runtime_error("app protocol mismatch"); }
void expect(const std::string& output) { require(TestSerial::output == output); TestSerial::output.clear(); }
void expect_sample(const std::string& prefix) {
    require(TestSerial::output.compare(0, prefix.size(), prefix) == 0);
    require(TestSerial::output.back() == '\n');
    TestSerial::output.clear();
}
void ready() { TestI2C::banks[0x6B][0x17] = 7; TestI2C::banks[0x1E][0x27] = 8; }
void run() { try { app_main(); } catch (const Finished&) {} TestTask::on_delay = {}; }

int main() {
    try {
        TestI2C::banked = true;
        TestI2C::banks[0x6B][0x0F] = 0x68;
        TestI2C::banks[0x1E][0x0F] = 0x3D;
        int step = 0;
        TestTask::on_delay = [&](unsigned ticks) {
            if (ticks != 10) return; // Ignore sensor reset waits.
            switch (++step) {
            case 1: expect(""); ready(); break;
            case 2:
                expect_sample("IMU,1,0,0,4294977296,");
                TestSerial::input = "CMD,1,STOP\n";
                break;
            case 3:
                expect("ACK,1,STOP\n");
                TestI2C::banks[0x1E][0x27] = 0;
                TestSerial::input = "CMD,2,READ\n";
                break;
            case 4:
                expect("ERR,2,ESP_ERR_NOT_FINISHED\n");
                ready(); TestSerial::input = "CMD,3,READ\n"; break;
            case 5:
                expect_sample("IMU,1,3,1,4294987296,");
                TestSerial::input = "CMD,4,INIT\n"; break;
            case 6: expect("ACK,4,INIT\n"); ready(); break;
            case 7: expect(""); TestSerial::input = "CMD,5,START\n"; break;
            case 8:
                expect_sample("ACK,5,START\nIMU,1,0,2,4294997296,");
                TestI2C::fail_register = 0x20; break;
            case 9: expect("ERR,0,ESP_ERR_NOT_FOUND\n"); TestI2C::fail_register = -1; break;
            case 10: expect(""); TestSerial::input = "CMD,6,INIT\n"; break;
            case 11: expect("ACK,6,INIT\n"); ready(); TestSerial::input = "CMD,7,START\n"; break;
            case 12: expect_sample("ACK,7,START\nIMU,1,0,3,4295007296,"); throw Finished{};
            }
        };
        run();
        require(step == 12 && TestI2C::buses == 0 && TestI2C::devices == 0);

        // Initialization failure must leave the command interface available for recovery.
        TestI2C::banks[0x1E][0x0F] = 0;
        TestSerial::input = "CMD,1,START\n";
        step = 0;
        TestTask::on_delay = [&](unsigned ticks) {
            if (ticks != 10) return;
            switch (++step) {
            case 1:
                expect("ERR,0,ESP_ERR_NOT_FOUND\nERR,1,ESP_ERR_INVALID_STATE\n");
                TestI2C::banks[0x1E][0x0F] = 0x3D;
                TestSerial::input = "CMD,2,INIT\n"; break;
            case 2: expect("ACK,2,INIT\n"); ready(); TestSerial::input = "CMD,3,START\n"; break;
            case 3: expect_sample("ACK,3,START\nIMU,1,0,0,"); throw Finished{};
            }
        };
        run();
        require(step == 3 && TestSerial::installations == 0 && TestI2C::devices == 0);
    } catch (const std::exception& error) {
        std::cerr << error.what() << ": " << TestSerial::output << '\n';
        return 1;
    }
}
