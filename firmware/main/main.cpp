#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <SerialIO.hpp>
#include <i2c_driver.hpp>
#include <i2c_tools.hpp>
#include <Max1704x_test.hpp>
#include <Max1704x.hpp>
#include <ina3221.hpp>
#include <ina3221_test.hpp>
#include <Platforms.hpp>
#include <driver/gpio.h>
#include <stdexcept>


using namespace Lunabotics::Common::Sensors;
using namespace Lunabotics::Common::DataTypes;
using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;
using SI = SensorInterface;
#define TFT_I2C_POWER_GPIO 21



extern "C" {
    void app_main(void);
};

void app_main(void) {
    // 1. Setup Hardware
    Boards::FeatherS3TFT board;
    board.enableI2C();
    // gpio_set_direction(gpio_num_t(21), GPIO_MODE_OUTPUT);
    // gpio_set_level(gpio_num_t(21), 1);
    gpio_set_direction(gpio_num_t(13), GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t(13), 1);

    Protocols::I2CPort i2cPort0;
    // Get I2C Port 0 from the board (Port 0 exists on FeatherS3)
    board.I2C(0, i2cPort0);

    // Attempt to initialize the driver:
    Drivers::I2CBus i2cBus0(i2cPort0);
    
    // Attempt to initialize the max1704x:
    // Drivers::I2CDevice max1704x(0x36, &i2cBus0); 
    Max1704x max1704x(0x36,&i2cBus0);
    max1704x.init();

    INA3221 ina3221(0x40, &i2cBus0);
    ina3221.init();

    // 2. Setup Terminal
    SerialIO Terminal;
    Terminal.init();
    
    // 3. User Interaction Loop
    std::string ioMsg;
    ioMsg.reserve(512);
    ioMsg = "[TEST START] System Ready. \n";
    Terminal.serial_out(ioMsg);
    
    while (true) {
        ioMsg = "Test I/O > 'check', 'scan', 'dump', 'checkread', 'read', or 'q' to quit: \n";
        Terminal.serial_out(ioMsg);
        ioMsg = "Input: ";
        ioMsg = Terminal.serial_in(ioMsg);

        if (ioMsg == "check") {
            i2c_status(Terminal, i2cBus0);
            i2c_device_status(Terminal, max1704x);
        }
        else if (ioMsg == "scan") {
            i2c_scan(Terminal, i2cBus0);
        }
        else if (ioMsg.size() >= 4 && ioMsg.substr(0, 4) == "dump") {
            // Default to MAX17048
            uint8_t targetAddr = 0x36;

            if (ioMsg.size() > 5) {
                std::string arg = ioMsg.substr(5);
                
                // --- REPLACEMENT LOGIC START ---
                char* endPtr;
                // strtoul(string, end_pointer, base 0 for auto-detect)
                unsigned long val = strtoul(arg.c_str(), &endPtr, 0);

                // check if conversion failed:
                // 1. endPtr == arg.c_str() -> No digits found
                // 2. *endPtr != '\0'       -> Junk characters at end (e.g. "0x36xyz")
                // 3. val > 255             -> Address too big for I2C
                if (endPtr == arg.c_str() || *endPtr != '\0' || val > 255) {
                    Terminal.serial_out("Invalid address. Usage: dump <hex|dec>\n");
                    return; // Changed from continue if inside a void function
                }
                
                targetAddr = static_cast<uint8_t>(val);
                // --- REPLACEMENT LOGIC END ---
            }

            i2c_dump(Terminal, i2cBus0, targetAddr, 1);
        } else if (ioMsg == "checkread") {
            uint8_t addresses[16] = {0x02, 0x04, 0x06, 0x08, 0x0C, 0x14, 0x16, 0x18, 0x1A};
            uint8_t numAddresses = 9;
            i2c_device_read(Terminal, max1704x, addresses, numAddresses);
        } else if (ioMsg == "read") {
            max1704x_test_data(Terminal, max1704x);
            ina3221_test_data(Terminal, ina3221);
        }
        else if (ioMsg == "q") {
            Terminal.serial_out("[TEST END] Quitting...\n");
            break; // Exit the loop
        }
        else {
            Terminal.serial_out("Echo: " + ioMsg + "\n");
        }
        
        // Small delay to keep the terminal responsive but not spammy
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 4. Cleanup
    Terminal.deinit();
    
    // In a real RTOS app, app_main should not return, but for a test, this is fine.
    // Ideally, delete the tasks or loop forever here.
    while(1) { vTaskDelay(1000); }
}

// OLD TEST INIT: 
//  FakeIMU imu(SI::HostController::ESP32);
// imu.init();
// OLD TEST LOOP:
// while (true) {
//         Terminal.serial_out("Test I/O (Enter any message, 'q' to quit, 'start' to collect 10 data samples) \n");
//         ioMsg = Terminal.serial_in("Input: ");
//         Terminal.serial_out(ioMsg + "\n");
//         if (ioMsg == "start") {
//             for (size_t i = 0; i < 100; i++) {
//                 if (!imu.read()) {
//                     Terminal.serial_out("[FAIL] Failed to read sensor! \n");
//                 } else {
//                     imu.getData(imu_data);
//                     float a_x = imu_data.a_x.in(au::meters / (au::seconds * au::seconds));
//                     float a_y = imu_data.a_y.in(au::meters / (au::seconds * au::seconds));
//                     float a_z = imu_data.a_z.in(au::meters / (au::seconds * au::seconds));
//                     float timestamp = std::chrono::duration<float>(imu_data.timestamp.time_since_epoch()).count();
//                     std::string a_xStr  = std::to_string(a_x) + ", ";
//                     std::string a_yStr  = std::to_string(a_y) + ", ";
//                     std::string a_zStr  = std::to_string(a_z) + "] (m/s^2)";
//                     std::string timeStr = std::to_string(timestamp) + "(s), ";

//                     Terminal.serial_out(timeStr + "[" + a_xStr + a_yStr + a_zStr + "\n");
//                 }
//             }
            
            
//         }
//         if (ioMsg == "q") {
//             Terminal.serial_out("[TEST END] \n");
//             break;
//         }
//     }