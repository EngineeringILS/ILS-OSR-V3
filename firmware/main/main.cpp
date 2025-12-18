#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <SerialIO.hpp>
#include <i2c_driver.hpp>
#include <Platforms.hpp>
#include <driver/gpio.h>

using namespace Lunabotics::Common::Sensors;
using namespace Lunabotics::Common::DataTypes;
using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;
using SI = SensorInterface;
#define TFT_I2C_POWER_GPIO 21

// Helper lambda or function to format hex cleanly
auto to_hex = [](int value) -> std::string {
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%02x", value);
    return std::string(buffer);
};

extern "C" {
    void app_main(void);
};

void app_main(void) {
    // 1. Setup Hardware
    Boards::FeatherS3TFT board;
    gpio_set_direction(gpio_num_t(21), GPIO_MODE_OUTPUT);
    gpio_set_level(gpio_num_t(21), 1);

    Protocols::I2CPort i2cPort0;
    // Get I2C Port 0 from the board (Port 0 exists on FeatherS3)
    board.I2C(0, i2cPort0);

    // Attempt to initialize the driver
    Drivers::I2CBus i2cBus0(i2cPort0);

    // 2. Setup Terminal
    SerialIO Terminal;
    Terminal.init();
    
    // 3. User Interaction Loop
    std::string ioMsg;
    Terminal.serial_out("[TEST START] System Ready. \n");
    
    while (true) {
        Terminal.serial_out("Test I/O > Enter 'check' for I2C status, or 'scan' for I2C scan,  or 'q' to quit: \n");
        ioMsg = Terminal.serial_in("Input: ");

        if (ioMsg == "check") {
            if (i2cBus0.initialized_) {
                Terminal.serial_out("I2C Bus 0 Status: [OK] (Initialized)\n");
            } else {
                // If failed, print the error code for debugging
                std::string errName = esp_err_to_name(i2cBus0.err);
                Terminal.serial_out("I2C Bus 0 Status: [FAIL] Error Code: " + errName + "\n");
            }
        }
        else if (ioMsg == "scan") {
            if (i2cBus0.initialized_) {
                Terminal.serial_out("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
                uint8_t address;
                // Iterate through all 128 possible addresses
                for (int i = 0; i < 128; i += 16) {
                    // Fix: Use the helper to print the Row Header (00:, 10:, etc.)
                    Terminal.serial_out(to_hex(i) + ": ");

                    for (int j = 0; j < 16; j++) {
                        // fflush is usually for stdout/printf, might not affect your Terminal class, 
                        // but harmless to keep if you are using underlying stdio.
                        fflush(stdout); 
                        
                        address = i + j;

                        // Skip reserved addresses (0x00-0x02, 0x78-0x7F) if you want to be strict,
                        // but probing them is usually fine.
                        esp_err_t ret = i2c_master_probe(i2cBus0.getI2CBus(), address, 50);

                        if (ret == ESP_OK) {
                            // Fix: Print the address in Hex, not Decimal
                            Terminal.serial_out(to_hex(address) + " ");
                        } else if (ret == ESP_ERR_TIMEOUT) {
                            Terminal.serial_out("UU "); // Timeout is usually just NACK ("--")
                        } else {
                            Terminal.serial_out("-- ");
                        }
                    }
                    Terminal.serial_out("\r\n");
                }
            }
            else {
                std::string errName = esp_err_to_name(i2cBus0.err);
                Terminal.serial_out("I2C Bus 0 Status: [FAIL] Error Code: " + errName + "\n");
            }
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