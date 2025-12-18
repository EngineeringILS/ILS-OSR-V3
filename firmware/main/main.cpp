#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <SerialIO.hpp>
#include <i2c_driver.hpp>
#include <Platforms.hpp>

using namespace Lunabotics::Common::Sensors;
using namespace Lunabotics::Common::DataTypes;
using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;
using SI = SensorInterface;

extern "C" {
    void app_main(void);
};

void app_main(void) {
    // 1. Setup Hardware
    Boards::FeatherS3TFT board;

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
        Terminal.serial_out("Test I/O > Enter 'check' for I2C status or 'q' to quit: \n");
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