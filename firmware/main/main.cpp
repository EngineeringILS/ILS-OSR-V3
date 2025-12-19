#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <SerialIO.hpp>

using namespace Lunabotics::Common::Sensors;
using namespace Lunabotics::Common::DataTypes;
using SI = SensorInterface;

extern "C" {
    void app_main(void);
};

void app_main(void) {
    // Instantiate a new Terminal:
    SerialIO Terminal;
    Terminal.init();
    FakeIMU imu(SI::HostController::ESP32);
    imu.init();
    // Insantiate a looping variable to control the test loop:
    bool inputTest = true;
    // Instantiate a string to hold the input and output for the Terminal:
    std::string ioMsg;
    Terminal.serial_out("[TEST START] \n");
    LinearAcceleration imu_data;
    // Run a simple loop to test functionality:
    while (inputTest) {
        Terminal.serial_out("Test I/O (Enter any message, 'q' to quit, 'start' to collect 10 data samples) \n");
        ioMsg = Terminal.serial_in("Input: ");
        Terminal.serial_out(ioMsg + "\n");
        if (ioMsg == "start") {
            for (size_t i = 0; i < 100; i++) {
                if (!imu.read()) {
                    Terminal.serial_out("[FAIL] Failed to read sensor! \n");
                } else {
                    imu.getData(imu_data);
                    float a_x = imu_data.a_x.in(au::meters / (au::seconds * au::seconds));
                    float a_y = imu_data.a_y.in(au::meters / (au::seconds * au::seconds));
                    float a_z = imu_data.a_z.in(au::meters / (au::seconds * au::seconds));
                    float timestamp = std::chrono::duration<float>(imu_data.timestamp.time_since_epoch()).count();
                    std::string a_xStr  = std::to_string(a_x) + ", ";
                    std::string a_yStr  = std::to_string(a_y) + ", ";
                    std::string a_zStr  = std::to_string(a_z) + "] (m/s^2)";
                    std::string timeStr = std::to_string(timestamp) + "(s), ";

                    Terminal.serial_out(timeStr + "[" + a_xStr + a_yStr + a_zStr + "\n");
                }
            }
            
            
        }
        if (ioMsg == "q") {
            Terminal.serial_out("[TEST END] \n");
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    Terminal.deinit();
}