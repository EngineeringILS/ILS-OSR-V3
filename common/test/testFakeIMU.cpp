#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

// Include the sensor driver to be tested:
#include "common/drivers/FakeIMU.hpp"

// Use the sensor namespace, in this case: Lunabotics::Common::Sensors:
using namespace Lunabotics::Common::Sensors;

int main() {
    // 1. Create the FakeIMU object
    FakeIMU my_imu(SensorInterface::HostController::OBC);

    // 2. Initialize the sensor
    if (!my_imu.init()) {
        std::cerr << "[FAIL] Failed to initialize the FakeIMU!" << std::endl;
        return 1; // Exit with a failure code
    }
    std::cout << "[INFO] FakeIMU initialized." << std::endl;

    // 3. Create a data struct
    FakeIMU::IMUData imu_data;

    // Store data readings to compare changes:
    std::vector<float> ax_readings;
    const int NUM_READINGS = 20; // Run for 200 iterations
    bool data_is_changing = false;

    // 4. Run a FINITE loop
    for (int i = 0; i < NUM_READINGS; ++i) {
        
        // A. Call read()
        if (!my_imu.read()) {
            std::cerr << "[FAIL] Failed to read sensor!" << std::endl;
            return 1; // Exit with failure
        }

        // B. Call getData()
        my_imu.getData(imu_data);
        ax_readings.push_back(imu_data.ax);

        // C. Print the data (optional, but good for debugging)
        // Note: This will spam the test log. You can comment it out.
        std::cout << "ax: " << imu_data.ax << std::endl;
        std::cout << "ay: " << imu_data.ay << std::endl;
        std::cout << "az: " << imu_data.az << std::endl;

        // D. Sleep for 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 5. --- Verification Step ---
    // Check if the data actually changed from the first reading.
    // This validates the sinewave output.
    for (size_t i = 1; i < ax_readings.size(); ++i) {
        if (ax_readings[i] != ax_readings[0]) {
            data_is_changing = true;
            break;
        }
    }

    if (!data_is_changing) {
        std::cerr << "[FAIL] Sensor data did not change. All " 
                  << NUM_READINGS << " readings were identical." << std::endl;
        return 1; // Exit with failure
    }

    // 6. --- Test Pass ---
    std::cout << "[PASS] Test completed. Sensor data was read and verified." << std::endl;
    return 0; // Exit with SUCCESS
}