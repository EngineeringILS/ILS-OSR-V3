#ifndef LUNABOTICS_FAKEIMU_HPP_
#define LUNABOTICS_FAKEIMU_HPP_

#include <common/drivers/SensorInterface.hpp>
#include <chrono>

namespace Lunabotics {
namespace Common {
namespace Sensors{

/**
 * @brief A fake IMU sensor for testing.
 * 
 * This class implements the SensorInterface and generates a simple square wave.
 * It serves as a test driver and a template for new hardware drivers.
 */
class FakeIMU : public SensorInterface {
public:
    // Public Nested Struct for Data:

    /**
     * @brief IMU datastruct that provides a float vector for Linear Acceleration.
     */
    struct IMUData {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;
    };

    // Constructor and Destructor:
    /**
     * @brief Constructor for the FakeIMU.
     * 
     * @param host The processor this driver is running on.
     */
    explicit FakeIMU(HostController host);

    /**
     * @brief Virtual Destructor.
     */
    virtual ~FakeIMU();

    // Implementation of the pure virtual functions:

    /**
     * @brief "Initializes" the fake sensor.
     * @return Always returns true.
     */

    bool init() override;

    // TODO: The base SensorInterface.hpp does not yet have a standardized read() function; however, it is planned that it WILL, so this would need to become an override in future releases!!
    /**
     * @brief "Reads" from the fake sensor.
     * This function updates the internal square wave value
     * and stores it in the private m_data variable.
     * @return Always returns true.
     */ 
    bool read();
    

    // Sensor Specific Public Functions:
    void getData(IMUData& data) const;

private:
    // Private Member Variables:

    /**
     * @brief Internal storage for the last-read sensor data.
     */
    IMUData m_data;

    /**
     * @brief A counter to manage the square wave timing.
     */
    int m_read_counter;
    
    /**
     * @brief Start time for data output calculations:
     */
    std::chrono::time_point<std::chrono::system_clock> m_start_time;
};
       
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics
#endif