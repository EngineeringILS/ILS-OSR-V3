#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <i2c_driver.hpp>
#include <max1704x.h>

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;


/**
 * @brief A wrapper class that aligns the Max1704x driver with Lunabotics standards as a SensorInterface.
 * 
 * This class implements the SensorInterface and allows data reading capabilities and lifecycle functions for a Max1704x fuel-gauge sensor.
 * 
 * @note This class may have updates to introduce new methods as needed (e.g. deep sleep), but the core functionality should stay the same.
 */
class Max1704x : public SensorInterface {
public: 

     // Constructor and Destructor:
    /**
     * @brief Constructor for the max1704x.
     * 
     * @param host The processor this driver is running on.
     * @param I2C_Config The platform specific protocols::I2CConfig for I2C operation.
     */
    explicit Max1704x(const uint8_t& addr, I2CBus* I2C_Config) : SensorInterface(Protocols::InterfaceType::I2C, SensorInterface::HostController::ESP32), address_(addr), I2Cbus_(I2C_Config) {}

    /**
     * @brief Virtual Destructor.
     */
    virtual ~Max1704x();

    // Implementation of the pure virtual functions:

    /**
     * @brief "Initializes" the max1704x.
     * @return May return true or false.
     */

    bool init() override;

    // TODO: The base SensorInterface.hpp does not yet have a standardized read() function; however, it is planned that it WILL, so this would need to become an override in future releases!!
    /**
     * @brief Reads (TODO: DATA??).
     * This function updates the current (TODO: DATA??)
     * and stores it in the private _batteryData variable.
     * @return May return true or false.
     */ 
    bool read();
    
    // Sensor Specific Public Functions:
    

    void getData(DataTypes::BatteryData& data) const;
     
private:
    // Private Member Variables:

    /**
     * @brief Internal storage for the last-read sensor data.
     */
    DataTypes::BatteryData _batteryData;

    /**
     * @brief I2C Address:
     */
    uint8_t address_;

    /**
     * @brief Internal storage for the I2C Configuration.
     */
    I2CBus* I2Cbus_;

    /**
     * @brief the Max1704x Device Descriptor.
     */
    max1704x_t max1704x_;

    // Private Member Functions:
    esp_err_t max1704x_init_desc_mod();
};
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics