#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <max1704x.h>

namespace Lunabotics {
namespace Common {
namespace Sensors {

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
    explicit Max1704x(HostController host, Protocols::I2CConfig I2C_Config) : SensorInterface(Protocols::InterfaceType::I2C, host), _I2CConfig(I2C_Config) {}

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
     * @brief Internal storage for the I2C Configuration.
     */
    Protocols::I2CConfig _I2CConfig;

};
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics