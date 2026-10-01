#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <i2c_driver.hpp>

#ifndef LUNABOTICS_MAX1704X_DRIVER_HPP
#define LUNABOTICS_MAX1704X_DRIVER_HPP

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
class Max1704x : public I2CDevice, public SensorInterface {
public: 

     // Constructor and Destructor:
    /**
     * @brief Constructor for the max1704x.
     * 
     * @param addr The 7-bit I2C address.
     * @param i2c_bus The initialized bus, which must outlive this driver.
     */
    explicit Max1704x(const uint8_t& addr, I2CBus* i2c_bus) : I2CDevice(addr, i2c_bus), SensorInterface(Protocols::InterfaceType::I2C, SensorInterface::HostController::ESP32) {}

    /**
     * @brief Releases the device through the I2CDevice destructor.
     */
    virtual ~Max1704x() = default;

    // Implementation of the pure virtual functions:

    /**
     * @brief Initializes the max1704x.
     * @return True on successful initialization, false on failure.
     */

    bool init() override;

    // TODO: The base SensorInterface.hpp does not yet have a standardized read() function; however, it is planned that it WILL, so this would need to become an override in future releases!!
    /**
     * @brief Reads voltage, state of charge, and charging status into battery_data_.
     * @return True on a complete sample, false on failure; the previous sample is retained.
     * @note A subsequent read retries communication after a recoverable error.
     */ 
    bool read();
    
    // Sensor Specific Public Functions:
    

    /** @brief Copies the last complete sample without performing I2C transactions. */
    void getData(DataTypes::BatteryData& data) const;
     
private:
    // Private Member Variables:

    /**
     * @brief Internal storage for the last-read sensor data.
     */
    DataTypes::BatteryData battery_data_;


    /**
     * @brief Hex addresses for each Max1704x Register:
     */
    enum class DataRegisters : uint8_t {
        VCELL = 0x02,   // ADC Measurement of VCELL.
        SOC = 0x04,     // Battery state of charge.
        MODE = 0x06,    // Initiates quick-start, reports hibernate mode, and enables sleep mode.
        VERSION = 0x08, // IC production version.
        HIBRT = 0x0A, // Controls thresholds for entering and exiting hibernate mode.
        CONFIG = 0x0C, // Compensation to optimize performance, sleep mode, alert indicators, and configuration.
        VALRT = 0x14, // Configures the VELL range outside of which alerts are generated.
        CRATE = 0x16, // Approximate charge or discharge rate of the battery.
        VRESET_ID = 0x18, // Configures the VCELL threshold below which the IC resets itself, ID is a one-time facotry programmable identifier.
        STATUS = 0x1A, // Indicates overvoltage, undervoltage, SOC change, SOC low, and reset alerts.
        TABLE_start = 0x40, // Configures battery parameters (start).
        TABLE_end = 0x4F, // Configures battery parameters (end).
        CMD = 0xFE // Sends POR command.
    };

    /**
     * @brief Register helper function (converts scoped enum to true uint8_t).
     * @param reg The register to convert to a uint8_t.
     * @returns uint8_t reg.
     */
    uint8_t reg(DataRegisters reg) {
        return static_cast<uint8_t>(reg);
    }

    // enum class RegisterFeatures : uint8_t {
    //     RO,  // Readonly Flag
    //     WO,  // Writeonly Flag
    //     RW   // Read/Write Flag
    // }
    
};
} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics

#endif
