#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <i2c_driver.hpp>

#ifndef LUNABOTICS_ina3221_DRIVER_HPP
#define LUNABOTICS_ina3221_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;


/**
 * @brief A wrapper class that aligns the ina3221 driver with Lunabotics standards as a SensorInterface.
 * 
 * This class implements the SensorInterface and allows data reading capabilities and lifecycle functions for a ina3221 fuel-gauge sensor.
 * 
 * @note This class may have updates to introduce new methods as needed (e.g. deep sleep), but the core functionality should stay the same.
 */
class ina3221 : public I2CDevice, public SensorInterface {
public: 

    // Constructor and Destructor:
    /**
     * @brief Constructor for the ina3221.
     * 
     * @param host The processor this driver is running on.
     * @param I2C_Config The platform specific protocols::I2CConfig for I2C operation.
     */
    explicit ina3221(const uint8_t& addr, I2CBus* i2c_bus) : I2CDevice(addr, i2c_bus), SensorInterface(Protocols::InterfaceType::I2C, SensorInterface::HostController::ESP32) {}

    /**
     * @brief Virtual Destructor, both I2CDevice and SensorInterface posess suitable virtual destuctors which safely handle memory.
     */
    virtual ~ina3221() = default;

    // Implementation of the pure virtual functions:

    /**
     * @brief Initializes the ina3221.
     * @return True on sucessful initialiation, False on failed initilization, check getErr().
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
    DataTypes::BatteryData battery_data_;


    /**
     * @brief INA3221 8-bit register addresses.
     *
     * All register values are 16 bits wide and transmitted most-significant
     * byte first.
     */
    enum class DataRegisters : uint8_t {
        CONFIGURATION            = 0x00, // RW: Reset, channel enables, averaging, conversion times, and operating mode.
        SHUNT_VOLTAGE_CH1        = 0x01, // RO: Channel 1 differential shunt-voltage measurement.
        BUS_VOLTAGE_CH1          = 0x02, // RO: Channel 1 bus-voltage measurement.
        SHUNT_VOLTAGE_CH2        = 0x03, // RO: Channel 2 differential shunt-voltage measurement.
        BUS_VOLTAGE_CH2          = 0x04, // RO: Channel 2 bus-voltage measurement.
        SHUNT_VOLTAGE_CH3        = 0x05, // RO: Channel 3 differential shunt-voltage measurement.
        BUS_VOLTAGE_CH3          = 0x06, // RO: Channel 3 bus-voltage measurement.
        CRITICAL_ALERT_LIMIT_CH1 = 0x07, // RW: Channel 1 critical shunt-voltage alert threshold.
        WARNING_ALERT_LIMIT_CH1  = 0x08, // RW: Channel 1 warning shunt-voltage alert threshold.
        CRITICAL_ALERT_LIMIT_CH2 = 0x09, // RW: Channel 2 critical shunt-voltage alert threshold.
        WARNING_ALERT_LIMIT_CH2  = 0x0A, // RW: Channel 2 warning shunt-voltage alert threshold.
        CRITICAL_ALERT_LIMIT_CH3 = 0x0B, // RW: Channel 3 critical shunt-voltage alert threshold.
        WARNING_ALERT_LIMIT_CH3  = 0x0C, // RW: Channel 3 warning shunt-voltage alert threshold.
        SHUNT_VOLTAGE_SUM        = 0x0D, // RO: Sum of the enabled shunt-voltage measurement channels.
        SHUNT_VOLTAGE_SUM_LIMIT  = 0x0E, // RW: Critical-alert threshold for the shunt-voltage sum.
        MASK_ENABLE              = 0x0F, // RW: Alert masks, channel summation enables, and status flags.
        POWER_VALID_UPPER_LIMIT  = 0x10, // RW: Upper bus-voltage threshold for the power-valid signal.
        POWER_VALID_LOWER_LIMIT  = 0x11, // RW: Lower bus-voltage threshold for the power-valid signal.
        MANUFACTURER_ID          = 0xFE, // RO: Texas Instruments manufacturer ID; expected value is 0x5449.
        DIE_ID                   = 0xFF  // RO: INA3221 device identification; expected value is 0x3220.
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
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics

#endif