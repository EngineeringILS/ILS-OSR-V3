#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <i2c_driver.hpp>


#ifndef LUNABOTICS_INA3221_DRIVER_HPP
#define LUNABOTICS_INA3221_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;
using Resistance = Units::QuantityD<Units::Ohms>;

/**
 * @brief A wrapper class that aligns the INA3221 driver witxh ILS OSR V3 standards as a SensorInterface.
 * 
 * This class implements the SensorInterface and allows data reading capabilities and lifecycle functions for a INA3221 multi-channel power measurement sensor.
 * 
 * @note This class may have updates to introduce new methods as needed (e.g. deep sleep), but the core functionality should stay the same.
 */
class INA3221 : public I2CDevice, public SensorInterface {
public: 

    // Constructor and Destructor:
    /**
     * @brief Constructor for the INA3221.
     * 
     * @param host The processor this driver is running on.
     * @param I2C_Config The platform specific protocols::I2CConfig for I2C operation.
     */
    explicit INA3221(const uint8_t& addr, 
                    I2CBus* i2c_bus, 
                    Resistance channel_1_shunt = Units::ohms(0.05),
                    Resistance channel_2_shunt = Units::ohms(0.05),
                    Resistance channel_3_shunt = Units::ohms(0.05)
                ) : 
                I2CDevice(addr, i2c_bus), 
                SensorInterface(Protocols::InterfaceType::I2C, SensorInterface::HostController::ESP32),
                channel_1_shunt_ohms_(channel_1_shunt),
                channel_2_shunt_ohms_(channel_2_shunt),
                channel_3_shunt_ohms_(channel_3_shunt) 
            {}

    /**
     * @brief Virtual Destructor, both I2CDevice and SensorInterface posess suitable virtual destuctors which safely handle memory.
     */
    virtual ~INA3221() = default;

    // Implementation of the pure virtual functions:

    /**
     * @brief Initializes the INA3221.
     * @return True on sucessful initialiation, False on failed initilization, check getErr().
     */

    bool init() override;

    /**
     * @brief Reads Sensor Data into memory.
     * This function queries the I2C Device Data
     * and stores it in the private power_channel_data_ variable.
     * @return May return true or false.
     */ 
    bool read();
    
    // Sensor Specific Public Functions:
    void getData(DataTypes::INA3221Data& data) const;
     
private:
    // Private Member Variables:

    /**
     * @brief Internal storage for the last-read sensor data.
     */
    DataTypes::INA3221Data power_channel_data_;

    Units::QuantityD<Units::Ohms> channel_1_shunt_ohms_;
    Units::QuantityD<Units::Ohms> channel_2_shunt_ohms_;
    Units::QuantityD<Units::Ohms> channel_3_shunt_ohms_;


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

    // Constants needed by init():
    static constexpr uint16_t EXPECTED_MANUFACTURER_ID = 0x5449;
    static constexpr uint16_t EXPECTED_DIE_ID          = 0x3220;
    static constexpr uint16_t CONFIG_POWER_DOWN        = 0x7120;
    static constexpr uint16_t CONFIG_BUS_SINGLE_SHOT   = 0x7122;
    static constexpr uint16_t CONFIG_CONTINUOUS        = 0x7127;

    /**
     * @brief Register helper function (converts scoped enum to true uint8_t).
     * @param reg The register to convert to a uint8_t.
     * @returns uint8_t reg.
     */
    static constexpr uint8_t reg(DataRegisters reg) {
        return static_cast<uint8_t>(reg);
    }
    
    /**
     * @brief Helper function to convert calculate channel conversion.
     * @param shunt_register The shunt-voltage measurement
     * @param bus_register The bus-voltage measurement
     * @param shunt_Resistance The shunt resistance
     * @param channel_data The channel data object
     */
    bool readChannel(
        DataRegisters shunt_register,
        DataRegisters bus_register,
        const Resistance& shunt_resistance,
        DataTypes::PowerChannelData& channel_data
    );

};
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics

#endif