#ifndef LUNABOTICS_DATATYPES_HPP_
#define LUNABOTICS_DATATYPES_HPP_

#include <common/drivers/Units.hpp>
#include <chrono>

namespace Lunabotics {
namespace Common {
/**
 * @brief Provides the major data types with unit standardization for all downstream project components.
 * @note It is planned to migrate the datatypes for specific devices into their specific drivers 
 * at a later date, the migration should be straightforward.
 */
namespace DataTypes {

    // Define a standard clock system for all timestamps and derived sensors/motors using std::chrono:
    using LocalClock = std::chrono::steady_clock;
    using Timestamp  = std::chrono::time_point<LocalClock>;

    /**
     * @brief Struct that provides a 3D float vector for Linear Acceleration (m/s^2) and  Time (s), with a std::chrono based timestamp.
     * 
     * Units::Acceleration derived from au::meters / au::squared(au::second).
     */
    struct LinearAcceleration {
        Units::QuantityD<Units::Acceleration> a_x = Units::ZERO;
        Units::QuantityD<Units::Acceleration> a_y = Units::ZERO;
        Units::QuantityD<Units::Acceleration> a_z = Units::ZERO;
        Timestamp                             timestamp;
    };

    /**
     * @brief Struct that provides a 3D float vector for Linear Velocity (m/s) and Time (s), with a std::chrono based timestamp.
     * 
     * Units::Velocity is dervied from au::meters / au::second.
     */
    struct LinearVelocity {
        Units::QuantityD<Units::Velocity> v_x = Units::ZERO;
        Units::QuantityD<Units::Velocity> v_y = Units::ZERO;
        Units::QuantityD<Units::Velocity> v_z = Units::ZERO;
        Timestamp                             timestamp;
    };

    /**
     * @brief Struct that provides a 3D float vector for Linear Position (m) and Time (s), with a std::chrono based timestamp.
     * 
     * Units::Position is dervied from au::meters.
     */
    struct LinearPosition {
        Units::QuantityD<Units::Position> r_x = Units::ZERO;
        Units::QuantityD<Units::Position> r_y = Units::ZERO;
        Units::QuantityD<Units::Position> r_z = Units::ZERO;
        Timestamp                             timestamp;
    };

    /**
     * @brief Struct that provides the Voltage (V) and Percent (%) of a Battery:
     * @note May be expanded as required.
     */
    struct BatteryData {
        Units::QuantityD<Units::Volts>   voltage = Units::ZERO;
        Units::QuantityD<Units::Percent> percent = Units::ZERO;
        // TODO: Add more fields as needed, e.g. current, temperature, etc.

        // Simple Flags:
        bool is_charging = false;
        bool is_low = false;
    };

    /**
     * @brief Struct that provides voltage, shunt voltage, current, and power
     * data for one INA3221 measurement channel.
     */
    struct PowerChannelData {
        Units::QuantityD<Units::Volts>   bus_voltage   = Units::ZERO;
        Units::QuantityD<Units::Volts>   shunt_voltage = Units::ZERO;
        Units::QuantityD<Units::Amperes> current       = Units::ZERO;
        Units::QuantityD<Units::Watts>   power         = Units::ZERO;
    };

    /**
     * @brief Struct that provides power-monitoring data for all three fixed
     * INA3221 measurement channels, together with a timestamp and status flags.
     */
    struct INA3221Data {
        PowerChannelData channel_1;
        PowerChannelData channel_2;
        PowerChannelData channel_3;

        Timestamp timestamp;
        uint16_t  flags = 0;
    };
}
}
}
#endif