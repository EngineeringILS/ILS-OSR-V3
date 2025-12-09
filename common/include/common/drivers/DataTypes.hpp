#ifndef LUNABOTICS_DATATYPES_HPP_
#define LUNABOTICS_DATATYPES_HPP_

#include <common/drivers/Units.hpp>
#include <chrono>

namespace Lunabotics {
namespace Common {
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
}
}
}
#endif