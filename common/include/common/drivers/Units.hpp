#ifndef LUNABOTICS_UNITS_HPP_
#define LUNABOTICS_UNITS_HPP_

#include <common/utils/au/au_all_units_noio.hh>

namespace Lunabotics {
namespace Common {
namespace Units {
    // Proxy the AU namespace into Lunabotics::Common:Units
    using namespace ::au;

    // Linear Position-Base Unit Types:
    using Acceleration = decltype(meters / squared(second))::Unit;
    using Velocity     = decltype(meters / second)::Unit;
    using Position     = decltype(meters):: Unit;

    // Static Base Factors:
    /**
     * @brief Defines several conversion factors for Gravity, these are especially useful as the units, Units::Acceleration, are inherited through multiplication.
     * 
     * Currently only has SeaLevel = 9.81f (m/s^2)
     */
    struct Gravity {
        static constexpr auto SeaLevel = make_quantity<Acceleration>(9.81);
    };
    
}
}
}

#endif