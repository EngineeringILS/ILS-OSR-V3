#ifndef LUNABOTICS_SERIALINTERFACE_HPP_
#define LUNABOTICS_SERIALINTERFACE_HPP_

#include <cstdint>

namespace Lunabotics {
namespace Common {
namespace Interfaces {

/**
 * @brief Abstract interface for all serial connections in the project.
 * TODO: Implement the serial interface base class.
 * TODO: Build a FakeSerial interface to demonstrate the wrapping of this class.
 */
class SerialInterface {
public:
    /**
     * @brief Defins the current operational state of the Serial interface
     */
    enum class ConnectionState : uint8_t {
        UNINITIALIZED, // Driver created, but init() not called
        INITIALIZED,   // init() called, but comms not confirmed
        CONNECTED,     // Connected over serial interface
        DISCONNECTED,  // Communication lost
        FAILED         // A hardware or unrecoverable error
    };

    // TODO: Implement Lifecycle Methods
    // Lifecycle Methods:

    // TODO: Implement State and Diagnostic Methods
    // State and Diagnostic Methods:


protected:
};
}
}
}
#endif