#ifndef LUNABOTICS_PERIPHERALINTERFACE_HPP_
#define LUNABOTICS_PERIPHERALINTERFACE_HPP_

#include <cstdint>
#include <common/protocols/InterfaceProtocols.hpp>

namespace Lunabotics {
namespace Common {
namespace Sensors {

/**
 * @brief Abstract interface for hardware peripherals in the project.
 */
class PeripheralInterface{
public:
    /**
    * @brief Defines the current operational state of the peripheral.
     */
    enum class PeripheralState : uint8_t {
        UNINITIALIZED, // Driver created, but init() not called
        INITIALIZED,   // init() called, but comms not confirmed
        CONNECTED,     // Comms established, ready to read
        DISCONNECTED,  // Communication lost
        ERROR,         // Some recoverable hardware error
        FAILED         // A hardware or unrecoverable error
    };

    /**
     * @brief Defines which processor is directly handling the peripheral driver.
     * @note This could potentially provide future functionality where configurations leverage the ESP32 for sensor passthrough/publishing.
     */
    enum class HostController : uint8_t {
        OBC, 
        ESP32
    };

    /** 
     * @brief Constructor to set the fixed properties of the peripheral.
     * 
     * @param interface the bus type (I2C, SPI, etc...)
     * @param host The host processor this driver is running on (OBC, ESP32)
     */
    explicit PeripheralInterface(Protocols::InterfaceType interface, HostController host) : 
        _State(PeripheralState::UNINITIALIZED),
        _Interface(interface),
        _Host(host)
    {}

    /**
     * @brief Virtual destructor.
     */
    virtual ~PeripheralInterface() = default;

    // Virtual Functions to implement in Child Peripheral Drivers:

    /**
     * @brief Initializes the peripheral hardware.
     * Connects, runs self-tests, and sets configuration.
     * Should update _State to CONNECTED or FAILED.
     * 
     * @return True on successful initialization, false on failure.
     */
    virtual bool init() = 0;

    // Implemented Functions:
    // These are common to all peripherals and are provided by the base class.

    /**
     * @brief Gets the current state of the peripheral.
     */
    PeripheralState getState() const { return _State; }

    /**
     * @brief Gets the interface (I2C, SPI, etc...).
     */
    Protocols::InterfaceType getinterface() { return _Interface; }

    /**
     * @brief Gets the host controller (OBC, ESP32).
     */
    HostController getHost() { return _Host; }


protected:
    /**
     * @brief The current state of the peripheral.
     * Child classes MUST update this variable during initialization and hardware operations.
     */
    PeripheralState _State;

    /**
     * @brief The fixed interface type for this peripheral.
     */
    const Protocols::InterfaceType _Interface;

    /**
     * @brief The fixed host controller for this peripheral driver.
     */
    const HostController _Host;
    
};
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics
#endif
