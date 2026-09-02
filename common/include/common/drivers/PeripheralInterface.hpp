#ifndef LUNABOTICS_PERIPHERALINTERFACE_HPP_
#define LUNABOTICS_PERIPHERALINTERFACE_HPP_

#include <cstdint>
#include <common/protocols/InterfaceProtocols.hpp>

namespace Lunabotics {
namespace Common {
namespace Sensors {

/**
 * @brief Abstract interface for all hardware sensors in the project.
 * TODO: Improve documentation for the common sensor interface.
 */
class PeripheralInterface{
public:
    /**
    * @brief Defines the current operational state of the sensor.
     */
    enum class SensorState : uint8_t {
        UNINITIALIZED, // Driver created, but init() not called
        INITIALIZED,   // init() called, but comms not confirmed
        CONNECTED,     // Comms established, ready to read
        DISCONNECTED,  // Communication lost
        ERROR,         // Some recoverable hardware error
        FAILED         // A hardware or unrecoverable error
    };

    // InterfaceType migrated to InterfaceProtocols.hpp

    /**
     * @brief Defines which processor is directly handling the sensor driver.
     * @note This could potentially provide future functionality where configurations leverage the ESP32 for sensor passthrough/publishing.
     */
    enum class HostController : uint8_t {
        OBC, 
        ESP32
    };

    /** 
     * @brief Constructor to set the fixed properties of the senesor.
     * 
     * @param interface the bus type (I2C, SPI, etc...)
     * @param host The host processor this driver is running on (OBC, ESP32)
     */
    explicit PeripheralInterface(Protocols::InterfaceType interface, HostController host) : 
        _State(SensorState::UNINITIALIZED),
        _Interface(interface),
        _Host(host)
    {}

    /**
     * @brief Virtual destructor.
     */
    virtual ~PeripheralInterface() = default;

    // Virtual Functions to implement in Child Sensor Drivers, these must be implemented in the Child class!

    /**
     * @brief Initializes the sensor hardware.
     * Connects, runs self-tests, and sets configuration.
     * Should update m_state to CONNECTED or FAILED
     * 
     * @return True on succesful read, false on failure
     */
    virtual bool init() = 0;

    // Implemented Functions:
    // These are common to all sensors and are provided by the base class.

    /**
     * @brief Gets the current state of the sensor.
     */
    SensorState getState() const { return _State; }

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
     * @brief The current state of the sensor.
     * Child classes MUST update this variable in their init() and read() functions.
     */
    SensorState _State;

    /**
     * @brief The fixed interface type for this sensor.
     */
    const Protocols::InterfaceType _Interface;

    /**
     * @brief the fixed host controller for this sensor driver.
     */
    const HostController _Host;
    
};
} // namespace Sensors
} // namespace Common
} // namespace Lunabotics
#endif 