
#ifndef LUNABOTICS_SENSORINTERFACE_HPP_
#define LUNABOTICS_SENSORINTERFACE_HPP_

#include <cstdint>

namespace Lunabotics {
namespace Common {
namespace Sensors {

/**
 * @brief Abstract interface for all hardware sensors in the project.
 * TODO: Improve documentation for the common sensor interface.
 */
class SensorInterface{
public:
    /**
    * @brief Defines the current operational state of the sensor.
     */
    enum class SensorState : uint8_t {
        UNINITIALIZED, // Driver created, but init() not called
        INITIALIZED,   // init() called, but comms not confirmed
        CONNECTED,     // Comms established, ready to read
        DISCONNECTED,  // Communication lost
        FAILED         // A hardware or unrecoverable error
    };

    /**
     * @brief Defines the communication bus being used.
     */
    enum class InterfaceType : uint8_t {
        I2C,   // I2C on ESP32 or Jetson 
        SPI,   // SPI on ESP32 or Jetson
        UART,  // UART on ESP32 or Jetson
        CAN,   // CAN on Jetson
        USB,   // USB on Jetson
        OTHER, // Any other interface
    };

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
    explicit SensorInterface(InterfaceType interface, HostController host) : 
        m_interface(interface),
        m_host(host),
        m_state(SensorState::UNINITIALIZED) {}

    /**
     * @brief Virtual destructor.
     */
    virtual ~SensorInterface() = default;

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
    SensorState getstate() const { return m_state; }

    /**
     * @brief Gets the interface (I2C, SPI, etc...).
     */
    InterfaceType getinterface() { return m_interface;}

    /**
     * @brief Gets the host controller (OBC, ESP32).
     */
    HostController getHost() { return m_host; }


protected:
    /**
     * @brief The current state of the sensor.
     * Child classes MUST update this variable in their init() and read() functions.
     */
    SensorState m_state;

    /**
     * @brief The fixed interface type for this sensor.
     */
    const InterfaceType m_interface;

    /**
     * @brief the fixed host controller for this sensor driver.
     */
    const HostController m_host;
    
};
}
}
}
#endif 