#ifndef LUNABOTICS_WIFIINTERFACE_HPP_
#define LUNABOTICS_WIFIINTERFACE_HPP_

#include <cstdint>

namespace Lunabotics {
namespace Common {
namespace Interfaces {

class WiFiInterface {
public:
    /**
     * @brief Defines the current operational state of the Wi-Fi Interface
     */
    enum class ConnectionState : uint8_t {
        UNINITIALIZED, // Driver created, but init() not called
        INITIALIZED,   // init() called, but comms not confirmed
        CONNECTING,    // connect() has been called, in progress
        CONNECTED,     // Connected to wireless network
        DISCONNECTED,  // Communication lost
        FAILED         // A hardware or unrecoverable error
    };


    // Lifecycle Methods:

    /**
     * @brief Constructor to create a new WiFi interface.
     * 
     * Default State is set to UNINITIALIZED in the base class, and will need to be properly handled in derived classes.
     */
    WiFiInterface() : m_state(ConnectionState::UNINITIALIZED) {}

    /**
     * @brief Virtual destructor.
     */
    virtual ~WiFiInterface() = default;
    
    // Virtual Functions to implement in Child Sensor Drivers
    // These must be implemented in the Child class!

    /**
     * @brief Initializes the WiFi hardware.
     * Connects, runs self-tests, and sets configuration.
     * Should update m_state to CONNECTED or FAILED
     * 
     * @return True on successful read, false on failure
     */
    virtual bool init() = 0;
    
    /**
     * @brief Connects to a specific Wi-Fi network.
     * Should update m_state to CONNECTING, then 
     * CONNECTED on success or FAILED on auth failure.
     * 
     * @return True on successful connection, false on failure.
     */
    virtual bool connect(const char* ssid, const char* password) = 0;

    /**
     * @brief Disconnects from the current Wi-Fi network.
     * Should updatge m_state to DISCONNECTED
     */
    virtual void disconnect() = 0;


    // State and Diagnostic Methods:
    
    /**
     * @brief Gets the current state of the sensor.
     */
    ConnectionState getState() const { return m_state; }
    
    /**
     * @brief Gets the current netwrok signal strength.
     * @return The RSSI value in dBm (e.g., -50). Returns 0 if not connected.
     */
    virtual int8_t getRSSI() const = 0;

    /**
     * @brief Gets the IP address assigned to this device.
     * @return A C-string (e.g. "192.168.1.101"). Returns "0.0.0.0" if not connected.
     */
    virtual const char* getIPaddress() const = 0;

protected:
    /**
     * @brief the current state of the sensor.
     * Child classes MUST update this variable in their init() functions
     */
    ConnectionState m_state;
};

}
}
}

#endif