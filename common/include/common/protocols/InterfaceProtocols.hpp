#ifndef LUNABOTICS_BUSPROTOCOLS_HPP_
#define LUNABOTICS_BUSPROTOCOLS_HPP_

#include <cstdint>

namespace Lunabotics {
namespace Common {

/**
 * @brief Contains all the bus protocols used in the CDH subsystem (Primarily the C++ portion).
 * 
 * @note Currently includes support for the following:
 * - Interface Types (Enum Class)
 * - Interface Structs (I2C, SPI)
 * 
 */
namespace Protocols {

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
     * @brief Simple Serial RX/TX Transaction Flag.
     */
    enum class TransactionFlag {
        READ,
        WRITE
    };

    /**
     * @brief Generic I2C Configuration:
     */
    struct I2CConfig {
        uint8_t address;  // I2C Device Address (E.G. 0x36)
        uint16_t sda_pin;  // Data Pin  (SDA, platform specific)
        uint16_t scl_pin;  // Clock Pin (SCL, platform specific)
        uint16_t i2c_port; // I2C Port Number (platform specific)
        uint32_t frequency; // (Hz)
    };
    // Unit Library avoided, because fundamentally, units add a layer of complexity that is not needed for struct.
    
    /**
     * @brief Generic SPI Configuration:
    */
   struct SPIConfig {
        uint16_t mosi_pin;  // Main Out, subnode in (MOSI, platform specific)
        uint16_t miso_pin;  // Main in, subnode out (MISO, platform specific)
        uint16_t sclk_pin;  // Clock Pin (SCLK, platform specific)
        uint16_t cs_pin;    // Chip Select (CS, platform specific)
        uint32_t frequency; // (Hz)
   };
    // Unit Library avoided, because fundamentally, units add a layer of complexity that is not needed for struct.

} // namespace Protocols
} // namespace Common
} // namespace Lunabotics

#endif