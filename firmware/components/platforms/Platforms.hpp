#include <common/protocols/InterfaceProtocols.hpp>
#include <driver/gpio.h>
#include <cstddef>  


namespace Lunabotics {
namespace ESP32 {
namespace Boards {

using namespace Lunabotics::Common;

/** 
 * @brief Enum class specifying all board manufacturers used by Lunabotics.
 */
enum class Manufacturer : uint16_t {
    Adafruit,
    UnexpectedMaker
};

/**
 * @brief Abstract class to represent a standard ESP32 Board.
 * @todo Features need to be added on an as-needed basis, add all features to the base class and then implemenet in child classes.
 */
class Board {
public:
    /** 
     * @brief Constructor to set the fixed properties of the Board.
     * @param manufacturer The manufacturer of the board. 
     */
    explicit Board(Manufacturer manufacturer) : manufacturer_(manufacturer) {}

    /**
     * @brief Virtual Destructor.
     */
    virtual ~Board() = default;

    // Virtual Functions to implement in Child Board Classes, these must be implemented by the child class. 

    /**
     * @brief Virtual I2C Feature Check for the board, checks if a given port exists, and configures it if it can.
     * @param portNumber The physical port number to check for, generally, all boards should have port 0.
     * @param port The I2C Port to populate, remains unpopulated if the port number is invalid.
     * @return True if the port exists on the given platform.
     */
    virtual bool I2C(size_t portNumber, Protocols::I2CPort &port) { return false; }

    /**
     * @brief Virtual I2C Power Enable
     * @return True if the platform power supports power enable (typically GPIO), false if the platform does not have this feature.
     */
    virtual bool enableI2C() { return false; }

    /**
     * @brief Virtual I2C Power Disable
     * @return True if the platform power supports power disable (typically GPIO), false if the platform does not have this feature.
     */
    virtual bool disableI2C() { return false; }

protected:
    Manufacturer manufacturer_;
};

class FeatherS3TFT : public Board {
public:
    // The board manufacturer is Adafruit.
    FeatherS3TFT() : Board(Manufacturer::Adafruit) {}

    /**
     * @brief I2C Implementation for the FeatherS3TFT board. This board only supports one onboard I2C Port.
     */

    bool I2C(size_t portNumber, Protocols::I2CPort &port) override { 
        if (portNumber == 0) {
            port = I2C_Port_0;
            return true;
        } else {
            return false;
        }
    }

    bool enableI2C() override {
        gpio_set_direction(gpio_num_t(i2c_pwr_pin), GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_num_t(i2c_pwr_pin), 1);
        return true;
    }

    bool disableI2C() override {
        // Simple call to avoid confition that the GPIO_MODE_OUTPUT has not yet been set.
        enableI2C();
        gpio_set_level(gpio_num_t(i2c_pwr_pin), 0);
        return true;
    }

    Protocols::GPIOConfig led_pwr_pin {.gpio_pin = 13};
    
private:
    const int i2c_pwr_pin = 21;
    static constexpr Protocols::I2CPort I2C_Port_0{
        .sda_pin = 42,
        .scl_pin = 41,
        .i2c_port = 0,
        .frequency = 100000,
        .exists = true
    };
};

class FeatherS3RevTFT : public Board {
public:
    FeatherS3RevTFT() : Board(Manufacturer::Adafruit) {}

    /**
     * @brief I2C Implementation for the FeatherS3TFT board. This board only supports one onboard I2C Port.
     */
    bool I2C(size_t portNumber, Protocols::I2CPort &port) override {
        if (portNumber == 0) {
            port = I2C_Port_0;
            return true;
        } else {
            return false;
        }
    }
private:
    static constexpr Protocols::I2CPort I2C_Port_0{
        .sda_pin = 3,
        .scl_pin = 4,
        .i2c_port = 0,
        .frequency = 100000,
        .exists = true
    };
};

}
}
}