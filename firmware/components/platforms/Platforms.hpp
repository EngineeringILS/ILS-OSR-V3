#include <common/protocols/InterfaceProtocols.hpp>
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

private:
    static constexpr Protocols::I2CPort I2C_Port_0{
        .sda_pin = 42,
        .scl_pin = 41,
        .i2c_port = 0,
        .frequency = 100000,
        .exists = true
    };
};

}
}
}