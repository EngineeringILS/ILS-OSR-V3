#include <common/protocols/InterfaceProtocols.hpp>
#include "driver/i2c_master.h"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {
using namespace Lunabotics::Common;

class I2CBus {
public:
    /**
     * @brief I2C Bus constructor that attemps initialization on construction.
     * @param port The reference I2CPort struct to initialize.
     */
    explicit I2CBus(const Protocols::I2CPort &port) : port_(port) {
        init();
    }

    ~I2CBus() {
        if (initialized_) {
            i2c_del_master_bus(bus_handle_);
        }
    }

    /**
     * @brief Attemps to initialize the I2C Bus. 
     * Suceeds if err = ESP_OK and initialized_ = true. 
     * Fails gracefully specifying the error code in err, and setting initialized_ to false.
     */
    bool init() {
        if (!port_.exists) {
            initialized_ = false;
            // Port cannot be found:
            err = ESP_ERR_NOT_FOUND;
            
        } else if (!initialized_) {
            i2c_master_bus_config_t i2c_bus_config = {
                .i2c_port = port_.i2c_port,  // select a free I2C port automatically
                .sda_io_num = static_cast<gpio_num_t>(port_.sda_pin),
                .scl_io_num = static_cast<gpio_num_t>(port_.scl_pin),
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .glitch_ignore_cnt = 7,
                .flags = { .enable_internal_pullup = true }
            };

            err = i2c_new_master_bus(&i2c_bus_config, &bus_handle_);
            if (err == ESP_OK) {
                initialized_ = true;
            } else {
                initialized_ = false;
            }
        }
    return initialized_;
    }

    esp_err_t err;
    bool initialized_ = false;
private:
    const Protocols::I2CPort port_;
    i2c_master_bus_handle_t bus_handle_ = nullptr;
   
};

}
}
}
