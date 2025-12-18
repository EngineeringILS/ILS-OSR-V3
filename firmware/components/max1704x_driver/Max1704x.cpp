#include "Max1704x.hpp"

// namespace Lunabotics {
// namespace ESP32 {
// namespace Drivers {


// esp_err_t Max1704x::max1704x_init_desc_mod() {
//     max1704x_.i2c_dev.port = static_cast<i2c_port_t>(I2Cbus_->getI2CPort().i2c_port);
//     max1704x_.i2c_dev.addr = address_;
//     max1704x_.i2c_dev.cfg.sda_io_num = static_cast<gpio_num_t>(I2Cbus_->getI2CPort().sda_pin);
//     max1704x_.i2c_dev.cfg.scl_io_num = static_cast<gpio_num_t>(I2Cbus_->getI2CPort().scl_pin);
//     max1704x_.i2c_dev.cfg.master.clk_speed = I2Cbus_->getI2CPort().frequency;
//     return i2c_dev_create_mutex(&max1704x_.i2c_dev);
// }

// bool Max1704x::init() {
//     if
//     const Protocols::I2CPort i2c_bus_port = I2Cbus_->getI2CPort();
//     gpio_num_t sda_pin = static_cast<gpio_num_t>(i2c_bus_port.sda_pin);
//     gpio_num_t scl_pin = static_cast<gpio_num_t>(i2c_bus_port.scl_pin);
//     esp_err_t err = max1704x_init_desc(
//         // device descriptor,
//         // i2c port number,
//         // sda GPIO
//         // cl GPIO
//     );

//     if (initStatus != ESP_OK) {
//         // TODO: Need to qualify the error to properly determine what kind of error and respective state change.
//         // Read ESP32 Docs for ESP_ERR_T status codes.
//         _State = SensorState::ERROR;
//         return false;
//     } else {
//         _State = SensorState::CONNECTED;
//     }
// }


// } // namespace Sensors
// } // namespace Common
// } // namespace Lunabotics