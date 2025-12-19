#include <i2c_driver.hpp>
#include <SerialIO.hpp>

#ifndef LUNABOTICS_I2C_TOOLS_HPP
#define LUNABOTICS_I2C_TOOLS_HPP

using namespace Lunabotics::ESP32::Drivers;
void i2c_status(SerialIO &terminal, I2CBus &bus);
void i2c_device_status(SerialIO &terminal, I2CDevice &device);
void i2c_device_read(SerialIO &terminal, I2CDevice &device, const uint8_t* addresses, const uint16_t num_addresses);
void i2c_scan(SerialIO &terminal, I2CBus &bus);
void i2c_dump(SerialIO &terminal, I2CBus &bus, uint8_t chip_address, int size = 1);

#endif