#ifndef LUNABOTICS_LSM9DS1_TEST_HPP
#define LUNABOTICS_LSM9DS1_TEST_HPP

#include <lsm9ds1.hpp>
#include <SerialIO.hpp>

/** @brief Prints one complete IMU sample or its readiness/error diagnostic. */
void lsm9ds1_test_data(SerialIO& terminal, Lunabotics::ESP32::Drivers::LSM9DS1& device);

#endif
