#include "lsm9ds1_test.hpp"

using namespace Lunabotics::Common;
using namespace Lunabotics::ESP32::Drivers;

void lsm9ds1_test_data(SerialIO& terminal, LSM9DS1& device) {
    if (!device.read()) {
        if (device.getErr() == ESP_ERR_NOT_FINISHED) {
            terminal.serial_out("LSM9DS1 [WAIT] Fresh sample not ready.\n");
        } else {
            terminal.serial_out("LSM9DS1 [FAIL] " + std::string(esp_err_to_name(device.getErr())) + "\n");
        }
        return;
    }
    DataTypes::LSM9DS1Data data;
    device.getData(data);
    const auto acceleration = Units::meters / Units::squared(Units::seconds);
    const auto angular_rate = Units::radians / Units::seconds;
    terminal.serial_out("LSM9DS1 [OK] Native-axis measurements:\n");
    terminal.serial_out("Acceleration (m/s^2): " + std::to_string(data.acceleration.a_x.in(acceleration)) + ", " +
        std::to_string(data.acceleration.a_y.in(acceleration)) + ", " + std::to_string(data.acceleration.a_z.in(acceleration)) + "\n");
    terminal.serial_out("Angular rate (rad/s): " + std::to_string(data.angular_velocity.x.in(angular_rate)) + ", " +
        std::to_string(data.angular_velocity.y.in(angular_rate)) + ", " + std::to_string(data.angular_velocity.z.in(angular_rate)) + "\n");
    terminal.serial_out("Magnetic field (uT): " + std::to_string(data.magnetic_field.x.in(Units::tesla) * 1e6) + ", " +
        std::to_string(data.magnetic_field.y.in(Units::tesla) * 1e6) + ", " + std::to_string(data.magnetic_field.z.in(Units::tesla) * 1e6) + "\n");
    terminal.serial_out("Die temperature (deg C): " + std::to_string(data.temperature.in(Units::kelvins) - 273.15) + "\n");
    terminal.serial_out("Status AG/M: " + std::to_string(data.ag_status) + "/" + std::to_string(data.mag_status) + "\n");
}
