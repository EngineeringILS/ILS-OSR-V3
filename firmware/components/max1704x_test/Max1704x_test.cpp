#include "Max1704x_test.hpp"

void max1704x_test_data(SerialIO &terminal, Max1704x &device) {
    Lunabotics::Common::DataTypes::BatteryData data;
    if (!device.read()) {
        std::string errname = esp_err_to_name(device.getErr());
        std::string rxerrname = esp_err_to_name(device.getRXErr());
        terminal.serial_out("Max1704X [FAIL] Error Codes: " + errname + ", " + rxerrname +  "\n");
    } else {
        device.getData(data);
        double voltage = data.voltage.in(Units::volts);
        double percent = data.percent.in(Units::percent);
        std::string isCharging;
        std::string isLow;
        if (data.is_charging) {
            isCharging = "(Charging)";
        } else {
            isCharging = "(Discharging)";
        }
        if (data.is_low) {
            isLow = "(Low)";
        } else {
            isLow = "(Not Low)";
        }
        terminal.serial_out("Max1704X [OK] Voltage (V), Percent (%): " + std::to_string(voltage) + ", " + std::to_string(percent) +  "\n");
        terminal.serial_out("Max1704X [FLAGS]: " + isCharging + " " + isLow + "\n");
    }
}