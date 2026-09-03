#include "ina3221_test.hpp"


void channel_data_to_str(
    const uint16_t &channel_num, 
    const Lunabotics::Common::DataTypes::PowerChannelData &channel, 
    std::string &output) 
    {  
    double bus_voltage = channel.bus_voltage.in(Units::volts);
    double shunt_voltage = channel.shunt_voltage.in(Units::volts);
    double current = channel.current.in(Units::amperes);
    double power = channel.power.in(Units::watts);
    output.clear();
    output += "[Channel " + std::to_string(channel_num) + "]\n";
    output += "Bus Voltage (V): " + std::to_string(bus_voltage) + "\n";
    output += "Shunt Voltage (V): " + std::to_string(shunt_voltage) + "\n";
    output += "Current (A): " + std::to_string(current) + "\n ";
    output += "Power (W):  " + std::to_string(power) + "\n";
    return;
}

void ina3221_test_data(SerialIO &terminal, INA3221 &device) {
    Lunabotics::Common::DataTypes::INA3221Data data;
    if (!device.read()) {
        std::string errname = esp_err_to_name(device.getErr());
        std::string rxerrname = esp_err_to_name(device.getRXErr());
        terminal.serial_out("INA3221 [FAIL] Error Codes: " + errname + ", " + rxerrname +  "\n");
    } else {
        device.getData(data);

        std::string channel_output;
        channel_output.reserve(128);
        terminal.serial_out("INA3221 [OK] \n");
        channel_data_to_str(1, data.channel_1, channel_output);
        terminal.serial_out(channel_output);
        channel_data_to_str(2, data.channel_2, channel_output);
        terminal.serial_out(channel_output);
        channel_data_to_str(3, data.channel_2, channel_output);
        terminal.serial_out(channel_output);

    }
}