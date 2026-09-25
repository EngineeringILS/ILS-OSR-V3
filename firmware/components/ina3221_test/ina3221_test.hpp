#include <SerialIO.hpp>
#include <ina3221.hpp>

using namespace Lunabotics::ESP32::Drivers;
void channel_data_to_str(const uint16_t &channel_num, const Lunabotics::Common::DataTypes::PowerChannelData &channel);
void ina3221_test_data(SerialIO &terminal, INA3221 &device);