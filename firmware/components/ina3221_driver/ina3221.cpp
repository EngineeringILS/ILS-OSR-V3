#include "ina3221.hpp"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

bool INA3221::init() {
    if (!I2CDevice::init()) {
        _State = SensorState::FAILED;
        return false;
    }

    // Prevent invalid current calculations.
    if (channel_1_shunt_ohms_ <= Units::ZERO ||
        channel_2_shunt_ohms_ <= Units::ZERO ||
        channel_3_shunt_ohms_ <= Units::ZERO) {
        _State = SensorState::FAILED;
        return false;
    }

    uint16_t manufacturer_id;
    uint16_t die_id;

    if (!readRegister(
            reg(DataRegisters::MANUFACTURER_ID),
            manufacturer_id
        ) ||
        !readRegister(
            reg(DataRegisters::DIE_ID),
            die_id
        )) {
        _State = SensorState::FAILED;
        return false;
    }

    if (manufacturer_id != EXPECTED_MANUFACTURER_ID ||
        die_id != EXPECTED_DIE_ID) {
        _State = SensorState::FAILED;
        return false;
    }

    // Continuously measure shunt and bus voltage on all three channels.
    if (!writeRegister(
            reg(DataRegisters::CONFIGURATION),
            CONFIG_CONTINUOUS)) {
        _State = SensorState::FAILED;
        return false;
    }

    _State = SensorState::CONNECTED;
    return true;
}

bool INA3221::readChannel(
    DataRegisters shunt_register,
    DataRegisters bus_register,
    const Resistance& shunt_resistance,
    DataTypes::PowerChannelData& channel_data
) {
    int16_t raw_shunt;
    uint16_t raw_bus;

    if (!readRegister(reg(shunt_register), raw_shunt) ||
        !readRegister(reg(bus_register), raw_bus)) {
        return false;
    }

    constexpr uint16_t DATA_MASK     = 0xFFF8;
    constexpr double SHUNT_LSB_VOLTS = 40e-6;
    constexpr double BUS_LSB_VOLTS   = 8e-3;

    const int16_t shunt_data = static_cast<int16_t>(
        static_cast<uint16_t>(raw_shunt) & DATA_MASK
    );

    channel_data.shunt_voltage = Units::volts(
        (static_cast<double>(shunt_data) / 8.0) * SHUNT_LSB_VOLTS
    );

    channel_data.bus_voltage = Units::volts(
        static_cast<double>(raw_bus >> 3) * BUS_LSB_VOLTS
    );

    channel_data.current =
        (channel_data.shunt_voltage / shunt_resistance)
            .as(Units::amperes);

    channel_data.power =
        (channel_data.bus_voltage * channel_data.current)
            .as(Units::watts);

    return true;
}

bool INA3221::read() {
    if (_State != SensorState::CONNECTED &&
        _State != SensorState::ERROR) {
        return false;
    }

    DataTypes::INA3221Data new_data{};

    if (!readChannel(
            DataRegisters::SHUNT_VOLTAGE_CH1,
            DataRegisters::BUS_VOLTAGE_CH1,
            channel_1_shunt_ohms_,
            new_data.channel_1) ||
        !readChannel(
            DataRegisters::SHUNT_VOLTAGE_CH2,
            DataRegisters::BUS_VOLTAGE_CH2,
            channel_2_shunt_ohms_,
            new_data.channel_2) ||
        !readChannel(
            DataRegisters::SHUNT_VOLTAGE_CH3,
            DataRegisters::BUS_VOLTAGE_CH3,
            channel_3_shunt_ohms_,
            new_data.channel_3) ||
        !readRegister(
            reg(DataRegisters::MASK_ENABLE),
            new_data.flags)) {
        _State = SensorState::ERROR;
        return false;
    }

    new_data.timestamp = DataTypes::LocalClock::now();
    power_channel_data_ = new_data;

    _State = SensorState::CONNECTED;
    return true;
}

void INA3221::getData(DataTypes::INA3221Data& data) const {
    data = power_channel_data_;
}



} // namespace Sensors
} // namespace Common
} // namespace Lunabotics