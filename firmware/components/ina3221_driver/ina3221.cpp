#include "ina3221.hpp"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

bool Max1704x::init() {
    // I2CDevice already checks the bus in its constructor and init, no need to check again here.
    if (!I2CDevice::init()) {
        // Failure to initialize --> likely unrecoverable hardware error since this chip is on platform, check I2C Bus & config.
        // The exact Error would need to be checked by invoking the I2CDevice getErr() method.
        _State = SensorState::FAILED;
        return false;
    }
    
    uint16_t version;
    // Failure to read from the register, likely a deeper issue than initialization.
    // Refer to the above getErr() method.
    if (!readRegister(reg(DataRegisters::VERSION), version)) {
        _State = SensorState::FAILED;
        return false;
    }

    // If initialization succeeded, and reading suceeded, then the sensor must be connected:
    _State = SensorState::CONNECTED;
    return true;
}

bool Max1704x::read() {
    // Connected State --> Good Read.
    // Error State --> Last read may have failed, but potentially not catastrophic, if the sensor is truly disconnected, the readRegister should just return false.
    if (!(_State == SensorState::CONNECTED)) {
        return false;
    }

    uint16_t rawVoltageData, rawSocData;
    int16_t rawChargeRate;
    // Read all regs:
    if (!readRegister(reg(DataRegisters::VCELL), rawVoltageData)) {
        // Failure to read --> ERROR.
        _State = SensorState::ERROR;
        return false;
    } 

    if (!readRegister(reg(DataRegisters::SOC), rawSocData)) {
        // Failure to read --> ERROR.
        _State = SensorState::ERROR;
        return false;
    }

    if (!readRegister(reg(DataRegisters::CRATE), rawChargeRate)) {
        // Failure to read --> ERROR.
        _State = SensorState::ERROR;
        return false;
    }

    // Run all conversions:
    double voltage = static_cast<double>((rawVoltageData >> 4) * 0.00125f);
    double percentage = static_cast<double>(rawSocData) / 256.0;
    double chargeRate = static_cast<double>(rawChargeRate) * 0.208f;

    // Populate battery_data_:
    battery_data_.voltage = Units::volts(voltage);
    battery_data_.percent = Units::percent(percentage);
    
    if (percentage < 40) {
        battery_data_.is_low = true;
    } else {
        battery_data_.is_low = false;
    }

    if (chargeRate > 0) {
        battery_data_.is_charging = true;
    } else {
        battery_data_.is_charging = false;
    }
    return true;
}

void Max1704x::getData(DataTypes::BatteryData& data) const {
    data = battery_data_;
}



} // namespace Sensors
} // namespace Common
} // namespace Lunabotics