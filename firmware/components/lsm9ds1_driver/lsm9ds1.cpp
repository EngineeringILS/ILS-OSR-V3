#include "lsm9ds1.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

bool LSM9DS1::fail(esp_err_t error, SensorState state) {
    err_ = error;
    _State = state;
    return false;
}

bool LSM9DS1::readByte(I2CDevice& device, uint8_t reg, uint8_t& value) {
    if (!device.readRegister(reg, value)) {
        return fail(device.getRXErr());
    }
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::writeByte(I2CDevice& device, uint8_t reg, uint8_t value) {
    if (!device.writeRegister(reg, value)) {
        return fail(device.getRXErr());
    }
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::waitReset(I2CDevice& device, uint8_t reg, uint8_t mask) {
    for (unsigned attempt = 0; attempt < 100; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(1) + 1);
        uint8_t value;
        if (!readByte(device, reg, value)) {
            return false;
        }
        if ((value & mask) == 0) {
            return true;
        }
    }
    return fail(ESP_ERR_TIMEOUT);
}

bool LSM9DS1::init() {
    if ((getAddress() != 0x6A && getAddress() != 0x6B) ||
        (magnetometer_.getAddress() != 0x1C && magnetometer_.getAddress() != 0x1E)) {
        return fail(ESP_ERR_INVALID_ARG, SensorState::FAILED);
    }
    configured_ = false;
    _State = SensorState::INITIALIZED;
    if (!I2CDevice::init()) {
        return fail(I2CDevice::getErr(), SensorState::FAILED);
    }
    if (!magnetometer_.init()) {
        return fail(magnetometer_.getErr(), SensorState::FAILED);
    }

    uint8_t ag_id, mag_id;
    if (!readByte(*this, reg(AGRegister::WHO_AM_I), ag_id) ||
        !readByte(magnetometer_, reg(MagRegister::WHO_AM_I), mag_id)) {
        return fail(err_, SensorState::FAILED);
    }
    if (ag_id != EXPECTED_AG_ID || mag_id != EXPECTED_MAG_ID) {
        return fail(ESP_ERR_NOT_FOUND, SensorState::FAILED);
    }

    // Restore known register defaults before applying scale and acquisition settings.
    if (!writeByte(*this, reg(AGRegister::CTRL_REG8), 0x05) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG2), 0x04) ||
        !waitReset(*this, reg(AGRegister::CTRL_REG8), 0x01) ||
        !waitReset(magnetometer_, reg(MagRegister::CTRL_REG2), 0x04)) {
        return fail(err_, SensorState::FAILED);
    }

    const uint8_t accel = 0x60;     // +/-2 g, 119 Hz.
    const uint8_t gyro = 0x60;      // +/-245 dps, 119 Hz.
    const uint8_t mag_rate = 0xF4;  // 20 Hz, temperature compensation, ultra-high performance.
    const uint8_t mag_range = 0;    // +/-4 gauss.

    // BDU prevents torn words. AG increments automatically; magnetic bursts use bit 7.
    // Enable all axes and ultra-high magnetic performance with temperature compensation.
    if (!writeByte(*this, reg(AGRegister::CTRL_REG8), 0x44) ||
        !writeByte(*this, reg(AGRegister::CTRL_REG4), 0x38) ||
        !writeByte(*this, reg(AGRegister::CTRL_REG5_XL), 0x38) ||
        !writeByte(*this, reg(AGRegister::CTRL_REG6_XL), accel) ||
        !writeByte(*this, reg(AGRegister::CTRL_REG1_G), gyro) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG1), mag_rate) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG2), mag_range) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG4), 0x0C) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG5), 0x40) ||
        !writeByte(magnetometer_, reg(MagRegister::CTRL_REG3), 0x00)) {
        return fail(err_, SensorState::FAILED);
    }
    configured_ = true;
    imu_data_ = {};
    _State = SensorState::CONNECTED;
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::readWords(I2CDevice& device, uint8_t reg, int16_t* values, size_t count) {
    uint8_t bytes[6];
    if (!device.readRegisters(reg, bytes, count * 2)) {
        return fail(device.getRXErr());
    }
    for (size_t i = 0; i < count; ++i) {
        const uint16_t raw = uint16_t(bytes[2 * i]) | (uint16_t(bytes[2 * i + 1]) << 8);
        // Explicit sign extension avoids implementation-defined unsigned-to-signed conversion.
        values[i] = static_cast<int16_t>(raw < 0x8000 ? int32_t(raw) : int32_t(raw) - 65536);
    }
    return true;
}

bool LSM9DS1::read() {
    if (!configured_ || (_State != SensorState::CONNECTED && _State != SensorState::ERROR)) {
        err_ = ESP_ERR_INVALID_STATE;
        return false;
    }
    uint8_t accel_ctrl, gyro_ctrl, ag_format, mag_scale, mag_mode, mag_format, mag_bdu;
    uint8_t gyro_axes, accel_axes, ag_mode;
    if (!readByte(*this, reg(AGRegister::CTRL_REG6_XL), accel_ctrl) ||
        !readByte(*this, reg(AGRegister::CTRL_REG1_G), gyro_ctrl) ||
        !readByte(*this, reg(AGRegister::CTRL_REG8), ag_format) ||
        !readByte(magnetometer_, reg(MagRegister::CTRL_REG2), mag_scale) ||
        !readByte(magnetometer_, reg(MagRegister::CTRL_REG3), mag_mode) ||
        !readByte(magnetometer_, reg(MagRegister::CTRL_REG4), mag_format) ||
        !readByte(magnetometer_, reg(MagRegister::CTRL_REG5), mag_bdu) ||
        !readByte(*this, reg(AGRegister::CTRL_REG4), gyro_axes) ||
        !readByte(*this, reg(AGRegister::CTRL_REG5_XL), accel_axes) ||
        !readByte(*this, reg(AGRegister::CTRL_REG9), ag_mode)) {
        return false;
    }

    // Reject incompatible hardware settings before converting a sample.
    const uint8_t gyro_range = (gyro_ctrl >> 3) & 3;
    if ((ag_format & 0xC7) != 0x44 || (mag_format & 2) || (mag_bdu & 0xC0) != 0x40 ||
        (mag_scale & 0x0C) || (mag_mode & 3) != 0 || gyro_range == 2 ||
        (gyro_axes & 0x38) != 0x38 || (accel_axes & 0x38) != 0x38 || (ag_mode & 0x42) ||
        (accel_ctrl >> 5) == 0 || (accel_ctrl >> 5) == 7 ||
        (gyro_ctrl >> 5) == 0 || (gyro_ctrl >> 5) == 7) {
        return fail(ESP_ERR_INVALID_STATE);
    }

    DataTypes::LSM9DS1Data sample{};
    if (!readByte(*this, reg(AGRegister::STATUS_REG), sample.ag_status) ||
        !readByte(magnetometer_, reg(MagRegister::STATUS_REG), sample.mag_status)) {
        return false;
    }
    if ((sample.ag_status & 3) != 3 || (sample.mag_status & 8) == 0) {
        _State = SensorState::CONNECTED;
        err_ = ESP_ERR_NOT_FINISHED;
        return false;
    }

    int16_t accel[3], gyro[3], mag[3], temperature;
    if (!readWords(*this, reg(AGRegister::OUT_X_L_XL), accel, 3) ||
        !readWords(*this, reg(AGRegister::OUT_X_L_G), gyro, 3) ||
        !readWords(magnetometer_, reg(MagRegister::OUT_X_L) | 0x80, mag, 3) ||
        !readWords(*this, reg(AGRegister::OUT_TEMP_L), &temperature, 1)) {
        return false;
    }

    // Datasheet sensitivities: mg/LSB, mdps/LSB, and mgauss/LSB, in register range order.
    constexpr double accel_mg[] = {0.061, 0.732, 0.122, 0.244};
    constexpr double gyro_mdps[] = {8.75, 17.5, 0, 70.0};
    constexpr double mag_mgauss[] = {0.14, 0.29, 0.43, 0.58};
    const double a = accel_mg[(accel_ctrl >> 3) & 3] * 0.001 * 9.80665;
    const double g = gyro_mdps[gyro_range] * 0.001 * (3.14159265358979323846 / 180.0);
    const double m = mag_mgauss[(mag_scale >> 5) & 3] * 1e-7;
    sample.acceleration.a_x = Units::make_quantity<Units::Acceleration>(accel[0] * a);
    sample.acceleration.a_y = Units::make_quantity<Units::Acceleration>(accel[1] * a);
    sample.acceleration.a_z = Units::make_quantity<Units::Acceleration>(accel[2] * a);
    sample.angular_velocity.x = Units::make_quantity<Units::AngularVelocity>(gyro[0] * g);
    sample.angular_velocity.y = Units::make_quantity<Units::AngularVelocity>(gyro[1] * g);
    sample.angular_velocity.z = Units::make_quantity<Units::AngularVelocity>(gyro[2] * g);
    sample.magnetic_field.x = Units::tesla(mag[0] * m);
    sample.magnetic_field.y = Units::tesla(mag[1] * m);
    sample.magnetic_field.z = Units::tesla(mag[2] * m);
    sample.temperature = Units::kelvins(298.15 + temperature / 16.0);
    sample.timestamp = DataTypes::LocalClock::now();
    sample.acceleration.timestamp = sample.timestamp;
    imu_data_ = sample;
    err_ = ESP_OK;
    _State = SensorState::CONNECTED;
    return true;
}

void LSM9DS1::getData(DataTypes::LSM9DS1Data& data) const {
    data = imu_data_;
}

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics
