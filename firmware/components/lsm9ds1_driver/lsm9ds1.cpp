#include "lsm9ds1.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

namespace {
using AG = LSM9DS1::AGRegister;
using MAG = LSM9DS1::MagRegister;

constexpr uint8_t reg(AG value) { return static_cast<uint8_t>(value); }
constexpr uint8_t reg(MAG value) { return static_cast<uint8_t>(value); }

bool validAG(uint8_t reg) {
    return (reg >= 0x04 && reg <= 0x0D) ||
           (reg >= 0x0F && reg <= 0x24) || (reg >= 0x26 && reg <= 0x37);
}

bool writableAG(uint8_t reg) {
    return (reg >= 0x04 && reg <= 0x0D) || (reg >= 0x10 && reg <= 0x13) ||
           (reg >= 0x1E && reg <= 0x24) || reg == 0x2E || (reg >= 0x30 && reg <= 0x37);
}

bool validMag(uint8_t reg) {
    return (reg >= 0x05 && reg <= 0x0A) || reg == 0x0F ||
           (reg >= 0x20 && reg <= 0x24) || (reg >= 0x27 && reg <= 0x2D) ||
           (reg >= 0x30 && reg <= 0x33);
}

bool writableMag(uint8_t reg) {
    return (reg >= 0x05 && reg <= 0x0A) || (reg >= 0x20 && reg <= 0x24) ||
           reg == 0x30 || reg == 0x32 || reg == 0x33;
}
}

LSM9DS1::LSM9DS1(I2CBus* bus, uint8_t ag_address, uint8_t mag_address) :
    SensorInterface(Protocols::InterfaceType::I2C, HostController::ESP32),
    I2CDevice(ag_address, bus),
    magnetometer_(mag_address, bus)
{}

bool LSM9DS1::fail(esp_err_t error, SensorState state) {
    err_ = error;
    _State = state;
    return false;
}

bool LSM9DS1::validConfig(const Config& c) {
    const auto gyro = static_cast<uint8_t>(c.gyro_range);
    return static_cast<uint8_t>(c.accel_range) <= 3 &&
           (gyro == 0 || gyro == 1 || gyro == 3) &&
           static_cast<uint8_t>(c.mag_range) <= 3 &&
           static_cast<uint8_t>(c.accel_rate) <= 6 &&
           static_cast<uint8_t>(c.gyro_rate) <= 6 &&
           static_cast<uint8_t>(c.mag_rate) <= 7;
}

bool LSM9DS1::readByte(I2CDevice& device, uint8_t reg, uint8_t& value) {
    if (!device.readRegister(reg, value)) return fail(device.getRXErr());
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::writeByte(I2CDevice& device, uint8_t reg, uint8_t value) {
    if (!device.writeRegister(reg, value)) return fail(device.getRXErr());
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::waitReset(I2CDevice& device, uint8_t reg, uint8_t mask) {
    for (unsigned attempt = 0; attempt < 100; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(1) + 1);
        uint8_t value;
        if (!readByte(device, reg, value)) return false;
        if ((value & mask) == 0) return true;
    }
    return fail(ESP_ERR_TIMEOUT);
}

bool LSM9DS1::init() { return init(Config{}); }

bool LSM9DS1::init(const Config& config) {
    if (!validConfig(config) ||
        (getAGAddress() != 0x6A && getAGAddress() != 0x6B) ||
        (getMagAddress() != 0x1C && getMagAddress() != 0x1E)) {
        err_ = ESP_ERR_INVALID_ARG;
        return false;
    }
    configured_ = false;
    has_data_ = false;
    _State = SensorState::INITIALIZED;
    if (!I2CDevice::init()) return fail(I2CDevice::getErr(), SensorState::FAILED);
    if (!magnetometer_.init()) return fail(magnetometer_.getErr(), SensorState::FAILED);

    uint8_t ag_id, mag_id;
    if (!readRegister(AG::WHO_AM_I, ag_id) || !readRegister(MAG::WHO_AM_I, mag_id)) {
        return fail(err_, SensorState::FAILED);
    }
    if (ag_id != 0x68 || mag_id != 0x3D) return fail(ESP_ERR_NOT_FOUND, SensorState::FAILED);

    // Restore known register defaults before applying scale and acquisition settings.
    if (!writeByte(*this, reg(AG::CTRL_REG8), 0x05) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG2), 0x04) ||
        !waitReset(*this, reg(AG::CTRL_REG8), 0x01) ||
        !waitReset(magnetometer_, reg(MAG::CTRL_REG2), 0x04)) {
        return fail(err_, SensorState::FAILED);
    }

    const uint8_t accel = (static_cast<uint8_t>(config.accel_rate) << 5) |
                          (static_cast<uint8_t>(config.accel_range) << 3);
    const uint8_t gyro = (static_cast<uint8_t>(config.gyro_rate) << 5) |
                         (static_cast<uint8_t>(config.gyro_range) << 3);
    const uint8_t mag_rate = 0xE0 | (static_cast<uint8_t>(config.mag_rate) << 2);
    const uint8_t mag_range = static_cast<uint8_t>(config.mag_range) << 5;

    // BDU prevents torn words. AG increments automatically; magnetic bursts use bit 7.
    // Enable all axes and ultra-high magnetic performance with temperature compensation.
    if (!writeByte(*this, reg(AG::CTRL_REG8), 0x44) ||
        !writeByte(*this, reg(AG::CTRL_REG4), 0x38) ||
        !writeByte(*this, reg(AG::CTRL_REG5_XL), 0x38) ||
        !writeByte(*this, reg(AG::CTRL_REG6_XL), accel) ||
        !writeByte(*this, reg(AG::CTRL_REG1_G), gyro) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG1), mag_rate) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG2), mag_range) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG4), 0x0C) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG5), 0x40) ||
        !writeByte(magnetometer_, reg(MAG::CTRL_REG3), 0x00)) {
        return fail(err_, SensorState::FAILED);
    }
    config_ = config;
    configured_ = true;
    data_ = {};
    _State = SensorState::CONNECTED;
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::configure(const Config& config) { return init(config); }
bool LSM9DS1::reset() { return init(config_); }

bool LSM9DS1::powerDown() {
    configured_ = false;
    has_data_ = false;
    // Attempt all three writes, even if one of the internal slaves is disconnected.
    esp_err_t first_error = ESP_OK;
    if (!writeByte(*this, reg(AG::CTRL_REG1_G), 0)) first_error = err_;
    if (!writeByte(*this, reg(AG::CTRL_REG6_XL), 0) && first_error == ESP_OK) first_error = err_;
    if (!writeByte(magnetometer_, reg(MAG::CTRL_REG3), 3) && first_error == ESP_OK) first_error = err_;
    if (first_error != ESP_OK) return fail(first_error);
    _State = SensorState::INITIALIZED;
    err_ = ESP_OK;
    return true;
}

bool LSM9DS1::deinit() {
    if (!I2CDevice::isInitialized() && !magnetometer_.isInitialized()) {
        configured_ = false;
        has_data_ = false;
        _State = SensorState::UNINITIALIZED;
        err_ = ESP_OK;
        return true;
    }
    const bool stopped = powerDown();
    magnetometer_.deinit();
    I2CDevice::deinit();
    _State = SensorState::UNINITIALIZED;
    return stopped;
}

bool LSM9DS1::readRegister(AGRegister reg, uint8_t& value) {
    const uint8_t address = static_cast<uint8_t>(reg);
    if (!validAG(address)) { err_ = ESP_ERR_INVALID_ARG; return false; }
    return readByte(*this, address, value);
}

bool LSM9DS1::readRegister(MagRegister reg, uint8_t& value) {
    const uint8_t address = static_cast<uint8_t>(reg);
    if (!validMag(address)) { err_ = ESP_ERR_INVALID_ARG; return false; }
    return readByte(magnetometer_, address, value);
}

bool LSM9DS1::writeRegister(AGRegister reg, uint8_t value) {
    const uint8_t address = static_cast<uint8_t>(reg);
    if (!writableAG(address) || (reg == AG::CTRL_REG9 && (value & 0x04))) {
        err_ = ESP_ERR_INVALID_ARG;
        return false;
    }
    has_data_ = false;
    if (reg == AG::CTRL_REG8 && (value & 0x81)) configured_ = false;
    if (!writeByte(*this, address, value)) return false;
    return true;
}

bool LSM9DS1::writeRegister(MagRegister reg, uint8_t value) {
    const uint8_t address = static_cast<uint8_t>(reg);
    if (!writableMag(address) || (reg == MAG::CTRL_REG3 && (value & 0x80))) {
        err_ = ESP_ERR_INVALID_ARG;
        return false;
    }
    has_data_ = false;
    if (reg == MAG::CTRL_REG2 && (value & 0x0C)) configured_ = false;
    return writeByte(magnetometer_, address, value);
}

bool LSM9DS1::dataReady(bool& ready) {
    if (!configured_) { err_ = ESP_ERR_INVALID_STATE; return false; }
    uint8_t ag, mag;
    if (!readRegister(AG::STATUS_REG, ag) || !readRegister(MAG::STATUS_REG, mag)) return false;
    ready = (ag & 0x03) == 0x03 && (mag & 0x08) != 0;
    return true;
}

bool LSM9DS1::readWords(I2CDevice& device, uint8_t reg, int16_t* values, size_t count) {
    uint8_t bytes[6];
    if (!device.readRegisters(reg, bytes, count * 2)) return fail(device.getRXErr());
    for (size_t i = 0; i < count; ++i) {
        const uint16_t raw = uint16_t(bytes[2 * i]) | (uint16_t(bytes[2 * i + 1]) << 8);
        // Explicit sign extension avoids implementation-defined unsigned-to-signed conversion.
        values[i] = static_cast<int16_t>(raw < 0x8000 ? int32_t(raw) : int32_t(raw) - 65536);
    }
    return true;
}

bool LSM9DS1::read() {
    if (!configured_) { err_ = ESP_ERR_INVALID_STATE; return false; }
    uint8_t accel_ctrl, gyro_ctrl, ag_format, mag_scale, mag_mode, mag_format, mag_bdu;
    uint8_t gyro_axes, accel_axes, ag_mode;
    if (!readRegister(AG::CTRL_REG6_XL, accel_ctrl) || !readRegister(AG::CTRL_REG1_G, gyro_ctrl) ||
        !readRegister(AG::CTRL_REG8, ag_format) || !readRegister(MAG::CTRL_REG2, mag_scale) ||
        !readRegister(MAG::CTRL_REG3, mag_mode) || !readRegister(MAG::CTRL_REG4, mag_format) ||
        !readRegister(MAG::CTRL_REG5, mag_bdu) || !readRegister(AG::CTRL_REG4, gyro_axes) ||
        !readRegister(AG::CTRL_REG5_XL, accel_axes) || !readRegister(AG::CTRL_REG9, ag_mode)) return false;

    // Raw register access must not silently change the assumptions used by conversion.
    const uint8_t gyro_range = (gyro_ctrl >> 3) & 3;
    if ((ag_format & 0xC7) != 0x44 || (mag_format & 2) || (mag_bdu & 0xC0) != 0x40 ||
        (mag_scale & 0x0C) || (mag_mode & 3) != 0 || gyro_range == 2 ||
        (gyro_axes & 0x38) != 0x38 || (accel_axes & 0x38) != 0x38 || (ag_mode & 0x42) ||
        (accel_ctrl >> 5) == 0 || (accel_ctrl >> 5) == 7 ||
        (gyro_ctrl >> 5) == 0 || (gyro_ctrl >> 5) == 7) {
        return fail(ESP_ERR_INVALID_STATE);
    }

    DataTypes::LSM9DS1Data sample{};
    if (!readRegister(AG::STATUS_REG, sample.ag_status) ||
        !readRegister(MAG::STATUS_REG, sample.mag_status)) return false;
    if ((sample.ag_status & 3) != 3 || (sample.mag_status & 8) == 0) {
        _State = SensorState::CONNECTED;
        err_ = ESP_ERR_NOT_FINISHED;
        return false;
    }

    int16_t accel[3], gyro[3], mag[3], temperature;
    if (!readWords(*this, reg(AG::OUT_X_L_XL), accel, 3) ||
        !readWords(*this, reg(AG::OUT_X_L_G), gyro, 3) ||
        !readWords(magnetometer_, reg(MAG::OUT_X_L) | 0x80, mag, 3) ||
        !readWords(*this, reg(AG::OUT_TEMP_L), &temperature, 1)) return false;

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
    data_ = sample;
    has_data_ = true;
    err_ = ESP_OK;
    _State = SensorState::CONNECTED;
    return true;
}

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics
