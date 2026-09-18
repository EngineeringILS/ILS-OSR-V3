#ifndef LUNABOTICS_LSM9DS1_DRIVER_HPP
#define LUNABOTICS_LSM9DS1_DRIVER_HPP

#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <i2c_driver.hpp>

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;

/**
 * @brief I2C LSM9DS1 accelerometer, gyroscope, magnetometer, and temperature driver.
 * @note The accelerometer/gyroscope and magnetometer use separate I2C addresses.
 * Call from one task, or serialize access externally. No fusion or axis remapping is applied.
 */
class LSM9DS1 : public SensorInterface, protected I2CDevice {
public:
    /** @brief Complete accelerometer/gyroscope register map (ST datasheet, Table 21). */
    enum class AGRegister : uint8_t {
        ACT_THS = 0x04,          // RW: Inactivity threshold and gyro sleep selection.
        ACT_DUR = 0x05,          // RW: Inactivity duration.
        INT_GEN_CFG_XL = 0x06,   // RW: Accelerometer interrupt conditions.
        INT_GEN_THS_X_XL = 0x07, // RW: X acceleration threshold.
        INT_GEN_THS_Y_XL = 0x08, // RW: Y acceleration threshold.
        INT_GEN_THS_Z_XL = 0x09, // RW: Z acceleration threshold.
        INT_GEN_DUR_XL = 0x0A,   // RW: Acceleration interrupt duration.
        REFERENCE_G = 0x0B,      // RW: Gyroscope high-pass reference.
        INT1_CTRL = 0x0C,        // RW: Interrupt 1 routing.
        INT2_CTRL = 0x0D,        // RW: Interrupt 2 routing.
        WHO_AM_I = 0x0F,         // RO: Expected identification 0x68.
        CTRL_REG1_G = 0x10,      // RW: Gyroscope rate, range, and bandwidth.
        CTRL_REG2_G = 0x11,      // RW: Gyroscope output and interrupt filters.
        CTRL_REG3_G = 0x12,      // RW: Gyroscope power and high-pass filter.
        ORIENT_CFG_G = 0x13,     // RW: Gyroscope axis orientation.
        INT_GEN_SRC_G = 0x14,    // RO: Gyroscope interrupt source; read clears latch.
        OUT_TEMP_L = 0x15,      // RO: Temperature low byte.
        OUT_TEMP_H = 0x16,      // RO: Temperature high byte.
        STATUS_REG = 0x17,      // RO: Temperature, gyro, and acceleration readiness.
        OUT_X_L_G = 0x18,       // RO: X angular rate low byte.
        OUT_X_H_G = 0x19,       // RO: X angular rate high byte.
        OUT_Y_L_G = 0x1A,       // RO: Y angular rate low byte.
        OUT_Y_H_G = 0x1B,       // RO: Y angular rate high byte.
        OUT_Z_L_G = 0x1C,       // RO: Z angular rate low byte.
        OUT_Z_H_G = 0x1D,       // RO: Z angular rate high byte.
        CTRL_REG4 = 0x1E,        // RW: Gyroscope axis enables and interrupt latch.
        CTRL_REG5_XL = 0x1F,     // RW: Acceleration decimation and axis enables.
        CTRL_REG6_XL = 0x20,     // RW: Acceleration rate, range, and bandwidth.
        CTRL_REG7_XL = 0x21,     // RW: Acceleration filters.
        CTRL_REG8 = 0x22,        // RW: Reset, byte order, BDU, and increment mode.
        CTRL_REG9 = 0x23,        // RW: Sleep, FIFO, and I2C control.
        CTRL_REG10 = 0x24,       // RW: Acceleration and gyroscope self-test excitation.
        INT_GEN_SRC_XL = 0x26,   // RO: Acceleration interrupt source; read clears latch.
        STATUS_REG_ALT = 0x27,   // RO: Alternate status register.
        OUT_X_L_XL = 0x28,      // RO: X acceleration low byte.
        OUT_X_H_XL = 0x29,      // RO: X acceleration high byte.
        OUT_Y_L_XL = 0x2A,      // RO: Y acceleration low byte.
        OUT_Y_H_XL = 0x2B,      // RO: Y acceleration high byte.
        OUT_Z_L_XL = 0x2C,      // RO: Z acceleration low byte.
        OUT_Z_H_XL = 0x2D,      // RO: Z acceleration high byte.
        FIFO_CTRL = 0x2E,        // RW: FIFO mode and threshold.
        FIFO_SRC = 0x2F,         // RO: FIFO count, threshold, and overrun.
        INT_GEN_CFG_G = 0x30,    // RW: Gyroscope interrupt conditions.
        INT_GEN_THS_XH_G = 0x31, // RW: X threshold high byte and counter mode.
        INT_GEN_THS_XL_G = 0x32, // RW: X threshold low byte.
        INT_GEN_THS_YH_G = 0x33, // RW: Y threshold high byte.
        INT_GEN_THS_YL_G = 0x34, // RW: Y threshold low byte.
        INT_GEN_THS_ZH_G = 0x35, // RW: Z threshold high byte.
        INT_GEN_THS_ZL_G = 0x36, // RW: Z threshold low byte.
        INT_GEN_DUR_G = 0x37     // RW: Gyroscope interrupt duration.
    };

    /** @brief Complete magnetometer register map (ST datasheet, Table 22). */
    enum class MagRegister : uint8_t {
        OFFSET_X_L = 0x05, // RW: X hard-iron offset low byte.
        OFFSET_X_H = 0x06, // RW: X hard-iron offset high byte.
        OFFSET_Y_L = 0x07, // RW: Y hard-iron offset low byte.
        OFFSET_Y_H = 0x08, // RW: Y hard-iron offset high byte.
        OFFSET_Z_L = 0x09, // RW: Z hard-iron offset low byte.
        OFFSET_Z_H = 0x0A, // RW: Z hard-iron offset high byte.
        WHO_AM_I = 0x0F,   // RO: Expected identification 0x3D.
        CTRL_REG1 = 0x20,  // RW: Compensation, XY performance, rate, and self-test.
        CTRL_REG2 = 0x21,  // RW: Range, reboot, and reset.
        CTRL_REG3 = 0x22,  // RW: Power mode and interface control.
        CTRL_REG4 = 0x23,  // RW: Z performance and byte order.
        CTRL_REG5 = 0x24,  // RW: Fast read and block data update.
        STATUS_REG = 0x27, // RO: Axis readiness and overrun.
        OUT_X_L = 0x28,    // RO: X magnetic field low byte.
        OUT_X_H = 0x29,    // RO: X magnetic field high byte.
        OUT_Y_L = 0x2A,    // RO: Y magnetic field low byte.
        OUT_Y_H = 0x2B,    // RO: Y magnetic field high byte.
        OUT_Z_L = 0x2C,    // RO: Z magnetic field low byte.
        OUT_Z_H = 0x2D,    // RO: Z magnetic field high byte.
        INT_CFG = 0x30,    // RW: Magnetic interrupt configuration.
        INT_SRC = 0x31,    // RO: Magnetic interrupt source; read clears latch.
        INT_THS_L = 0x32,  // RW: Magnetic interrupt threshold low byte.
        INT_THS_H = 0x33   // RW: Magnetic interrupt threshold high seven bits.
    };

    /** @brief Acceleration full-scale values encoded for CTRL_REG6_XL. */
    enum class AccelRange : uint8_t { G2 = 0, G16 = 1, G4 = 2, G8 = 3 };
    /** @brief Angular rate full-scale values; encoding 2 is reserved. */
    enum class GyroRange : uint8_t { DPS245 = 0, DPS500 = 1, DPS2000 = 3 };
    /** @brief Magnetic full-scale values encoded for CTRL_REG2_M. */
    enum class MagRange : uint8_t { GAUSS4 = 0, GAUSS8 = 1, GAUSS12 = 2, GAUSS16 = 3 };
    /** @brief Accelerometer-only rates; enabled gyro overrides the effective AG rate. */
    enum class AccelRate : uint8_t { OFF = 0, HZ10, HZ50, HZ119, HZ238, HZ476, HZ952 };
    /** @brief Gyroscope rates; OFF disables complete nine-axis acquisition. */
    enum class GyroRate : uint8_t { OFF = 0, HZ14_9, HZ59_5, HZ119, HZ238, HZ476, HZ952 };
    /** @brief Normal magnetometer rates; fast-rate selection is available through registers. */
    enum class MagRate : uint8_t { HZ0_625 = 0, HZ1_25, HZ2_5, HZ5, HZ10, HZ20, HZ40, HZ80 };

    /** @brief Normal acquisition configuration; both AG rates follow the gyro when it is enabled. */
    struct Config {
        AccelRange accel_range = AccelRange::G2;
        GyroRange gyro_range = GyroRange::DPS245;
        MagRange mag_range = MagRange::GAUSS4;
        AccelRate accel_rate = AccelRate::HZ119;
        GyroRate gyro_rate = GyroRate::HZ119;
        MagRate mag_rate = MagRate::HZ20;
    };

    // Constructor and Destructor:
    /**
     * @brief Registers the two I2C slaves; init() performs identification and setup.
     * @param bus Initialized bus, which must outlive this driver.
     * @param ag_address Acceleration/gyro address: 0x6B by default, or 0x6A.
     * @param mag_address Magnetometer address: 0x1E by default, or 0x1C.
     */
    explicit LSM9DS1(I2CBus* bus, uint8_t ag_address = 0x6B, uint8_t mag_address = 0x1E);
    /** @brief Releases both I2C registrations; call powerDown() before destruction to stop sampling. */
    ~LSM9DS1() override = default;

    // Implementation of the pure virtual functions:
    /** @brief Resets both blocks and applies default acquisition settings. */
    bool init() override;
    /**
     * @brief Resets both blocks and applies the supplied acquisition settings.
     * @param config Ranges and rates; invalid enum values leave hardware unchanged.
     * @return True when both blocks are configured, false on identification, reset, or bus failure.
     */
    bool init(const Config& config);

    // Sensor Specific Public Functions:
    /**
     * @brief Reads a complete nine-axis sample and latest temperature.
     * @return True on a complete sample. False preserves cached data; ESP_ERR_NOT_FINISHED
     * means a fresh sample is not ready, other errors require diagnostics or reinitialization.
     */
    bool read();
    /** @brief Copies the last complete sample without bus traffic. */
    void getData(DataTypes::LSM9DS1Data& data) const { data = data_; }
    /** @brief Reports whether read() has published a sample since initialization/configuration. */
    bool hasData() const { return has_data_; }
    /** @brief Returns the most recent driver/transaction error, including either I2C address. */
    esp_err_t getErr() const { return err_; }
    /** @brief Returns the acceleration/gyroscope slave address. */
    uint8_t getAGAddress() const { return I2CDevice::getAddress(); }
    /** @brief Returns the magnetometer slave address. */
    uint8_t getMagAddress() const { return magnetometer_.getAddress(); }
    /** @brief Resets and reapplies the last successful high-level configuration; raw settings are cleared. */
    bool reset();
    /** @brief Powers down all sensing blocks; init() or reset() resumes acquisition. */
    bool powerDown();
    /** @brief Powers down and releases both registrations; repeat calls succeed without bus traffic. */
    bool deinit();
    /** @brief Applies ranges and rates by resetting both blocks; invalid values leave hardware unchanged. */
    bool configure(const Config& config);
    /** @brief Copies the last high-level configuration; raw writes are not reflected here. */
    Config getConfig() const { return config_; }
    /** @brief Reads status without consuming measurement registers; outputs change only on success. */
    bool dataReady(bool& ready);

    /** @brief Reads a documented AG register; reserved addresses are rejected. */
    bool readRegister(AGRegister reg, uint8_t& value);
    /** @brief Reads a documented magnetic register; reserved addresses are rejected. */
    bool readRegister(MagRegister reg, uint8_t& value);
    /**
     * @brief Writes a documented writable AG register for filters, FIFO, interrupts, or self-test.
     * @note Caller supplies datasheet-valid bit fields. read() checks its required format and
     * current range before conversion. Reset/reboot writes require init() before acquisition.
     */
    bool writeRegister(AGRegister reg, uint8_t value);
    /** @brief Writes a documented writable magnetic register; I2C disable is rejected. */
    bool writeRegister(MagRegister reg, uint8_t value);

private:
    // Private Member Variables:
    I2CDevice magnetometer_;
    Config config_{};
    DataTypes::LSM9DS1Data data_{};
    esp_err_t err_ = ESP_OK;
    bool configured_ = false;
    bool has_data_ = false;

    /** @brief Validates configuration enum values before touching hardware. */
    static bool validConfig(const Config& config);
    /** @brief Reads one byte and records transport failure. */
    bool readByte(I2CDevice& device, uint8_t reg, uint8_t& value);
    /** @brief Writes one byte and records transport failure. */
    bool writeByte(I2CDevice& device, uint8_t reg, uint8_t value);
    /** @brief Reads signed little-endian output words using one bus transaction. */
    bool readWords(I2CDevice& device, uint8_t reg, int16_t* values, size_t count);
    /** @brief Polls hardware reset completion with a bounded delay. */
    bool waitReset(I2CDevice& device, uint8_t reg, uint8_t mask);
    /** @brief Records a failed operation and its lifecycle state. */
    bool fail(esp_err_t error, SensorState state = SensorState::ERROR);
};

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics
#endif
