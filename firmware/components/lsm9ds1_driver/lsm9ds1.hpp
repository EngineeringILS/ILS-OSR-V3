#include <common/drivers/SensorInterface.hpp>
#include <common/drivers/DataTypes.hpp>
#include <common/drivers/Units.hpp>
#include <common/protocols/InterfaceProtocols.hpp>
#include <i2c_driver.hpp>

#ifndef LUNABOTICS_LSM9DS1_DRIVER_HPP
#define LUNABOTICS_LSM9DS1_DRIVER_HPP

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

using namespace Lunabotics::Common;
using namespace Lunabotics::Common::Sensors;

/**
 * @brief A wrapper class that aligns the LSM9DS1 driver with ILS OSR V3 standards as a SensorInterface.
 *
 * This class provides acceleration, angular velocity, magnetic field, and temperature readings.
 *
 * @note The accelerometer/gyroscope and magnetometer use separate I2C addresses on the same bus.
 */
class LSM9DS1 : public I2CDevice, public SensorInterface {
public:

    // Constructor and Destructor:
    /**
     * @brief Constructor for the LSM9DS1.
     *
     * @param addr The accelerometer/gyroscope I2C address (0x6A or 0x6B).
     * @param i2c_bus The initialized bus, which must outlive this driver.
     * @param mag_addr The magnetometer I2C address (0x1C or 0x1E).
     */
    explicit LSM9DS1(const uint8_t& addr,
                    I2CBus* i2c_bus,
                    const uint8_t& mag_addr = 0x1E
                ) :
                I2CDevice(addr, i2c_bus),
                SensorInterface(Protocols::InterfaceType::I2C, SensorInterface::HostController::ESP32),
                magnetometer_(mag_addr, i2c_bus)
            {}

    /**
     * @brief Releases both device registrations through their I2CDevice destructors.
     */
    virtual ~LSM9DS1() = default;

    // Implementation of the pure virtual functions:

    /**
     * @brief Initializes the LSM9DS1 with continuous measurements on all axes.
     * @return True on successful initialization, false on failure; check getErr().
     * @note Uses +/-2 g, +/-245 dps, +/-4 gauss, 119 Hz AG, and 20 Hz magnetic output.
     */
    bool init() override;

    /**
     * @brief Reads sensor data into memory.
     * Stores a complete sample in the private imu_data_ variable.
     * @return True on a complete sample, false on failure; the previous sample is retained.
     * @note ESP_ERR_NOT_FINISHED indicates that a fresh sample is not ready.
     */
    bool read();

    // Sensor Specific Public Functions:

    /**
     * @brief Copies the last complete sample without performing I2C transactions.
     * @param data The destination for the cached measurements.
     */
    void getData(DataTypes::LSM9DS1Data& data) const;

    /**
     * @brief Gets the most recent initialization or transaction error from either slave.
     */
    esp_err_t getErr() const { return err_; }

private:
    // Private Member Variables:

    /**
     * @brief Internal storage for the last-read sensor data.
     */
    DataTypes::LSM9DS1Data imu_data_{};

    /**
     * @brief The magnetometer device on the shared I2C bus.
     */
    I2CDevice magnetometer_;

    esp_err_t err_ = ESP_OK;
    bool configured_ = false;

    /**
     * @brief LSM9DS1 accelerometer/gyroscope register addresses.
     * Output words are transmitted least-significant byte first.
     */
    enum class AGRegister : uint8_t {
        ACT_THS          = 0x04, // RW: Inactivity threshold and gyro sleep selection.
        ACT_DUR          = 0x05, // RW: Inactivity duration.
        INT_GEN_CFG_XL   = 0x06, // RW: Accelerometer interrupt conditions.
        INT_GEN_THS_X_XL = 0x07, // RW: X acceleration threshold.
        INT_GEN_THS_Y_XL = 0x08, // RW: Y acceleration threshold.
        INT_GEN_THS_Z_XL = 0x09, // RW: Z acceleration threshold.
        INT_GEN_DUR_XL   = 0x0A, // RW: Acceleration interrupt duration.
        REFERENCE_G      = 0x0B, // RW: Gyroscope high-pass reference.
        INT1_CTRL        = 0x0C, // RW: Interrupt 1 routing.
        INT2_CTRL        = 0x0D, // RW: Interrupt 2 routing.
        WHO_AM_I         = 0x0F, // RO: Expected identification 0x68.
        CTRL_REG1_G      = 0x10, // RW: Gyroscope rate, range, and bandwidth.
        CTRL_REG2_G      = 0x11, // RW: Gyroscope output and interrupt filters.
        CTRL_REG3_G      = 0x12, // RW: Gyroscope power and high-pass filter.
        ORIENT_CFG_G     = 0x13, // RW: Gyroscope axis orientation.
        INT_GEN_SRC_G    = 0x14, // RO: Gyroscope interrupt source; read clears latch.
        OUT_TEMP_L       = 0x15, // RO: Temperature low byte.
        OUT_TEMP_H       = 0x16, // RO: Temperature high byte.
        STATUS_REG       = 0x17, // RO: Temperature, gyro, and acceleration readiness.
        OUT_X_L_G        = 0x18, // RO: X angular rate low byte.
        OUT_X_H_G        = 0x19, // RO: X angular rate high byte.
        OUT_Y_L_G        = 0x1A, // RO: Y angular rate low byte.
        OUT_Y_H_G        = 0x1B, // RO: Y angular rate high byte.
        OUT_Z_L_G        = 0x1C, // RO: Z angular rate low byte.
        OUT_Z_H_G        = 0x1D, // RO: Z angular rate high byte.
        CTRL_REG4        = 0x1E, // RW: Gyroscope axis enables and interrupt latch.
        CTRL_REG5_XL     = 0x1F, // RW: Acceleration decimation and axis enables.
        CTRL_REG6_XL     = 0x20, // RW: Acceleration rate, range, and bandwidth.
        CTRL_REG7_XL     = 0x21, // RW: Acceleration filters.
        CTRL_REG8        = 0x22, // RW: Reset, byte order, BDU, and increment mode.
        CTRL_REG9        = 0x23, // RW: Sleep, FIFO, and I2C control.
        CTRL_REG10       = 0x24, // RW: Acceleration and gyroscope self-test excitation.
        INT_GEN_SRC_XL   = 0x26, // RO: Acceleration interrupt source; read clears latch.
        STATUS_REG_ALT   = 0x27, // RO: Alternate status register.
        OUT_X_L_XL       = 0x28, // RO: X acceleration low byte.
        OUT_X_H_XL       = 0x29, // RO: X acceleration high byte.
        OUT_Y_L_XL       = 0x2A, // RO: Y acceleration low byte.
        OUT_Y_H_XL       = 0x2B, // RO: Y acceleration high byte.
        OUT_Z_L_XL       = 0x2C, // RO: Z acceleration low byte.
        OUT_Z_H_XL       = 0x2D, // RO: Z acceleration high byte.
        FIFO_CTRL        = 0x2E, // RW: FIFO mode and threshold.
        FIFO_SRC         = 0x2F, // RO: FIFO count, threshold, and overrun.
        INT_GEN_CFG_G    = 0x30, // RW: Gyroscope interrupt conditions.
        INT_GEN_THS_XH_G = 0x31, // RW: X threshold high byte and counter mode.
        INT_GEN_THS_XL_G = 0x32, // RW: X threshold low byte.
        INT_GEN_THS_YH_G = 0x33, // RW: Y threshold high byte.
        INT_GEN_THS_YL_G = 0x34, // RW: Y threshold low byte.
        INT_GEN_THS_ZH_G = 0x35, // RW: Z threshold high byte.
        INT_GEN_THS_ZL_G = 0x36, // RW: Z threshold low byte.
        INT_GEN_DUR_G    = 0x37  // RW: Gyroscope interrupt duration.
    };

    /**
     * @brief LSM9DS1 magnetometer register addresses.
     * Multiple-byte reads require the register command's increment bit.
     */
    enum class MagRegister : uint8_t {
        OFFSET_X_L       = 0x05, // RW: X hard-iron offset low byte.
        OFFSET_X_H       = 0x06, // RW: X hard-iron offset high byte.
        OFFSET_Y_L       = 0x07, // RW: Y hard-iron offset low byte.
        OFFSET_Y_H       = 0x08, // RW: Y hard-iron offset high byte.
        OFFSET_Z_L       = 0x09, // RW: Z hard-iron offset low byte.
        OFFSET_Z_H       = 0x0A, // RW: Z hard-iron offset high byte.
        WHO_AM_I         = 0x0F, // RO: Expected identification 0x3D.
        CTRL_REG1        = 0x20, // RW: Compensation, XY performance, rate, and self-test.
        CTRL_REG2        = 0x21, // RW: Range, reboot, and reset.
        CTRL_REG3        = 0x22, // RW: Power mode and interface control.
        CTRL_REG4        = 0x23, // RW: Z performance and byte order.
        CTRL_REG5        = 0x24, // RW: Fast read and block data update.
        STATUS_REG       = 0x27, // RO: Axis readiness and overrun.
        OUT_X_L          = 0x28, // RO: X magnetic field low byte.
        OUT_X_H          = 0x29, // RO: X magnetic field high byte.
        OUT_Y_L          = 0x2A, // RO: Y magnetic field low byte.
        OUT_Y_H          = 0x2B, // RO: Y magnetic field high byte.
        OUT_Z_L          = 0x2C, // RO: Z magnetic field low byte.
        OUT_Z_H          = 0x2D, // RO: Z magnetic field high byte.
        INT_CFG          = 0x30, // RW: Magnetic interrupt configuration.
        INT_SRC          = 0x31, // RO: Magnetic interrupt source; read clears latch.
        INT_THS_L        = 0x32, // RW: Magnetic interrupt threshold low byte.
        INT_THS_H        = 0x33  // RW: Magnetic interrupt threshold high seven bits.
    };


    // Constants needed by init():
    static constexpr uint8_t EXPECTED_AG_ID = 0x68;
    static constexpr uint8_t EXPECTED_MAG_ID = 0x3D;

    /**
     * @brief Register helper function (converts scoped enum to uint8_t).
     * @param reg The accelerometer/gyroscope register to convert.
     */
    static constexpr uint8_t reg(AGRegister reg) {
        return static_cast<uint8_t>(reg);
    }

    /**
     * @brief Register helper function (converts scoped enum to uint8_t).
     * @param reg The magnetometer register to convert.
     */
    static constexpr uint8_t reg(MagRegister reg) {
        return static_cast<uint8_t>(reg);
    }

    /**
     * @brief Reads one byte from the selected device and records transaction errors.
     */
    bool readByte(I2CDevice& device, uint8_t reg, uint8_t& value);

    /**
     * @brief Writes one byte to the selected device and records transaction errors.
     */
    bool writeByte(I2CDevice& device, uint8_t reg, uint8_t value);

    /**
     * @brief Reads signed little-endian output words in one transaction.
     */
    bool readWords(I2CDevice& device, uint8_t reg, int16_t* values, size_t count);

    /**
     * @brief Polls reset completion with a bounded delay.
     */
    bool waitReset(I2CDevice& device, uint8_t reg, uint8_t mask);

    /**
     * @brief Records an operation failure and updates the sensor state.
     */
    bool fail(esp_err_t error, SensorState state = SensorState::ERROR);
};

} // namespace Drivers
} // namespace ESP32
} // namespace Lunabotics

#endif
