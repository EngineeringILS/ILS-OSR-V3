### LSM9DS1 IMU Library
- Derives from `SensorInterface` and uses `I2CDevice` for both internal slaves.
- Supports Adafruit LSM9DS1 I2C boards; LSM9DS0 is a different device and is rejected by identification checks.
- Defaults: AG address `0x6B`, magnetic address `0x1E`, +/-2 g, +/-245 dps, +/-4 gauss, AG 119 Hz, magnetic 20 Hz.
- Alternative addresses are `0x6A` and `0x1C`; select them explicitly to match the board straps.

### Shared Data
| Field | Units | Meaning |
|-------|-------|---------|
| `acceleration.a_x/a_y/a_z` | m/s^2 | Includes gravity |
| `angular_velocity.x/y/z` | rad/s | Native gyroscope axes |
| `magnetic_field.x/y/z` | tesla | Native magnetometer axes |
| `temperature` | kelvin | Internal die temperature |
| `timestamp` | steady clock | Host completion time |
| `ag_status`, `mag_status` | raw bytes | Readiness and overrun flags before acquisition |

### Lifecycle and Configuration
1. Construct with an initialized bus, which must outlive the driver.
2. Call `init()` or `init(config)` to identify, reset, and configure both slaves.
3. Poll `read()`, then use `getData()` after success. `hasData()` distinguishes an acquired sample from initial storage.
4. `configure(config)` resets and applies all ranges/rates. `reset()` reapplies the last successful high-level configuration.
5. `powerDown()` stops sampling; `deinit()` also releases both I2C registrations. The destructor releases registrations without bus transactions.

`read()` requires fresh acceleration, gyro, and magnetic data. It returns false
with `ESP_ERR_NOT_FINISHED` while waiting. Transport errors set `ERROR` and allow
retry; failed initialization sets `FAILED` and requires `init()`/`reset()`.
Failed reads preserve the last complete sample. Temperature is the latest available
value and may not be fresh for every sample. Acquisition is limited by the slowest
sensor; the three blocks are not synchronized.

### Example
```cpp
#include <lsm9ds1.hpp>

using namespace Lunabotics::ESP32::Drivers;

LSM9DS1 imu(&i2cBus0);
LSM9DS1::Config config;
config.accel_range = LSM9DS1::AccelRange::G4;
config.gyro_range = LSM9DS1::GyroRange::DPS500;

if (imu.init(config) && imu.read()) {
    Lunabotics::Common::DataTypes::LSM9DS1Data data;
    imu.getData(data);
}
// Poll read() again from the application loop if the first sample is not ready.
```

### Register Access
- `AGRegister` and `MagRegister` enumerate every documented register in ST Tables 21 and 22, including FIFO, filters, offsets, interrupts, and self-test controls.
- Typed `readRegister()`/`writeRegister()` select the correct slave. Reserved addresses and writes to read-only registers are rejected.
- Supply datasheet-valid bit fields for advanced writes; the driver does not validate every reserved bit or filter combination. I2C-disable writes are rejected.
- Raw settings are cleared by `init()`, `reset()`, and `configure()`; `getConfig()` reports the last high-level configuration only.
- High-level `read()` requires all axes active, FIFO bypass, continuous magnetometer conversion, little-endian 16-bit output, BDU, and AG address increment. It rejects incompatible settings and reads hardware ranges before conversion.
- FIFO data can be read through its documented output registers. Interrupt routing and self-test excitation are accessible through registers; ISR handling and a calibrated self-test pass/fail procedure belong to the application.
- Reading output registers consumes readiness; reading interrupt-source registers can clear latched interrupts. Serialize diagnostics with acquisition.

### Hardware Validation
1. Connect VIN to 3.3 V, GND, SDA, and SCL on the FeatherS3TFT I2C bus. Keep chip selects high for I2C operation.
2. Build and flash the firmware with ESP-IDF v5.4.3 and target `esp32s3`.
3. Run `scan`; confirm both configured addresses. Run `imuinit` to check identification and initialization.
4. Run `imu` repeatedly while stationary and rotating the board. Stationary acceleration magnitude should be near 9.81 m/s^2; angular rate should be near zero with sensor bias.
5. Verify magnetic response and axis signs against the board orientation; magnetometer axes are not remapped into the AG frame.
6. Run `imustop`, then `imuinit` and `imu` to verify restart. Validate reconnection and non-default ranges before review.

### Functionality
- [x] SensorInterface lifecycle and complete register address maps
- [x] Unit-aware nine-axis and temperature acquisition
- [x] Range/rate configuration, reset, power-down, and diagnostics
- [x] Host regression tests and interactive hardware harness
- [ ] Hardware validation
- [ ] Maintainer code review after hardware validation

### References
- [ST LSM9DS1 datasheet, Rev 3](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
- [Adafruit LSM9DS1 pinouts](https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/pinouts)
