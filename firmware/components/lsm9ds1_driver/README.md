### LSM9DS1 IMU Library
- Wraps the LSM9DS1 as a `SensorInterface` and `I2CDevice`, following the Max1704x and INA3221 driver structure.
- Uses a second private `I2CDevice` for the magnetometer on the same bus.
- `init()` identifies and resets both slaves, then enables continuous measurements.
- `read()` stores a complete sample; `getData()` copies the last successful sample.
- Register maps and conversion helpers remain private.

### Configuration
- Constructor: `LSM9DS1 imu(0x6B, &i2cBus0, 0x1E);`
- Alternative AG and magnetic addresses: `0x6A` and `0x1C`.
- Initialization uses +/-2 g, +/-245 dps, +/-4 gauss, 119 Hz AG, and 20 Hz magnetic output.
- The bus must outlive the driver. Call `init()` again to reset and reinitialize both sensing blocks.

### Shared Data
| Field | Units | Meaning |
|-------|-------|---------|
| `acceleration.a_x/a_y/a_z` | m/s^2 | Includes gravity |
| `angular_velocity.x/y/z` | rad/s | Native gyroscope axes |
| `magnetic_field.x/y/z` | tesla | Native magnetometer axes |
| `temperature` | kelvin | Internal die temperature |
| `timestamp` | steady clock | Host acquisition completion |
| `ag_status`, `mag_status` | raw bytes | Readiness and overrun before acquisition |

### Usage Notes
- Use data only after `read()` succeeds. Failed reads preserve the previous sample.
- `getErr()` reports errors from either slave; `ESP_ERR_NOT_FINISHED` means a fresh sample is not ready.
- Recoverable communication failures permit another `read()`; initialization failures require `init()`.
- Temperature is the latest available value. The sensing blocks are not synchronized.
- Measurements retain native chip axes; no sensor fusion or magnetometer axis remapping is applied.
- Serialize access from multiple tasks. The inherited I2C helpers refer to the AG slave only.

### Hardware Validation
1. Connect VIN to 3.3 V, GND, SDA, and SCL on the FeatherS3TFT I2C bus. Keep chip selects high.
2. Build and flash with ESP-IDF v5.4.3, target `esp32s3`.
3. Run `scan` and confirm both configured addresses. Run `imuinit`.
4. Run `imu` while stationary and while rotating. Stationary acceleration magnitude should be near 9.81 m/s^2, with angular rate near zero plus sensor bias.
5. Check magnetic response and axis signs. Repeat `imuinit` and verify acquisition resumes.

### Functionality
- [x] SensorInterface and I2CDevice integration
- [x] Complete private register address maps
- [x] Unit-aware nine-axis and temperature acquisition
- [x] Host regression tests and interactive hardware harness
- [ ] Hardware validation
- [ ] Maintainer code review after hardware validation

### References
- [ST LSM9DS1 datasheet, Rev 3](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
- [Adafruit LSM9DS1 pinouts](https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/pinouts)
