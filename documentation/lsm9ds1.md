# LSM9DS1 Integration

### Implementation Plan
1. Derive the IMU from `SensorInterface` and use `I2CDevice` for each internal slave.
2. Define both complete register maps, lifecycle methods, configuration, and unit-aware samples.
3. Add an interactive hardware test and host regression tests.
4. Validate the ESP32-S3 build and submit a draft PR for hardware validation before review.

### References
- [ST LSM9DS1 datasheet, Rev 3](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)
- [Adafruit board and pinouts](https://learn.adafruit.com/adafruit-lsm9ds1-accelerometer-plus-gyro-plus-magnetometer-9-dof-breakout/pinouts)

### Validation
- [x] ESP32-S3 firmware build with ESP-IDF v5.4.3; no compiler warnings
- [x] Common test and both firmware host test executables pass
- [ ] Hardware validation by maintainer
- [ ] Code review after hardware validation

### Hardware Test Entry Point
- [Driver usage and hardware checklist](../firmware/components/lsm9ds1_driver/README.md)
- Console commands: `scan`, `imuinit`, `imu`, and `imustop`.
- Feature branch includes the preceding sensor-integration repair commit required for the firmware build.
