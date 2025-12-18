### Lunabotics ESP32 I2C_Tools Component
- This library provides I2C debugging tools to provide manual hardware testing capability for the `i2c_driver` library.
- This class must be completed in order to comply with Project Design Principles.
Functionality
- [ ] Code Functionally Complete
    - [ ] Proper Commandline Wrapper to Parse I2C Commands from SerialIO (e.g. supports a full command line: "i2c scan 0x36")
    - [x] Bus status method (i2c_status)
    - [x] Device status method (i2c_device_status)
    - [x] Device register read method (i2c_device_read)
    - [x] Bus scan method (i2c_scan)
    - [x] Manual Bus Device dump method (i2c_dump)

Requires
1. `i2c_driver` for I2C Bus and Device definitions & methods.
2. `SerialIO` for IO capability.
