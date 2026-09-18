# ESP-IDF ESP32 Firmware

### Purpose
Provide a Wi-Fi/Serial command interface for:
1. Recieving Ground Station commands (over Wi-Fi) and OBC System State (over Serial)
2. Processing commands and monitoring the system state
3. Transmitting OBC System State (over Wi-Fi), Processed Commands (over Serial), and physical Start/Stop/Restart (over GPIO)

### Simplified Communication Flows
1. Ground Station (Wi-Fi) <--> ESP32 (Serial) <--> OBC
2. Ground Station (commands) --> ESP32 (processed commands + physical controls) --> OBC
3. OBC (System State) --> ESP32 (processed System State) --> Ground Station

### Project Structure
1. The ESP32 project is programmed in C++ using the ESP-IDF framework.
2. The `/common` library is incorporated into the `CMakeLists` of this project, ensuring proper compilation and linking.
3. Communication standards are to be defined using the `/common` library framework. 
4. The filesystem structure of this module is below:
```
lunabotics-cdh-dev/firmware
├── components
│   ├── platforms # Board pin mappings and power control
│   ├── i2c_driver # ESP-IDF bus and device wrappers
│   ├── i2c_tools # Interactive bus diagnostics
│   ├── ina3221_driver # Three-channel power monitor
│   ├── lsm9ds1_driver # Nine-axis IMU and die temperature
│   ├── lsm9ds1_test # IMU hardware test output
│   ├── max1704x_driver # Battery fuel gauge
│   ├── led_driver # GPIO LED and SPI NeoPixel drivers
│   ├── SerialIO # USB serial JTAG console
│   ├── ina3221_test # Power monitor hardware test output
│   └── max1704x_test # Fuel gauge hardware test output
├── main # app_main() and interactive hardware test loop
└── tests # Host regression tests with emulated transport

lunabotics-cdh-dev/common
├── include  
│   └── common  
│       ├── drivers # /common base class definitions
│       └── utils   # /common utility definitions
├── src
│   ├── drivers # /common base class implementations
│   └── utils   # /common utility implementations
└── test # /common specific tests, likely un-used for the ESP32 portion of this project.
```

### Development Standards
1. Plan before programming, document program plans in `/documentation`
2. Ensure adherence to `/common` libraries and communication standards.
3. Create unit tests for `/firmware` specific functionality and `/common` specific functionality.

### Build and Test
1. Activate ESP-IDF v5.4.3 and enter `firmware`.
2. Run `idf.py set-target esp32s3`, then `idf.py build`.
3. With a supported board connected, run `idf.py -p <port> flash monitor`.

The current application targets FeatherS3TFT. Commands include `check`, `scan`,
`dump <hex|dec>`, `checkread`, `read`, `imu`, `imuinit`, `blink`, `stopblink`, and `q`.
Hardware test components print measurements; they do not replace automated tests.
See [LSM9DS1 hardware validation](components/lsm9ds1_driver/README.md) for wiring, configuration, and expected output.

### Host Regression Tests
```bash
cmake -S firmware/tests -B firmware/tests/build
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure
```

These tests use emulated I2C and serial transports to check conversions,
failed-read recovery, sample preservation, resource ownership, and long output.

> Note: Shared interfaces remain in `common`; ESP-IDF handles and board-specific operations belong in firmware components.
