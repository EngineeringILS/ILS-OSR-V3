# ESP-IDF ESP32 Firmware

### Purpose
The current application is an IMU-only service for the onboard computer. It
streams timestamped LSM9DS1 measurements and accepts START, STOP, READ, and INIT
commands over USB Serial/JTAG using SerialIO strings. It starts streaming on
successful initialization and emits only protocol sample, acknowledgment, and
error records. Battery, power-monitor, LED, and interactive test components remain
available for separate hardware tests.

See [protocol, timestamps, and validation](../documentation/timestamped-imu.md)
and the [Python OBC client](../python/imu/README.md).

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
├── main # app_main() and timestamped IMU serial service
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

The application targets FeatherS3TFT, with LSM9DS1 AG at 0x6B and magnetometer
at 0x1E. See [LSM9DS1 wiring and configuration](components/lsm9ds1_driver/README.md).

`firmware/sdkconfig.defaults` disables secondary console output on USB Serial/JTAG.
If using an existing `sdkconfig`, select **Component config → ESP System Settings →
Channel for console secondary output → No secondary console** in `idf.py menuconfig`.
Keep the primary console on UART, not USB Serial/JTAG. Existing sdkconfig choices
override defaults. Early boot text can still appear; the Python client skips it.

### Host Regression Tests
```bash
cmake -S firmware/tests -B firmware/tests/build
cmake --build firmware/tests/build
ctest --test-dir firmware/tests/build --output-on-failure
```

These tests use emulated I2C and serial transports to check conversions,
failed-read recovery, sample preservation, resource ownership, serial framing,
command handling, timestamps, and initialization/stream recovery. The app test
executes the actual main loop against emulated transports. Python tests also
exercise real pyserial through a pseudo-terminal; see the Python client README.

> Note: Shared interfaces remain in `common`; ESP-IDF handles and board-specific operations belong in firmware components.
