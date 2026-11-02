# LSM9DS1 Linux IMU wrapper

A self-contained C11 library and CLI for an ST LSM9DS1 connected to Linux through an ESP32 USB-I2C bridge presented by the `i2c-tiny-usb` kernel driver. It uses native `i2c-dev`; there is no Arduino, Teensy, or Python runtime dependency.

## Electrical assumptions

Use 3.3 V logic and power appropriate to the sensor board. Confirm that the particular breakout includes any required regulation and I2C pull-ups. Do not connect a 5 V-only bus. This software accesses only the configured LSM9DS1 addresses.

Defaults are accelerometer/gyro address `0x6b` and magnetometer address `0x1e`. Legal alternates `0x6a` and `0x1c` are accepted.

## Build and test

```sh
cmake -S . -B build -DLSM9DS1_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The public API is `include/lsm9ds1.h`. The portable core takes injected register read, register write, and monotonic-clock callbacks. The Linux adapter is isolated in `include/lsm9ds1_linux_i2c.h` and `src/linux_i2c.c`. The library sources are C11; the hardware-free test executable uses C++17.

## Usage

```sh
./build/lsm9ds1-cli --help
./build/lsm9ds1-cli --probe
./build/lsm9ds1-cli --bus /dev/i2c-11 --probe
./build/lsm9ds1-cli --init --samples 20
./build/lsm9ds1-cli --xg-address 0x6a --mag-address 0x1c --probe
```

Without `--bus`, discovery scans `/sys/class/i2c-adapter/*/name` and requires exactly one adapter name containing `i2c-tiny-usb`. It fails clearly when none or several match. `/dev/i2c-11` is a possible current assignment, not a stable name.

`--probe` reads only WHO_AM_I (`0x0f`) and expects `0x68` from the accelerometer/gyro and `0x3d` from the magnetometer. It performs no configuration writes. Sampling is bounded by `--samples N` and requires `--init`, making configuration writes explicit.

The Linux read callback uses one `I2C_RDWR` transaction containing a register-address write followed by a read, preserving the repeated start. Your account needs read/write permission on the selected `/dev/i2c-N`, commonly via a site-specific udev rule or membership in the `i2c` group. Avoid running as root merely to evade a permissions problem.

## Explicit configuration and units

Initialization verifies both identities, then configures:

- BDU enabled, little-endian output, and XG register-address auto-increment through `CTRL_REG8`.
- Accelerometer XYZ, 119 Hz, +/-2 g (`0.061 mg/LSB`). Output is m/s^2 using standard gravity 9.80665 m/s^2.
- Gyroscope XYZ, 119 Hz, +/-245 dps (`8.75 mdps/LSB`). Output is rad/s.
- Magnetometer temperature compensation, ultra-high-performance XYZ, 20 Hz continuous conversion, +/-4 gauss (`0.14 mgauss/LSB`), little endian and BDU. Output is tesla.
- Temperature output is converted as `25 + raw/16` degrees Celsius. It is sensor die temperature, not calibrated ambient temperature.

Each complete sample carries a `CLOCK_MONOTONIC` timestamp in nanoseconds and a status. Reads are staged: any I2C or clock failure returns an error while preserving all values and the timestamp from the last complete sample. No reset or readiness polling loop is used.

Register settings and sensitivities follow the ST LSM9DS1 datasheet and ST's `lsm9ds1-pid` reference driver.

## Limitations

- Hardware-free tests validate the core callback contract, identity rejection, legal alternate addresses, exact configuration bytes, signed little-endian conversions/scales, error propagation, and last-complete-sample preservation. They do not establish electrical integrity or accuracy on a physical sensor.
- There is no calibration, bias estimation, axis remapping, FIFO use, data-ready synchronization, or magnetic hard/soft-iron compensation.
- The CLI samples every 50 ms, matching the configured 20 Hz magnetometer rate; it does not promise phase-coherent accel/gyro/magnetometer measurements.
- The Linux backend is Linux-specific and requires kernel `i2c-dev` plus an adapter supporting `I2C_RDWR`. `O_NONBLOCK` is requested when opening the device, but adapter/kernel I2C transaction timing is not controlled by this library.
