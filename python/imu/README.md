# OBC IMU serial interface

This Python 3.10+ package talks to the IMU-only ESP32 firmware over USB Serial/JTAG.
It uses pyserial and the firmware's string-based SerialIO protocol. The ESP32
streams fresh LSM9DS1 acceleration, angular velocity, magnetic field, and die
temperature in native sensor axes. Complete records arrive at roughly 20 Hz.

From the repository root:

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install ./python/imu
python -m imu_serial --port /dev/ttyACM0 --count 100
```

Use the appropriate port on the OBC (`/dev/ttyACM0`, a persistent
`/dev/serial/by-id/...` path, or `COM3` on Windows). The CLI prints one JSON object
per sample. It returns a nonzero status on errors and stops streaming on exit
when the connection still works. Only one process may own the serial port;
close `idf.py monitor` before connecting.

```python
from imu_serial import IMUClient

with IMUClient('/dev/ttyACM0', timeout=1.0) as imu:
    imu.stop()  # Synchronize and discard queued samples from before the ACK.
    imu.start()
    for _ in range(100):
        sample = imu.read_sample()
        print(sample.timestamp_us, sample.acceleration_m_s2,
              sample.angular_velocity_rad_s, sample.magnetic_field_ut)
    imu.stop()
```

`read_once()` sends READ and returns a new sample immediately if all sensor data
is ready. It raises `DeviceError` with code `ESP_ERR_NOT_FINISHED` otherwise;
retry later, or use streaming. `reinitialize()` sends INIT and leaves streaming
stopped. Call `start()` after successful initialization. A startup or streaming
failure surfaces as `DeviceError`; INIT can recover past old asynchronous error
records, which remain available as `last_device_error`.

The immutable `ImuSample` exposes:

| Field | Meaning |
| --- | --- |
| `request_id` | 0 for streaming; positive ID for a READ reply |
| `sequence` | Counter across successful acquisitions; resets on MCU boot |
| `timestamp_us` | Integer microseconds since MCU boot, sampled after I2C acquisition |
| `acceleration_m_s2` | Native x/y/z acceleration, m/s² |
| `angular_velocity_rad_s` | Native x/y/z angular velocity, rad/s |
| `magnetic_field_ut` | Native x/y/z magnetic field, µT |
| `temperature_c` | Sensor die temperature, °C |

Timestamps are not UTC, host arrival times, or synchronized hardware sample times.
The separate sensors run at different rates. No orientation fusion, gravity
removal, calibration, or rover-frame transform is applied.

The client is synchronous and is not thread-safe. Each operation has a finite
`timeout`, including the command write. The context manager owns and closes the
transport. `TimeoutError`, `DeviceError`, `ProtocolError`, and serial/OSError
failures are distinguishable. A failed/partial write closes the connection;
reconnect before retrying. A response timeout does not retry the command because
it may already have executed. Subsequent commands use new IDs and ignore stale
replies. Reopening does not automatically reset the MCU.

Streaming samples arriving while waiting for command replies are buffered, up to
`queue_size` (default 64). The oldest is dropped on overflow and `dropped_samples`
increases. Successful STOP and INIT clear this queue. There is no background
reader; call `read_sample()` regularly. Sequence gaps may also reflect firmware
or USB losses. Boot noise is ignored; malformed recognized records raise an error.
Partial received lines survive timeouts; oversized lines are discarded through
the next newline before parsing resumes.

See [wire protocol and firmware validation](../../documentation/timestamped-imu.md).

Tests from the repository root:

```sh
python -m unittest discover -s python/imu/tests -v
# Also verify Python parses the actual C++ formatter output:
IMU_PROTOCOL_TEST_EXECUTABLE="$PWD/firmware/tests/build/imu_protocol_tests" \
    python -m unittest discover -s python/imu/tests -v
```

The POSIX integration test uses a pseudo-terminal and real pyserial. Hardware
validation still requires a connected ESP32 and LSM9DS1.
