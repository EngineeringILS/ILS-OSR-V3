# Timestamped IMU migration plan

Base: `feature/codex/lsm9ds1` at `ebb9f50`, which supplies the working LSM9DS1
driver. Feature branch: `feature/codex/timestamped-imu`.

1. Replace the hardware-test menu in `firmware/main` with a single IMU service.
   Keep the FeatherS3TFT wiring and existing driver conversion/readiness checks.
2. Add bounded, nonblocking, non-echoing line input to SerialIO. Retain the
   interactive methods for existing test components.
3. Stream fresh nine-axis measurements and temperature as versioned ASCII CSV.
   Use integer microseconds since ESP32 boot, captured just after acquisition,
   and a sequence number. Do not imply wall-clock synchronization or simultaneous
   acquisition by the accelerometer, gyroscope, and magnetometer.
4. Accept correlated START, STOP, READ, and INIT commands. Separate acknowledgments
   and errors from sample records; reject invalid and overlong commands.
5. Add an independently installable Python client for the OBC, with typed samples,
   finite I/O timeouts, bounded buffering, and context-managed serial ownership.
6. Verify serial framing, conversions, command handling, Python stream/response
   interleaving, and errors using host tests; build the ESP32-S3 firmware. Document
   physical validation separately because emulated transports cannot verify USB
   timing, sensor wiring, or timestamp behavior on a board.

The existing driver requires fresh AG and magnetometer data. Its 20 Hz magnetic
output limits complete records to approximately 20 Hz. No fusion, motor control,
power monitoring, or interactive prompts belong in the new main loop.

## Wire protocol v1

Transport: USB Serial/JTAG, ASCII, one LF-terminated record per line. The Python
client configures 115200 baud; native USB transport does not use UART bit timing.
Commands accept CRLF as well. There are no prompts or command echoes. The firmware
accepts up to 127 printable command bytes, discarding invalid/overlong lines until
LF. One command is processed per loop; partial input cannot block sampling.

```text
CMD,<id>,START
CMD,<id>,STOP
CMD,<id>,READ
CMD,<id>,INIT
ACK,<id>,<action>
ERR,<id>,<code>
IMU,1,<id>,<sequence>,<timestamp_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>,<temperature_c>
```

Commands and actions are case-sensitive. IDs are decimal integers from 1 through
4294967295. ID 0 is reserved for unsolicited samples/errors. IMU records have
exactly 15 fields. Sequence is an unsigned 64-bit integer starting at 0; timestamp
is a nonnegative signed 64-bit integer from `esp_timer_get_time()`, captured just
after a successful complete I2C read. Both reset on reboot, not on INIT. The
sequence advances for READ samples as well as streamed samples. Values use m/s²,
rad/s, µT, and °C in native sensor axes. The default driver ranges/rates are unchanged.

| Command | Success response | Effect |
| --- | --- | --- |
| START | `ACK,<id>,START` | Enable streaming; requires successful initialization |
| STOP | `ACK,<id>,STOP` | Disable streaming before sending the ACK |
| READ | `IMU,1,<id>,...` | Acquire fresh data now; no separate ACK |
| INIT | `ACK,<id>,INIT` | Stop streaming and reinitialize; START must resume it |

READ responds with `ERR,<id>,ESP_ERR_NOT_FINISHED` when a fresh complete sample is
not ready. During streaming this condition produces no record. Other acquisition
errors emit `ERR,0,<ESP error name>` once and stop streaming. Initialization failure
leaves the command interface running so INIT can retry; board power/port or serial
setup failures require a restart. No previous cached sample is emitted as new data.

Malformed commands use `ERR,0,BAD_COMMAND`. Unknown actions with a valid command
ID use `ERR,<id>,BAD_COMMAND`. START before successful initialization returns
`ESP_ERR_INVALID_STATE`. Firmware processes commands sequentially; the host must
use one command caller at a time and correlate replies by ID. A response timeout
can leave command execution uncertain; the Python client does not automatically
retry it. Sustained USB backpressure can delay firmware polling, so this service
is not a hard real-time acquisition or lossless logging system.

## Validation

Automated checks cover the actual main loop with emulated I2C/serial, partial and
oversized input, recovery from initialization/read failures, C++ to Python sample
format compatibility, bounded Python buffering, finite reads/writes, stale replies,
and a real pyserial round trip over a POSIX pseudo-terminal. Build with ESP-IDF
v5.4.3 for ESP32-S3. See the firmware and Python READMEs for test commands.

Physical acceptance checks (require the board; not performed by host tests):

1. Connect FeatherS3TFT and LSM9DS1 using the driver README wiring; flash the build.
2. Close the serial monitor and run the Python CLI. Verify plausible stationary
   acceleration (gravity), near-zero angular rates, magnetic field, and die temperature.
3. Collect for several minutes: timestamps and sequence must increase, with
   approximately 20 complete records per second under normal conditions.
4. STOP and confirm no samples follow its ACK; READ repeatedly with suitable delays;
   INIT then START and verify continued sequence/time without resetting the MCU.
5. Disconnect/reconnect USB, and test missing-sensor startup and recovery. Verify
   errors surface and the OBC can reconnect/reinitialize without interpreting boot
   text or partial lines as samples. An MCU reboot resets its clock/sequence; the OBC
   must treat this as a new time epoch.
