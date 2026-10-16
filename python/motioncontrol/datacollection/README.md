# LSM9DS1 Python collection

The collector wraps Adafruit's CircuitPython LSM9DS1 driver and publishes an
immutable `LSM9DS1Data` snapshot. Its fields mirror ROS2 conventions:

- `header.stamp` and `header.frame_id`
- `linear_acceleration` in m/s²
- `angular_velocity` in rad/s
- `magnetic_field` in tesla
- `temperature` in degrees Celsius

The CircuitPython driver reports magnetic field in gauss, so the collector
converts it to tesla before publishing the sample.

## Reading while driving

The I²C properties perform synchronous register transactions. The collector
therefore reads them on one background thread and exposes only the latest
complete sample to control code:

```python
sample = imu_collector.latest(max_age_seconds=0.25)
if sample is not None:
    yaw_rate = sample.angular_velocity.z
```

`latest()` does no I²C work and returns an immutable object while holding its
lock only long enough to copy a reference. A future ROS2 node can translate
the same snapshot into `sensor_msgs/Imu`, `sensor_msgs/MagneticField`, and
`sensor_msgs/Temperature`. Slow logging or publishing should run outside the
drive loop; the drive loop should only fetch the snapshot or enqueue it.

The integrated motion harness discovers the Linux adapter whose sysfs name
contains `i2c-tiny-usb`, opens its dynamic `/dev/i2c-N` node with
`ExtendedI2C`, and gives that same object to the PCA9685 and LSM9DS1. Set
`I2C_BUS_ID=N` to select an adapter explicitly, which is required when more
than one i2c-tiny-usb bridge is attached. The harness starts the collector
before entering the drive loop and stops it during normal or exceptional
shutdown.

The ESP32 bridge must first be bound to Linux as described by its firmware:

```bash
sudo modprobe i2c-tiny-usb delay=1
sudo sh -c "echo 0x303a 0x4020 > /sys/bus/usb/drivers/i2c-tiny-usb/new_id"
i2cdetect -l | grep tiny
```

The `delay=1` module setting requests the bridge's 400 kHz mode; the Linux
driver's default `delay=10` selects the slower mode. The bridge firmware does
not initialize any I²C target while bridge mode is running. Its separate test
console contains INA3221 test code, but that path is unreachable during normal
bridge operation. Independently of firmware, do not physically populate an
INA3221 and the PCA9685 at the same `0x40` address on one bus.

## Motion logs

The motion harness writes `python/log/motion-<UTC timestamp>.csv`. Each new IMU
sample and each command event is joined with three timestamps and the current
control state:

- log timestamp: when the drive loop created the joined record;
- command timestamp: when the current input command was received, or when the
  watchdog generated a stop;
- IMU timestamp: when the collector completed the sensor read;
- input command, active drive mode, selected speed, watchdog state, and turn mode;
- commanded QPPS and the most recent RoboClaw velocity readback for all motors.
- action, status, and message returned by command helpers, including failures
  and shutdown results.

Command events are recorded even if the IMU is absent, stale, or failing; sensor
columns are blank in those rows. Sample-only rows use `action=sample` and are
written only for a new sample. CSV messages retain the full helper result;
operator messages are single-line and capped at 79 visible characters.

Speed selection does not immediately change motor commands. The terminal states
that the selected speed applies on the next drive command, and the CSV keeps
selected speed separate from commanded and cached readback velocities. A failed
hardware command logs `status=error`; it does not certify that motors stopped.

The loop dispatches keys with `match`, polls input without blocking, and sleeps
explicitly for 10 ms after each continuing iteration. The watchdog remains
iteration-based: controller I/O and scheduling add to its elapsed timeout.
Shutdown attempts a zero-speed write to every controller before reporting
failures. Steering is centered only when all stop writes succeed.

Rows are queued without waiting and written by a background thread. If storage
cannot keep up, new rows are dropped instead of delaying the motor loop; the
logger exposes the count through `dropped_records`. Motor readbacks are the
values cached by the most recent motor command transaction, so logging does not
add serial traffic to the control loop.

## Standalone CSV stream

With the dependencies installed and the sensor wired on I²C:

```bash
python3 python/motioncontrol/datacollection/main.py
```
