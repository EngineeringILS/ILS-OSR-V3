import csv
import time
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType
from unittest.mock import patch

from datacollection.data_methods import LSM9DS1Collector, create_lsm9ds1_i2c
from datacollection.data_types import Time
from datacollection.i2c_linux import create_i2c_tiny_usb_bus, find_i2c_adapter
from datacollection.motion_logging import MotionCSVLogger


class FakeLSM9DS1:
    acceleration = (1.0, 2.0, 3.0)
    gyro = (0.1, 0.2, 0.3)
    magnetic = (0.4, 0.5, 0.6)
    temperature = 24.5


class MutableClock:
    def __init__(self, value: int) -> None:
        self.value = value

    def __call__(self) -> int:
        return self.value


class FakeMotor:
    def __init__(self, motor_id: int, name: str, speed: int, actual_speed: int) -> None:
        self.id = motor_id
        self.name = name
        self.speed = speed
        self.actual_speed = actual_speed


class LSM9DS1CollectorTests(unittest.TestCase):
    def test_collect_once_builds_ros_style_sample_and_converts_gauss(self) -> None:
        monotonic = MutableClock(10_000_000_000)
        collector = LSM9DS1Collector(
            FakeLSM9DS1(),
            frame_id="base_imu",
            clock_ns=lambda: 12_345_678_901,
            monotonic_ns=monotonic,
        )

        sample = collector.collect_once()

        self.assertEqual(sample.header.stamp, Time(sec=12, nanosec=345_678_901))
        self.assertEqual(sample.header.frame_id, "base_imu")
        self.assertEqual(sample.sequence, 0)
        self.assertEqual(sample.linear_acceleration.x, 1.0)
        self.assertEqual(sample.angular_velocity.z, 0.3)
        self.assertAlmostEqual(sample.magnetic_field.x, 0.00004)
        self.assertAlmostEqual(sample.magnetic_field.z, 0.00006)
        self.assertEqual(sample.temperature, 24.5)
        self.assertIs(collector.latest(), sample)

        with self.assertRaises(FrozenInstanceError):
            sample.temperature = 100.0

    def test_latest_rejects_stale_sample(self) -> None:
        monotonic = MutableClock(1_000_000_000)
        collector = LSM9DS1Collector(FakeLSM9DS1(), monotonic_ns=monotonic)
        collector.collect_once()

        monotonic.value += 250_000_001

        self.assertIsNone(collector.latest(max_age_seconds=0.25))
        self.assertIsNotNone(collector.latest())

    def test_background_collection_starts_and_stops(self) -> None:
        collector = LSM9DS1Collector(FakeLSM9DS1(), sample_hz=100.0)
        collector.start()
        deadline = time.monotonic() + 0.5
        while collector.latest() is None and time.monotonic() < deadline:
            time.sleep(0.005)
        collector.stop()

        self.assertIsNotNone(collector.latest())
        self.assertFalse(collector.is_running)

    def test_factory_uses_supplied_bus_and_addresses(self) -> None:
        calls = []
        fake_driver = ModuleType("adafruit_lsm9ds1")
        fake_driver.LSM9DS1_I2C = lambda bus, **kwargs: calls.append((bus, kwargs)) or FakeLSM9DS1()
        bus = object()

        with patch.dict("sys.modules", {"adafruit_lsm9ds1": fake_driver}):
            sensor = create_lsm9ds1_i2c(bus, mag_address=0x1C, xg_address=0x6A)

        self.assertIsInstance(sensor, FakeLSM9DS1)
        self.assertEqual(calls, [(bus, {"mag_address": 0x1C, "xg_address": 0x6A})])

    def test_discovers_i2c_tiny_usb_linux_bus(self) -> None:
        with TemporaryDirectory() as directory:
            sysfs_root = Path(directory)
            onboard = sysfs_root / "i2c-1"
            bridge = sysfs_root / "i2c-7"
            onboard.mkdir()
            bridge.mkdir()
            (onboard / "name").write_text("NVIDIA I2C adapter", encoding="utf-8")
            (bridge / "name").write_text(
                "i2c-tiny-usb at bus 001 device 016",
                encoding="utf-8",
            )

            self.assertEqual(find_i2c_adapter(sysfs_root=sysfs_root), 7)

    def test_extended_bus_factory_uses_explicit_linux_bus(self) -> None:
        calls = []
        fake_extended_bus = ModuleType("adafruit_extended_bus")
        fake_extended_bus.ExtendedI2C = (
            lambda bus_id, **kwargs: calls.append((bus_id, kwargs)) or object()
        )

        with patch.dict("sys.modules", {"adafruit_extended_bus": fake_extended_bus}):
            create_i2c_tiny_usb_bus(9, frequency=100_000)

        self.assertEqual(calls, [(9, {"frequency": 100_000})])

    def test_extended_bus_factory_defaults_to_400_khz(self) -> None:
        calls = []
        fake_extended_bus = ModuleType("adafruit_extended_bus")
        fake_extended_bus.ExtendedI2C = (
            lambda bus_id, **kwargs: calls.append((bus_id, kwargs)) or object()
        )

        with patch.dict("sys.modules", {"adafruit_extended_bus": fake_extended_bus}):
            create_i2c_tiny_usb_bus(9)

        self.assertEqual(calls, [(9, {"frequency": 400_000})])

    def test_command_result_csv_without_imu_preserves_fields_and_message(self) -> None:
        motors = [FakeMotor(0, "Left", 100, 90)]
        message = 'Controller said "failed", retry required\n' + "x" * 100
        with TemporaryDirectory() as directory:
            path = Path(directory) / "motion.csv"
            with MotionCSVLogger(path, motors) as logger:
                logger.submit(None, command_timestamp_ns=6_000_000_007, input_command="w", drive_mode="unknown", selected_speed_qpps=100, watchdog_ticks_remaining=0, inplace_turn=False, motors=motors, action="control_error", status="error", message=message)
            with path.open(newline="", encoding="utf-8") as log_file:
                rows = list(csv.DictReader(log_file))
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertNotIn(None, row)
        self.assertEqual(row["message"], message)
        self.assertEqual(row["action"], "control_error")
        self.assertEqual(row["status"], "error")
        self.assertEqual(row["imu_sec"], "")
        self.assertEqual(row["temperature_c"], "")
        self.assertEqual(row["motor_0_left_command_qpps"], "100")
        self.assertEqual(row["motor_0_left_readback_qpps"], "90")

    def test_motion_logger_joins_timestamps_commands_and_velocities(self) -> None:
        collector = LSM9DS1Collector(
            FakeLSM9DS1(),
            clock_ns=lambda: 5_000_000_006,
            monotonic_ns=lambda: 1,
        )
        sample = collector.collect_once()
        motors = [
            FakeMotor(0, "Front Left", 1000, 950),
            FakeMotor(1, "Front Right", 1000, 960),
        ]

        with TemporaryDirectory() as directory:
            path = Path(directory) / "motion.csv"
            with MotionCSVLogger(path, motors, clock_ns=lambda: 7_000_000_008) as logger:
                accepted = logger.submit(
                    sample,
                    command_timestamp_ns=6_000_000_007,
                    input_command="w",
                    drive_mode="forward",
                    selected_speed_qpps=1000,
                    watchdog_ticks_remaining=49,
                    inplace_turn=False,
                    motors=motors,
                )

            self.assertTrue(accepted)
            with path.open(newline="", encoding="utf-8") as log_file:
                rows = list(csv.DictReader(log_file))

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["log_sec"], "7")
        self.assertEqual(rows[0]["log_nanosec"], "8")
        self.assertEqual(rows[0]["command_sec"], "6")
        self.assertEqual(rows[0]["imu_sec"], "5")
        self.assertEqual(rows[0]["drive_mode"], "forward")
        self.assertEqual(rows[0]["motor_0_front_left_command_qpps"], "1000")
        self.assertEqual(rows[0]["motor_1_front_right_readback_qpps"], "960")


if __name__ == "__main__":
    unittest.main()
