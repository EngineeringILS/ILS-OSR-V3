"""
imu_wrapper.py
Python binding of the LSM9DS1 C driver in lsm9ds1_driver, with state change measurement.

Build the driver on the target before use:
    cmake -S lsm9ds1_driver -B lsm9ds1_driver/build -DBUILD_SHARED_LIBS=ON
    cmake --build lsm9ds1_driver/build
"""

from __future__ import annotations

import ctypes
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

try:
    from .data_types import Header, LSM9DS1Data, Time, Vector3
except ImportError:  # Allow direct execution from this directory.
    from data_types import Header, LSM9DS1Data, Time, Vector3


DEFAULT_LIBRARY_DIR = Path(__file__).resolve().parent / "lsm9ds1_driver" / "build"
LSM9DS1_OK = 0
ERROR_BUFFER_BYTES = 256


class _IO(ctypes.Structure):
    # Function pointers are only passed through to the driver, never called here.
    _fields_ = [
        ("user", ctypes.c_void_p),
        ("read", ctypes.c_void_p),
        ("write", ctypes.c_void_p),
        ("monotonic_clock", ctypes.c_void_p),
    ]


class _Config(ctypes.Structure):
    _fields_ = [("xg_address", ctypes.c_uint8), ("m_address", ctypes.c_uint8)]


class _Sample(ctypes.Structure):
    _fields_ = [
        ("acceleration_m_s2", ctypes.c_double * 3),
        ("angular_rate_rad_s", ctypes.c_double * 3),
        ("magnetic_field_t", ctypes.c_double * 3),
        ("die_temperature_c", ctypes.c_double),
        ("monotonic_timestamp_ns", ctypes.c_uint64),
        ("status", ctypes.c_int),
    ]


class _Device(ctypes.Structure):
    _fields_ = [
        ("io", _IO),
        ("config", _Config),
        ("last_complete", _Sample),
        ("initialized", ctypes.c_int),
        ("has_complete_sample", ctypes.c_int),
    ]


class _Bus(ctypes.Structure):
    _fields_ = [("fd", ctypes.c_int)]


@dataclass(frozen=True, slots=True)
class StateChange:
    """Measured change across one movement, in the sensor's axes.

    delta_rotation_rad is the bias-corrected gyro integral about each axis, valid
    for small tilt. tilt_change_rad compares gravity while stationary before and
    after the movement. largest_gap_s and read_errors indicate measurement quality.
    samples holds every sample taken, including the stationary and settle periods.
    """

    duration_s: float
    sample_count: int
    read_errors: int
    largest_gap_s: float
    gyro_bias_rad_s: Vector3
    delta_rotation_rad: Vector3
    peak_rate_rad_s: Vector3
    start_acceleration: Vector3
    end_acceleration: Vector3
    tilt_change_rad: float
    peak_acceleration_change: float
    samples: tuple[LSM9DS1Data, ...] = field(default=(), repr=False)


def _mean(vectors: list[Vector3]) -> Vector3:
    count = len(vectors)
    return Vector3(sum(v.x for v in vectors) / count, sum(v.y for v in vectors) / count, sum(v.z for v in vectors) / count)


def _tilt_between(first: Vector3, second: Vector3) -> float:
    dot = first.x * second.x + first.y * second.y + first.z * second.z
    norm = math.hypot(first.x, first.y, first.z) * math.hypot(second.x, second.y, second.z)
    if norm == 0.0:
        return 0.0
    return math.acos(max(-1.0, min(1.0, dot / norm)))


class IMUWrapper:
    """Owns one LSM9DS1 through the C driver and measures movements.

    Initialization configures the sensor registers. Do not run another driver
    against the same sensor concurrently.

    Args:
        bus_path: I2C device path. Discovers the i2c-tiny-usb adapter when None.
        library_dir: Directory holding liblsm9ds1.so and liblsm9ds1_linux.so.
        xg_address: Accelerometer and gyro I2C address.
        m_address: Magnetometer I2C address.
        frame_id: Frame recorded on each sample.

    Raises:
        RuntimeError: If the library, bus or sensor is unavailable.
    """

    def __init__(self, bus_path: Optional[str] = None, library_dir: Optional[Path] = None,
                 xg_address: int = 0x6B, m_address: int = 0x1E, frame_id: str = "imu_link"):
        directory = Path(library_dir) if library_dir is not None else DEFAULT_LIBRARY_DIR
        try:
            self._core = ctypes.CDLL(str(directory / "liblsm9ds1.so"))
            self._linux = ctypes.CDLL(str(directory / "liblsm9ds1_linux.so"))
        except OSError as e:
            raise RuntimeError(f"[IMUWRAPPER] Driver library not loaded, see module header: {e}") from e

        self._core.lsm9ds1_status_string.restype = ctypes.c_char_p
        self._core.lsm9ds1_status_string.argtypes = [ctypes.c_int]
        self._linux.lsm9ds1_linux_discover.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
        self._linux.lsm9ds1_linux_open.argtypes = [ctypes.POINTER(_Bus), ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
        self._linux.lsm9ds1_linux_close.argtypes = [ctypes.POINTER(_Bus)]
        self._linux.lsm9ds1_linux_close.restype = None
        self._linux.lsm9ds1_linux_io.argtypes = [ctypes.POINTER(_Bus)]
        self._linux.lsm9ds1_linux_io.restype = _IO
        self._core.lsm9ds1_open.argtypes = [ctypes.POINTER(_Device), ctypes.POINTER(_IO), ctypes.POINTER(_Config)]
        self._core.lsm9ds1_initialize.argtypes = [ctypes.POINTER(_Device)]
        self._core.lsm9ds1_read_sample.argtypes = [ctypes.POINTER(_Device), ctypes.POINTER(_Sample)]

        self._frame_id = frame_id
        self._sequence = 0
        self._bus = _Bus(-1)
        self._device = _Device()
        self._open = False

        error = ctypes.create_string_buffer(ERROR_BUFFER_BYTES)
        if bus_path is None:
            found = ctypes.create_string_buffer(ERROR_BUFFER_BYTES)
            if self._linux.lsm9ds1_linux_discover(found, len(found), error, len(error)) != 0:
                raise RuntimeError(f"[IMUWRAPPER] Bus discovery failed: {error.value.decode(errors='replace')}")
            device_path = found.value
        else:
            device_path = bus_path.encode()

        if self._linux.lsm9ds1_linux_open(ctypes.byref(self._bus), device_path, error, len(error)) != 0:
            raise RuntimeError(f"[IMUWRAPPER] Bus open failed: {error.value.decode(errors='replace')}")
        self._open = True

        try:
            self._io = self._linux.lsm9ds1_linux_io(ctypes.byref(self._bus))
            config = _Config(xg_address, m_address)
            self._check(self._core.lsm9ds1_open(ctypes.byref(self._device), ctypes.byref(self._io), ctypes.byref(config)))
            self._check(self._core.lsm9ds1_initialize(ctypes.byref(self._device)))
        except Exception:
            self.close()
            raise

    def _check(self, status: int):
        if status != LSM9DS1_OK:
            raise RuntimeError(f"[IMUWRAPPER] Driver error: {self._core.lsm9ds1_status_string(status).decode()}")

    def close(self):
        """Release the I2C bus. Repeated calls are harmless."""
        if self._open:
            self._linux.lsm9ds1_linux_close(ctypes.byref(self._bus))
            self._open = False

    def __enter__(self) -> "IMUWrapper":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()

    def read(self) -> LSM9DS1Data:
        """Read one complete sample. The header stamp is CLOCK_MONOTONIC.

        Raises:
            RuntimeError: If the driver reports a failure.
        """
        sample = _Sample()
        self._check(self._core.lsm9ds1_read_sample(ctypes.byref(self._device), ctypes.byref(sample)))
        data = LSM9DS1Data(
            header=Header(stamp=Time.from_nanoseconds(sample.monotonic_timestamp_ns), frame_id=self._frame_id),
            linear_acceleration=Vector3.from_iterable(sample.acceleration_m_s2),
            angular_velocity=Vector3.from_iterable(sample.angular_rate_rad_s),
            magnetic_field=Vector3.from_iterable(sample.magnetic_field_t),
            temperature=sample.die_temperature_c,
            sequence=self._sequence,
        )
        self._sequence = self._sequence + 1
        return data

    def measure(self, movement: Callable[[], object], stationary_s: float = 1.0,
                settle_s: float = 0.5, sample_hz: float = 100.0) -> StateChange:
        """Run a blocking movement and return the measured state change.

        The rover must be stationary when called. The gyro bias is estimated over
        stationary_s beforehand; sampling continues for settle_s afterward.

        Args:
            movement: The blocking movement, called with no arguments.
            stationary_s: Stationary period before the movement, in seconds.
            settle_s: Period after the movement, in seconds.
            sample_hz: Target sample rate.

        Raises:
            RuntimeError: If too few samples were obtained. The movement's own
                exceptions propagate unchanged, with sampling stopped.
        """
        samples : list[LSM9DS1Data] = []
        errors = [0]
        finished = threading.Event()
        period_s = 1.0 / sample_hz

        def sample_loop():
            next_sample = time.monotonic()
            while not finished.is_set():
                try:
                    samples.append(self.read())
                except RuntimeError:
                    errors[0] = errors[0] + 1
                next_sample += period_s
                finished.wait(max(0.0, next_sample - time.monotonic()))

        worker = threading.Thread(target=sample_loop, name="imu-measure", daemon=True)
        worker.start()
        try:
            time.sleep(stationary_s)
            start_index = len(samples)
            movement()
            end_index = len(samples)
            time.sleep(settle_s)
        finally:
            finished.set()
            worker.join()

        return self._summarize(samples, start_index, end_index, errors[0])

    @staticmethod
    def _summarize(samples: list[LSM9DS1Data], start_index: int, end_index: int, read_errors: int) -> StateChange:
        """Compute the state change from the stationary, movement and settle samples."""
        before = samples[:start_index]
        after = samples[end_index:]
        tracked = samples[max(start_index - 1, 0):]
        if not before or not after or len(tracked) < 2:
            raise RuntimeError(f"[IMUWRAPPER] Too few samples: before={len(before)}, after={len(after)}, total={len(samples)}")

        bias = _mean([s.angular_velocity for s in before])
        start_acceleration = _mean([s.linear_acceleration for s in before])
        end_acceleration = _mean([s.linear_acceleration for s in after])

        rotation = [0.0, 0.0, 0.0]
        peak_rate = [0.0, 0.0, 0.0]
        peak_acceleration = 0.0
        largest_gap_s = 0.0
        for previous, current in zip(tracked, tracked[1:]):
            dt = (current.header.stamp.to_nanoseconds() - previous.header.stamp.to_nanoseconds()) / 1e9
            largest_gap_s = max(largest_gap_s, dt)
            for axis, name in enumerate(("x", "y", "z")):
                rate_previous = getattr(previous.angular_velocity, name) - getattr(bias, name)
                rate_current = getattr(current.angular_velocity, name) - getattr(bias, name)
                rotation[axis] += 0.5 * (rate_previous + rate_current) * dt
                peak_rate[axis] = max(peak_rate[axis], abs(rate_current))
            peak_acceleration = max(peak_acceleration, math.hypot(
                current.linear_acceleration.x - start_acceleration.x,
                current.linear_acceleration.y - start_acceleration.y,
                current.linear_acceleration.z - start_acceleration.z))

        return StateChange(
            duration_s=(tracked[-1].header.stamp.to_nanoseconds() - tracked[0].header.stamp.to_nanoseconds()) / 1e9,
            sample_count=len(tracked),
            read_errors=read_errors,
            largest_gap_s=largest_gap_s,
            gyro_bias_rad_s=bias,
            delta_rotation_rad=Vector3(*rotation),
            peak_rate_rad_s=Vector3(*peak_rate),
            start_acceleration=start_acceleration,
            end_acceleration=end_acceleration,
            tilt_change_rad=_tilt_between(start_acceleration, end_acceleration),
            peak_acceleration_change=peak_acceleration,
            samples=tuple(samples),
        )
