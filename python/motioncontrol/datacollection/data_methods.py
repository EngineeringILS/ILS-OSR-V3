"""CircuitPython-backed sensor collectors."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional, Protocol, Tuple

try:
    from .data_types import Header, LSM9DS1Data, Time, Vector3
except ImportError:  # Allow direct execution from this directory.
    from data_types import Header, LSM9DS1Data, Time, Vector3


GAUSS_TO_TESLA = 1e-4


class LSM9DS1Sensor(Protocol):
    """The subset of the CircuitPython LSM9DS1 API used by the collector."""

    @property
    def acceleration(self) -> Tuple[float, float, float]: ...

    @property
    def gyro(self) -> Tuple[float, float, float]: ...

    @property
    def magnetic(self) -> Tuple[float, float, float]: ...

    @property
    def temperature(self) -> float: ...


class LSM9DS1Collector:
    """Continuously sample an LSM9DS1 and publish its latest complete sample.

    The worker thread owns the sensor. Consumers call :meth:`latest`, which
    never performs I²C work and therefore remains suitable for a drive loop.
    Samples are immutable, so returning the stored object is safe.
    """

    def __init__(
        self,
        sensor: LSM9DS1Sensor,
        sample_hz: float = 20.0,
        frame_id: str = "imu_link",
        *,
        clock_ns: Callable[[], int] = time.time_ns,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if sample_hz <= 0:
            raise ValueError("sample_hz must be greater than zero")

        self._sensor = sensor
        self._period_ns = int(1_000_000_000 / sample_hz)
        self._frame_id = frame_id
        self._clock_ns = clock_ns
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest: Optional[LSM9DS1Data] = None
        self._latest_monotonic_ns: Optional[int] = None
        self._last_error: Optional[Exception] = None
        self._sequence = 0

    def collect_once(self) -> LSM9DS1Data:
        """Read one complete sample synchronously and publish it."""

        acceleration = Vector3.from_iterable(self._sensor.acceleration)
        angular_velocity = Vector3.from_iterable(self._sensor.gyro)
        magnetic_gauss = Vector3.from_iterable(self._sensor.magnetic)
        magnetic_tesla = Vector3(
            magnetic_gauss.x * GAUSS_TO_TESLA,
            magnetic_gauss.y * GAUSS_TO_TESLA,
            magnetic_gauss.z * GAUSS_TO_TESLA,
        )
        temperature = float(self._sensor.temperature)
        timestamp_ns = self._clock_ns()
        sampled_monotonic_ns = self._monotonic_ns()

        sample = LSM9DS1Data(
            header=Header(
                stamp=Time.from_nanoseconds(timestamp_ns),
                frame_id=self._frame_id,
            ),
            linear_acceleration=acceleration,
            angular_velocity=angular_velocity,
            magnetic_field=magnetic_tesla,
            temperature=temperature,
            sequence=self._sequence,
        )

        with self._lock:
            self._latest = sample
            self._latest_monotonic_ns = sampled_monotonic_ns
            self._last_error = None
            self._sequence += 1
        return sample

    def latest(self, max_age_seconds: Optional[float] = None) -> Optional[LSM9DS1Data]:
        """Return the newest sample, or ``None`` when absent or stale."""

        if max_age_seconds is not None and max_age_seconds < 0:
            raise ValueError("max_age_seconds cannot be negative")

        with self._lock:
            sample = self._latest
            sampled_ns = self._latest_monotonic_ns

        if sample is None or sampled_ns is None:
            return None
        if max_age_seconds is not None:
            max_age_ns = int(max_age_seconds * 1_000_000_000)
            if self._monotonic_ns() - sampled_ns > max_age_ns:
                return None
        return sample

    @property
    def last_error(self) -> Optional[Exception]:
        """Return the last sampling error without clearing it."""

        with self._lock:
            return self._last_error

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Start sampling in a daemon thread; repeated calls are harmless."""

        if self.is_running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="lsm9ds1-collector",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        """Request shutdown and wait briefly for the collector to exit."""

        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive():
                raise TimeoutError("LSM9DS1 collector did not stop in time")
        self._thread = None

    def _run(self) -> None:
        next_sample_ns = self._monotonic_ns()
        while not self._stop_event.is_set():
            try:
                self.collect_once()
            except Exception as error:  # Keep driving even if telemetry fails.
                with self._lock:
                    self._last_error = error

            next_sample_ns += self._period_ns
            remaining_ns = next_sample_ns - self._monotonic_ns()
            if remaining_ns <= 0:
                next_sample_ns = self._monotonic_ns()
                continue
            self._stop_event.wait(remaining_ns / 1_000_000_000)

    def __enter__(self) -> "LSM9DS1Collector":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.stop()


def create_lsm9ds1_i2c(
    i2c: object = None,
    *,
    mag_address: int = 0x1E,
    xg_address: int = 0x6B,
) -> LSM9DS1Sensor:
    """Create the CircuitPython I²C sensor, importing hardware modules lazily."""

    import adafruit_lsm9ds1

    if i2c is None:
        import board

        i2c = board.I2C()
    return adafruit_lsm9ds1.LSM9DS1_I2C(
        i2c,
        mag_address=mag_address,
        xg_address=xg_address,
    )
