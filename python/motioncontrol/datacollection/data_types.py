"""ROS2-style data containers used by the Python sensor collectors."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Tuple


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class Time:
    """Equivalent to ``builtin_interfaces/msg/Time``."""

    sec: int = 0
    nanosec: int = 0

    def __post_init__(self) -> None:
        if not 0 <= self.nanosec < NANOSECONDS_PER_SECOND:
            raise ValueError("nanosec must be in the range [0, 1_000_000_000)")

    @classmethod
    def from_nanoseconds(cls, timestamp_ns: int) -> "Time":
        seconds, nanoseconds = divmod(timestamp_ns, NANOSECONDS_PER_SECOND)
        return cls(sec=seconds, nanosec=nanoseconds)

    def to_nanoseconds(self) -> int:
        return self.sec * NANOSECONDS_PER_SECOND + self.nanosec


@dataclass(frozen=True, slots=True)
class Header:
    """Equivalent to ``std_msgs/msg/Header``."""

    stamp: Time = field(default_factory=Time)
    frame_id: str = ""


@dataclass(frozen=True, slots=True)
class Vector3:
    """Equivalent to ``geometry_msgs/msg/Vector3``."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_iterable(cls, values: Iterable[float]) -> "Vector3":
        x, y, z = values
        return cls(float(x), float(y), float(z))


@dataclass(frozen=True, slots=True)
class LSM9DS1Data:
    """One complete IMU sample with ROS-compatible field conventions.

    Units match ROS sensor messages: acceleration is m/s², angular velocity
    is rad/s, magnetic field is tesla, and temperature is degrees Celsius.
    The sequence number is local process metadata and is not part of a ROS
    header; it lets in-process consumers detect whether a sample is new.
    """

    header: Header = field(default_factory=Header)
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    magnetic_field: Vector3 = field(default_factory=Vector3)
    temperature: float = 0.0
    sequence: int = 0

    @staticmethod
    def csv_header() -> str:
        return (
            "sec,nanosec,frame_id,sequence,"
            "ax_mps2,ay_mps2,az_mps2,"
            "gx_radps,gy_radps,gz_radps,"
            "mx_tesla,my_tesla,mz_tesla,temperature_c"
        )

    def to_csv_row(self) -> str:
        values = (
            self.header.stamp.sec,
            self.header.stamp.nanosec,
            self.header.frame_id,
            self.sequence,
            self.linear_acceleration.x,
            self.linear_acceleration.y,
            self.linear_acceleration.z,
            self.angular_velocity.x,
            self.angular_velocity.y,
            self.angular_velocity.z,
            self.magnetic_field.x,
            self.magnetic_field.y,
            self.magnetic_field.z,
            self.temperature,
        )
        return ",".join(str(value) for value in values)


@dataclass(frozen=True, slots=True)
class MotorVelocityData:
    """Commanded and last-read encoder velocity for one motor, in QPPS."""

    motor_id: int
    name: str
    commanded_qpps: int
    readback_qpps: int


@dataclass(frozen=True, slots=True)
class MotionTelemetryData:
    """A timestamped IMU sample joined with the current drive state."""

    header: Header
    command_stamp: Time
    input_command: str
    drive_mode: str
    selected_speed_qpps: int
    watchdog_ticks_remaining: int
    inplace_turn: bool
    imu: LSM9DS1Data
    motors: Tuple[MotorVelocityData, ...]
