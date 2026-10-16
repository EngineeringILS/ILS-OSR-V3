"""Asynchronous CSV logging for synchronized motion and IMU telemetry."""

from __future__ import annotations

import csv
import queue
import re
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Sequence, TextIO, Tuple

from .data_types import (
    Header,
    LSM9DS1Data,
    MotionTelemetryData,
    MotorVelocityData,
    Time,
)


def _column_name(name: str) -> str:
    value = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
    return value or "unnamed"


class MotionCSVLogger:
    """Write motion telemetry on a worker thread without blocking control."""

    def __init__(
        self,
        path: Path,
        motors: Sequence[object],
        *,
        queue_size: int = 2048,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("queue_size must be greater than zero")
        self.path = Path(path)
        self._motor_schema: Tuple[Tuple[int, str], ...] = tuple(
            (int(getattr(motor, "id")), str(getattr(motor, "name")))
            for motor in motors
        )
        self._queue: queue.Queue[MotionTelemetryData] = queue.Queue(queue_size)
        self._clock_ns = clock_ns
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._file: Optional[TextIO] = None
        self._writer: Optional[object] = None
        self._state_lock = threading.Lock()
        self._last_error: Optional[Exception] = None
        self._dropped_records = 0

    @property
    def dropped_records(self) -> int:
        with self._state_lock:
            return self._dropped_records

    @property
    def last_error(self) -> Optional[Exception]:
        with self._state_lock:
            return self._last_error

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._csv_header())
        self._file.flush()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="motion-csv-logger",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        imu: LSM9DS1Data,
        *,
        command_timestamp_ns: int,
        input_command: str,
        drive_mode: str,
        selected_speed_qpps: int,
        watchdog_ticks_remaining: int,
        inplace_turn: bool,
        motors: Sequence[object],
    ) -> bool:
        """Enqueue one joined snapshot; return false rather than blocking if full."""

        motor_data = tuple(
            MotorVelocityData(
                motor_id=int(getattr(motor, "id")),
                name=str(getattr(motor, "name")),
                commanded_qpps=int(getattr(motor, "speed")),
                readback_qpps=int(getattr(motor, "actual_speed")),
            )
            for motor in motors
        )
        if tuple((motor.motor_id, motor.name) for motor in motor_data) != self._motor_schema:
            raise ValueError("motor order or identity does not match logger configuration")

        record = MotionTelemetryData(
            header=Header(stamp=Time.from_nanoseconds(self._clock_ns()), frame_id="base_link"),
            command_stamp=Time.from_nanoseconds(command_timestamp_ns),
            input_command=input_command,
            drive_mode=drive_mode,
            selected_speed_qpps=selected_speed_qpps,
            watchdog_ticks_remaining=watchdog_ticks_remaining,
            inplace_turn=inplace_turn,
            imu=imu,
            motors=motor_data,
        )
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            with self._state_lock:
                self._dropped_records += 1
            return False
        return True

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive():
                raise TimeoutError("motion CSV logger did not stop in time")
        self._thread = None
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None

    def _run(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                record = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                if self._writer is None:
                    raise RuntimeError("logger is not started")
                self._writer.writerow(self._csv_row(record))
            except Exception as error:
                with self._state_lock:
                    self._last_error = error
                self._stop_event.set()
                return
            finally:
                self._queue.task_done()

    def _csv_header(self) -> list[str]:
        columns = [
            "log_sec", "log_nanosec", "command_sec", "command_nanosec",
            "input_command", "drive_mode", "selected_speed_qpps",
            "watchdog_ticks_remaining", "inplace_turn",
            "imu_sec", "imu_nanosec", "imu_frame_id", "imu_sequence",
            "ax_mps2", "ay_mps2", "az_mps2",
            "gx_radps", "gy_radps", "gz_radps",
            "mx_tesla", "my_tesla", "mz_tesla", "temperature_c",
        ]
        for motor_id, name in self._motor_schema:
            prefix = f"motor_{motor_id}_{_column_name(name)}"
            columns.extend((f"{prefix}_command_qpps", f"{prefix}_readback_qpps"))
        return columns

    @staticmethod
    def _csv_row(record: MotionTelemetryData) -> list[object]:
        imu = record.imu
        values: list[object] = [
            record.header.stamp.sec,
            record.header.stamp.nanosec,
            record.command_stamp.sec,
            record.command_stamp.nanosec,
            record.input_command,
            record.drive_mode,
            record.selected_speed_qpps,
            record.watchdog_ticks_remaining,
            record.inplace_turn,
            imu.header.stamp.sec,
            imu.header.stamp.nanosec,
            imu.header.frame_id,
            imu.sequence,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.magnetic_field.x,
            imu.magnetic_field.y,
            imu.magnetic_field.z,
            imu.temperature,
        ]
        for motor in record.motors:
            values.extend((motor.commanded_qpps, motor.readback_qpps))
        return values

    def __enter__(self) -> "MotionCSVLogger":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.stop()
