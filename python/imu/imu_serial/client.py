"""Synchronous IMU protocol v1 client. One reader/command caller at a time."""
from collections import deque
from dataclasses import dataclass
import math
import re
import time

from .transport import LineTransport, ProtocolError


@dataclass(frozen=True)
class ImuSample:
    request_id: int
    sequence: int
    timestamp_us: int
    acceleration_m_s2: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    magnetic_field_ut: tuple[float, float, float]
    temperature_c: float


@dataclass(frozen=True)
class Ack:
    request_id: int
    action: str


class DeviceError(Exception):
    """An ERR record from the ESP32; request_id=0 is asynchronous."""

    def __init__(self, request_id: int, code: str):
        self.request_id = request_id
        self.code = code
        super().__init__(f"IMU error {code} (request {request_id})")


def _uint(text: str, maximum: int) -> int:
    if not re.fullmatch(r"[0-9]+", text):
        raise ProtocolError("Expected an unsigned decimal integer")
    try:
        value = int(text)
    except ValueError as exc:
        raise ProtocolError("Integer field is too long") from exc
    if value > maximum:
        raise ProtocolError("Integer field out of range")
    return value


def parse_line(line: str) -> ImuSample | Ack | DeviceError | None:
    """Parse one ASCII record. Unknown boot/log lines return None."""
    fields = line.removesuffix('\n').removesuffix('\r').split(',')
    kind = fields[0]
    if kind not in ("IMU", "ACK", "ERR"):
        return None
    if not line.isascii():
        raise ProtocolError("Non-ASCII protocol record")
    if kind == "IMU":
        if len(fields) != 15 or fields[1] != "1":
            raise ProtocolError("Expected 15 fields and IMU protocol version 1")
        request_id = _uint(fields[2], 2**32 - 1)
        sequence = _uint(fields[3], 2**64 - 1)
        timestamp = _uint(fields[4], 2**63 - 1)
        try:
            values = tuple(float(value) for value in fields[5:])
        except ValueError as exc:
            raise ProtocolError("Invalid IMU measurement") from exc
        if not all(math.isfinite(value) for value in values):
            raise ProtocolError("Non-finite IMU measurement")
        return ImuSample(request_id, sequence, timestamp, values[0:3],
                         values[3:6], values[6:9], values[9])
    if len(fields) != 3:
        raise ProtocolError(f"Expected three {kind} fields")
    request_id = _uint(fields[1], 2**32 - 1)
    if kind == "ACK":
        if request_id == 0 or fields[2] not in ("START", "STOP", "INIT"):
            raise ProtocolError("Invalid acknowledgment")
        return Ack(request_id, fields[2])
    if not re.fullmatch(r"[A-Z0-9_]+", fields[2]):
        raise ProtocolError("Invalid error code")
    return DeviceError(request_id, fields[2])


class IMUClient:
    """Owns its serial transport; close explicitly or use a context manager.

    Not thread-safe. Timeouts cover writing a command and awaiting its response.
    Stream samples encountered during commands enter a bounded queue. On overflow,
    the oldest sample is discarded and dropped_samples increases. INIT permits
    recovery past earlier asynchronous errors, saved in last_device_error.
    """

    def __init__(self, port=None, *, baudrate=115200, timeout=1.0,
                 transport=None, queue_size=64):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        if not isinstance(queue_size, int) or queue_size <= 0:
            raise ValueError("queue_size must be a positive integer")
        if transport is None:
            import serial
            transport = serial.Serial(port, baudrate, timeout=timeout, write_timeout=timeout)
        self._io = LineTransport(transport)
        self.timeout = timeout
        self._next_id = 1
        self._samples = deque(maxlen=queue_size)
        self.dropped_samples = 0
        self.last_device_error = None

    def __enter__(self):
        if self._io.closed:
            raise OSError("IMU connection is closed")
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self._io.close()
        self._samples.clear()

    def _record(self, deadline):
        while True:
            line = self._io.read_line(deadline)
            # Ignore unrecognized boot text, including undecodable noise.
            if line.split(b',', 1)[0] not in (b'IMU', b'ACK', b'ERR'):
                continue
            try:
                record = parse_line(line.decode('ascii'))
            except UnicodeDecodeError as exc:
                raise ProtocolError("Non-ASCII protocol record") from exc
            if record is not None:
                return record

    def _queue(self, sample):
        if len(self._samples) == self._samples.maxlen:
            self.dropped_samples += 1
        self._samples.append(sample)

    def command(self, action: str) -> Ack | ImuSample:
        """Send START, STOP, INIT or READ and wait for its correlated response."""
        if action not in ("START", "STOP", "INIT", "READ"):
            raise ValueError("action must be START, STOP, INIT or READ")
        if self._next_id > 2**32 - 1:
            raise OverflowError("Request IDs exhausted; open a new connection")
        request_id = self._next_id
        self._next_id += 1
        deadline = time.monotonic() + self.timeout
        self._io.write_all(f"CMD,{request_id},{action}\n".encode('ascii'), deadline)
        while True:
            record = self._record(deadline)
            if isinstance(record, DeviceError):
                if record.request_id == 0:
                    self.last_device_error = record
                    if action == "INIT":
                        continue
                    raise record
                if record.request_id == request_id:
                    raise record
            elif record.request_id == request_id:
                if action == "READ" and isinstance(record, ImuSample):
                    return record
                if isinstance(record, Ack) and record.action == action:
                    if action in ("STOP", "INIT"):
                        self._samples.clear()
                    return record
                raise ProtocolError("Response type/action does not match command")
            elif isinstance(record, ImuSample) and record.request_id == 0:
                self._queue(record)
            # Responses to earlier timed-out commands are deliberately ignored.

    def start(self) -> Ack:
        return self.command("START")

    def stop(self) -> Ack:
        return self.command("STOP")

    def reinitialize(self) -> Ack:
        """Reinitialize the IMU and leave streaming stopped. Call start to resume."""
        return self.command("INIT")

    def read_once(self) -> ImuSample:
        """Acquire now; DeviceError(ESP_ERR_NOT_FINISHED) means no fresh sample."""
        return self.command("READ")

    def read_sample(self) -> ImuSample:
        """Return the next queued or streaming sample, with a finite timeout."""
        if self._io.closed:
            raise OSError("IMU connection is closed")
        if self._samples:
            return self._samples.popleft()
        deadline = time.monotonic() + self.timeout
        while True:
            record = self._record(deadline)
            if isinstance(record, ImuSample) and record.request_id == 0:
                return record
            if isinstance(record, DeviceError) and record.request_id == 0:
                self.last_device_error = record
                raise record
