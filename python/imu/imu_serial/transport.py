"""Bounded line framing over a pyserial-compatible transport."""
import time


class ProtocolError(ValueError):
    """Malformed protocol record or oversized line."""


class LineTransport:
    MAX_LINE = 512

    def __init__(self, serial_like):
        self.serial = serial_like
        self._buffer = bytearray()
        self._discard = False
        self.closed = False

    def close(self):
        if not self.closed:
            self.closed = True
            self.serial.close()

    def _remaining(self, deadline):
        if self.closed:
            raise OSError("IMU connection is closed")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("IMU operation timed out")
        return remaining

    def read_line(self, deadline):
        while True:
            remaining = self._remaining(deadline)
            previous = self.serial.timeout
            try:
                self.serial.timeout = remaining
                char = self.serial.read(1)
            finally:
                self.serial.timeout = previous
            if not char:
                # Real serial reads block; injected/nonblocking transports may not.
                time.sleep(min(0.001, max(0, deadline - time.monotonic())))
                continue
            if self._discard:
                if char == b'\n':
                    self._discard = False
                continue
            if char == b'\n':
                line = bytes(self._buffer)
                self._buffer.clear()
                return line.removesuffix(b'\r')
            if len(self._buffer) >= self.MAX_LINE:
                self._buffer.clear()
                self._discard = True
                raise ProtocolError("IMU line exceeds 512 bytes")
            self._buffer.extend(char)

    def write_all(self, data, deadline):
        try:
            offset = 0
            while offset < len(data):
                remaining = self._remaining(deadline)
                previous = self.serial.write_timeout
                try:
                    self.serial.write_timeout = remaining
                    count = self.serial.write(data[offset:])
                finally:
                    self.serial.write_timeout = previous
                if not isinstance(count, int) or count <= 0 or count > len(data) - offset:
                    raise OSError("Invalid or zero-length serial write")
                offset += count
        except Exception:
            # A partial command must never be concatenated with the next command.
            self.close()
            raise
