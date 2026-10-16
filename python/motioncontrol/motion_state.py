"""State, hardware references, and operator I/O for the motion loop."""

import os
import select
import sys
from dataclasses import dataclass, field
from typing import Any


@dataclass
class MotionState:
    """Mutable control state; selected speed is distinct from motor commands."""

    selected_speed_qpps: int = 0
    speed_index: int = 0
    watchdog_ticks: int = 0
    watchdog_stopped: bool = False
    inplace_turn: bool = False
    drive_mode: str = "stopped"
    command_timestamp_ns: int = 0
    input_command: str = "startup"
    last_imu_sequence: int = -1
    exit_requested: bool = False
    logging_enabled: bool = True
    queue_warning_sent: bool = False


@dataclass
class MotionHardware:
    """Configured devices in the existing servo and motor mapping order."""

    servodriver: Any
    servos: list
    configs: list
    roboclaws: list
    motors: list
    step_degrees: int
    stop_counts: int
    speeds: list[int] = field(default_factory=lambda: [0])


class MotionTerminal:
    """Non-blocking byte input and single-line output shorter than 80 chars."""

    def __init__(self, input_stream=None, output_stream=None):
        self.input = sys.stdin if input_stream is None else input_stream
        self.output = sys.stdout if output_stream is None else output_stream

    def fileno(self):
        return self.input.fileno()

    def poll(self):
        """Return one key, None when idle, or an empty string on disconnect."""
        fd = self.fileno()
        if not select.select([fd], [], [], 0)[0]:
            return None
        return os.read(fd, 1).decode("ascii", errors="replace").lower()

    def report(self, message):
        """Sanitize and cap visible output; broken output must not abort control."""
        line = "".join(char if char.isprintable() else " " for char in str(message))
        line = line if len(line) < 80 else line[:76] + "..."
        try:
            self.output.write(line + "\r\n")
            self.output.flush()
        except (OSError, ValueError):
            pass
