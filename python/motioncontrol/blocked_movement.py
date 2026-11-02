"""
blocked_movement.py
Blocking movements built on RoverDrive.
Each call returns once the movement has ended and the motors are stopped.
"""

import time
from typing import Any, Optional

from datacollection.imu_wrapper import StateChange
from new_motion_methods import (
    CONTROL_LOOP_INTERVAL_SECONDS,
    MovementState,
    RotaryState,
    RoverDrive,
    SteerInput,
)

# Motor and servo indices, as ordered in main.py.
LEFT_MOTORS = [0, 2, 4]
RIGHT_MOTORS = [1, 3, 5]
INPLACE_STEER = {
    "i_front_r": 1, "sign_fr": -1,
    "i_front_l": 2, "sign_fl": 1,
    "i_back_r": 0, "sign_br": 1,
    "i_back_l": 3, "sign_bl": -1,
}

# Estimated servo travel time. To be confirmed on the bench.
SERVO_SETTLE_SECONDS = 0.5
# Connection check period; approximately 0.5 s at the loop interval.
CHECK_INTERVAL_TICKS = 50


class BlockedMovement:
    """Timed movements that hold the caller until complete.

    The control loop runs inside each call. Motors are stopped on completion,
    on any exception, and on KeyboardInterrupt. When given an IMUWrapper, a call
    samples the IMU throughout and returns the StateChange; otherwise it returns None.
    """

    @staticmethod
    def moveforward(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Drive forward with straight wheels.

        Args:
            rover: The drive to command.
            speed: Positive speed magnitude in QPPS.
            duration_s: Movement duration in seconds.
            imu: Optional IMUWrapper to sample during the movement.

        Returns:
            The StateChange when imu is given, otherwise None.
        """
        return BlockedMovement._execute(imu, rover, BlockedMovement._linear, rover, speed, duration_s, MovementState.FORWARD)

    @staticmethod
    def movebackward(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Drive backward with straight wheels. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, BlockedMovement._linear, rover, speed, duration_s, MovementState.BACKWARD)

    @staticmethod
    def rotateleft(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Rotate left in place, then return the wheels to straight. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, BlockedMovement._rotate, rover, speed, duration_s, RotaryState.TURN_LEFT)

    @staticmethod
    def rotateright(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Rotate right in place, then return the wheels to straight. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, BlockedMovement._rotate, rover, speed, duration_s, RotaryState.TURN_RIGHT)

    @staticmethod
    def _execute(imu: Any, rover: RoverDrive, movement, *args) -> Optional[StateChange]:
        """Run the movement, under IMU measurement when an IMU is given."""
        if imu is None:
            movement(*args)
            return None
        # The gyro bias estimate requires a stationary rover.
        rover.set_state()
        rover.stop()
        return imu.measure(movement, *args)

    @staticmethod
    def _linear(rover: RoverDrive, speed: int, duration_s: float, movement_state: MovementState):
        """Straighten the wheels and drive in the given direction."""
        BlockedMovement._prepare(rover, speed, duration_s)
        rover.set_state(movement_state, RotaryState.NOT_TURN)
        BlockedMovement._run(rover, duration_s, lambda: rover.linear_motion(speeds=[speed]))

    @staticmethod
    def _rotate(rover: RoverDrive, speed: int, duration_s: float, rotary_state: RotaryState):
        """Steer for an in-place turn, rotate, and straighten the wheels."""
        BlockedMovement._prepare(rover, speed, duration_s)
        rover.set_inplace_steer(**INPLACE_STEER)
        time.sleep(SERVO_SETTLE_SECONDS)
        rover.set_state(MovementState.TURN_INPLACE, rotary_state)
        BlockedMovement._run(rover, duration_s, lambda: rover.rotary_motion(
            speeds=[speed], left_motors=LEFT_MOTORS, right_motors=RIGHT_MOTORS))
        rover.steer_servos(SteerInput.STRAIGHT)

    @staticmethod
    def _prepare(rover: RoverDrive, speed: int, duration_s: float):
        """Validate the request, stop the motors and straighten the wheels.

        Raises:
            ValueError: If the speed or duration is invalid. Nothing has moved.
            RuntimeError: If a fault is latched.
        """
        if speed < 0 or speed > rover.safe_maximum_speed:
            raise ValueError(f"[BLOCKEDMOVEMENT] Unsafe Speed Entered: {speed}")
        if duration_s < 0:
            raise ValueError(f"[BLOCKEDMOVEMENT] Invalid Duration: {duration_s}")
        if rover.error:
            raise RuntimeError(f"[BLOCKEDMOVEMENT] Fault latched: {rover.error_message}")

        rover.set_state()
        rover.stop()
        rover.steer_servos(SteerInput.STRAIGHT)
        time.sleep(SERVO_SETTLE_SECONDS)

    @staticmethod
    def _run(rover: RoverDrive, duration_s: float, motion):
        """Call the motion method each tick until the deadline, then stop the motors.

        Raises:
            RuntimeError: If the rover refuses motion, or a connection check fails.
        """
        ticks = 0
        next_tick = time.monotonic()
        deadline = next_tick + duration_s
        try:
            while time.monotonic() < deadline:
                # One tick permitted; refilled while this loop runs.
                rover.stop_counter = 1
                if motion().status == "stopped":
                    raise RuntimeError(f"[BLOCKEDMOVEMENT] Motion refused: {rover.error_message}")

                ticks = ticks + 1
                if ticks >= CHECK_INTERVAL_TICKS:
                    ticks = 0
                    rover.check_connections()

                next_tick += CONTROL_LOOP_INTERVAL_SECONDS
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            rover.set_state()
            rover.stop()
