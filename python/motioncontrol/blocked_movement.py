"""
blocked_movement.py
Blocking movements built on RoverDrive.
Each call returns once the movement has ended and the motors are stopped.

Call flow:
    moveforward, movebackward -> _execute -> _linear -> _prepare, _run
    rotateleft, rotateright   -> _execute -> _rotate -> _prepare, _run
"""

import time
from typing import Any, Callable, Optional

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

    The public methods select the direction only. _execute applies the optional
    measurement, _linear and _rotate sequence the movement, _prepare makes the
    rover ready, and _run holds the timed control loop.
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
        return BlockedMovement._execute(imu, rover, lambda: BlockedMovement._linear(rover, speed, duration_s, MovementState.FORWARD))

    @staticmethod
    def movebackward(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Drive backward with straight wheels. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, lambda: BlockedMovement._linear(rover, speed, duration_s, MovementState.BACKWARD))

    @staticmethod
    def rotateleft(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Rotate left in place, then return the wheels to straight. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, lambda: BlockedMovement._rotate(rover, speed, duration_s, RotaryState.TURN_LEFT))

    @staticmethod
    def rotateright(rover: RoverDrive, speed: int, duration_s: float, imu: Any = None) -> Optional[StateChange]:
        """Rotate right in place, then return the wheels to straight. Arguments are those of moveforward()."""
        return BlockedMovement._execute(imu, rover, lambda: BlockedMovement._rotate(rover, speed, duration_s, RotaryState.TURN_RIGHT))

    @staticmethod
    def _execute(imu: Any, rover: RoverDrive, movement: Callable[[], None]) -> Optional[StateChange]:
        """Run the movement, under IMU measurement when an IMU is given.

        Flow with an IMU: stop the motors, then imu.measure() samples while stationary,
        runs the movement, and samples while settling. The measured movement includes
        the servo moves and settling time of _linear and _rotate.

        Args:
            imu: Optional IMUWrapper.
            rover: The drive to stop before measurement.
            movement: The movement, called with no arguments.

        Returns:
            The StateChange when imu is given, otherwise None.
        """
        if imu is None:
            movement()
            return None
        # The gyro bias estimate requires a stationary rover.
        rover.set_state()
        rover.stop()
        return imu.measure(movement)

    @staticmethod
    def _linear(rover: RoverDrive, speed: int, duration_s: float, movement_state: MovementState):
        """Straighten the wheels and drive in the given direction.

        Flow: prepare the rover, set the movement state, then run linear_motion each
        tick until the deadline. The wheels remain straight on return.
        """
        BlockedMovement._prepare(rover, speed, duration_s)
        # linear_motion applies the sign for the direction from this state.
        rover.set_state(movement_state, RotaryState.NOT_TURN)
        BlockedMovement._run(rover, duration_s, lambda: rover.linear_motion(speeds=[speed]))

    @staticmethod
    def _rotate(rover: RoverDrive, speed: int, duration_s: float, rotary_state: RotaryState):
        """Steer for an in-place turn, rotate, and straighten the wheels.

        Flow: prepare the rover, steer to the in-place position and wait for the servos,
        set the rotary state, run rotary_motion each tick until the deadline, then
        straighten. If _run raises, the motors are stopped but the wheels stay steered.
        """
        BlockedMovement._prepare(rover, speed, duration_s)
        rover.set_inplace_steer(**INPLACE_STEER)
        # The wheels must arrive before the motors turn.
        time.sleep(SERVO_SETTLE_SECONDS)
        rover.set_state(MovementState.TURN_INPLACE, rotary_state)
        BlockedMovement._run(rover, duration_s, lambda: rover.rotary_motion(
            speeds=[speed], left_motors=LEFT_MOTORS, right_motors=RIGHT_MOTORS))
        rover.steer_servos(SteerInput.STRAIGHT)

    @staticmethod
    def _prepare(rover: RoverDrive, speed: int, duration_s: float):
        """Validate the request, stop the motors and straighten the wheels.

        Flow: reject an invalid request before any hardware command, clear the movement
        state, stop the motors, then straighten and wait for the servos.

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

        Flow per tick: refill the stop counter, call the motion method, check the
        connections every CHECK_INTERVAL_TICKS, then wait for the next tick. Tick times
        are absolute, so a late tick does not extend the schedule. The motors are
        stopped however the loop ends.

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
                # A stopped result means the rover refused the motion, so the loop ends.
                if motion().status == "stopped":
                    raise RuntimeError(f"[BLOCKEDMOVEMENT] Motion refused: {rover.error_message}")

                # Periodic connection check.
                ticks = ticks + 1
                if ticks >= CHECK_INTERVAL_TICKS:
                    ticks = 0
                    rover.check_connections()

                # Wait until the next scheduled tick; no wait if already late.
                next_tick += CONTROL_LOOP_INTERVAL_SECONDS
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            # Reached on completion, on any exception and on KeyboardInterrupt.
            rover.set_state()
            rover.stop()
