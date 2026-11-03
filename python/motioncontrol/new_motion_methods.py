"""
new_motion_methods.py
RoverDrive: setup and movement methods for the Roboclaw MCU + PCA9685 Servo Driver.
"""

''' System Imports: '''
import csv
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
from pathlib import Path
from typing import Any

''' Data Collection Imports: '''
from datacollection.motion_logging import _column_name

''' Servo Method Imports: '''
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

import servocontrol.servo_methods as servo_methods
from   servocontrol.servo_methods import ServoConfig

'''Motor Method Imports: '''
import motorcontrol.roboclaw_methods as roboclaw_methods
from motorcontrol.roboclaw_types import RoboclawUnit, Motor
from typing import Tuple

#TODO migrate to motion_types when convenient:
# Encodes the general state of how the Rover is moving. 
# Generally, turn_right, and turn_left mean turn inplace as a state.
class MovementState(IntEnum):
    STOPPED = 0
    FORWARD = 1
    BACKWARD = 2
    TURN_INPLACE = 3

# Encodes the specific turn in place rotary state 
class RotaryState(IntEnum):
    STOPPED = 0
    NOT_TURN = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3

# Input for steering front or back
class SteerInput(IntEnum):
    STRAIGHT = 0
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4

    
CONTROL_LOOP_INTERVAL_SECONDS = 0.01


# TODO: This class ONLY NEEDS TO BE MOVEMENT COMPLETE, DO NOT ADD ADDITIONAL METHODS WITHOUT ASKING
# TODO: Modifications should not rewrite existing code, unless for a safety critical, or bugfixing reason.
# TODO: Modifications ought to be minimal, not changing the underlying structure/syntax/style.
class RoverDrive:
    """Sets up the drive hardware and exposes movement methods.

    State belongs to the implementation, which calls set_state() and sets stop_counter.
    Each movement method records its result as a Command in the log file and returns it.
    """
    roboclaws: list[RoboclawUnit]
    motors: list[Motor]
    safe_maximum_speed: int
    servodriver: ServoKit
    servos: list[Servo]
    configs : list[ServoConfig]
    error: bool = False
    error_message: str = ""
    stop_counter : int = 0
    movement_state : MovementState
    rotary_state : RotaryState
    debug : bool = False
    log_file : Any = None
    log_writer : Any = None

    @dataclass(frozen=True, slots=True)
    class Command:
        """Record of one movement method call and its result."""
        action : str
        status : str
        message : str
        speeds : list[int]
        actual_speeds : list[int]
        movement_state : MovementState
        rotary_state : RotaryState
        stop_counter : int
        timestamp_ns : int = field(default_factory=time.time_ns)

    def __init__(self, safe_maximum_speed: int = 100):
        """Set the speed limit. Movement is refused until bringup() succeeds.

        Args:
            safe_maximum_speed: Maximum speed magnitude in QPPS.
        """
        self.safe_maximum_speed = safe_maximum_speed
        self.roboclaws = []
        self.motors = []
        self.servos = []
        self.configs = []
        self.error = True
        self.error_message = "[ROVERDRIVE] Bringup not performed"
        self.set_state()

    def check_connections(self):
        """Confirm that every RoboClaw and steering servo responds.

        Raises:
            RuntimeError: If any device does not respond, after emergency_stop().
        """
        failures : list[str] = []
        for roboclaw in self.roboclaws:
            try:
                if not roboclaw.unit.ReadVersion(roboclaw.address)[0]:
                    failures.append(roboclaw.name)
            except Exception as e:
                failures.append(f"{roboclaw.name}: {e}")

        for servo, config in zip(self.servos, self.configs):
            try:
                # Reading the angle reads the PCA9685 registers over I2C.
                servo.angle
            except Exception as e:
                failures.append(f"{config.name}: {e}")

        if failures:
            self.error = True
            self.error_message = f"[CHECKCONNECTIONS] Not responding: {failures}"
            self.log("check_connections", status="error", message=self.error_message)
            self.emergency_stop()
            raise RuntimeError(self.error_message)

    def bringup(self, 
                usb_ports: list[str], rc_addresses: list[int], rc_names: list[str], motor_cfgs: list[Tuple[Motor, int]],
                baud_rate: int, servo_channels: int, servo_pwm: int, servo_i2c_addr: int,
                servo_cfgs: list[ServoConfig],
                debug: bool, i2c_bus: Any = None, log_path: Any = None):
        """Unified bringup for Roboclaw MCUs and PCA9685 Servo Driver.

        Args:
            usb_ports: Permitted tty/usb ports.
            rc_addresses: Expected Roboclaw addresses.
            rc_names: Desired names for each Roboclaw.
            motor_cfgs: Motor configurations and ordering.
            baud_rate: MCU communication baud rate.
            servo_channels: ServoDriver channel count.
            servo_pwm: Servo PWM frequency.
            servo_i2c_addr: ServoDriver I2C address.
            servo_cfgs: Steering servo configurations, ordered as self.servos.
            debug: Debug enable.
            i2c_bus: Optional I2CBus instance.
            log_path: Optional CSV log path. Defaults to a timestamped file in python/log.
        """
        self.error = False
        self.error_message = ""
        self.debug = debug
        self.stop_counter = 0
        self.set_state()
        try:
            if self.log_file is not None:
                self.log_file.close()
            if log_path is None:
                log_name = datetime.now(timezone.utc).strftime("rover-%Y%m%dT%H%M%SZ.csv")
                log_path = Path(__file__).resolve().parents[1] / "log" / log_name
            log_path = Path(log_path)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = open(log_path, "a", newline="", buffering=1)
            # Time, record and motor columns are named as in the MotionCSVLogger output.
            columns = ["log_sec", "log_nanosec", "action", "status", "message",
                       "movement_state", "rotary_state", "stop_counter"]
            for motor, _ in motor_cfgs:
                prefix = f"motor_{motor.id}_{_column_name(motor.name)}"
                columns.extend((f"{prefix}_command_qpps", f"{prefix}_readback_qpps"))
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=columns, restval="", extrasaction="ignore")
            if log_path.stat().st_size == 0:
                self.log_writer.writeheader()
            # A repeated bringup releases the prior serial handles.
            for roboclaw in getattr(self, "roboclaws", []):
                roboclaw.unit.close()
            roboclaws = roboclaw_methods.roboclaw_setup(
                candidate_ports=usb_ports, expected_addresses=rc_addresses,
                names=rc_names, baud_rate=baud_rate, debug=debug)
            roboclaw_methods.motor_setup(
                motor_configs=motor_cfgs, roboclaws=roboclaws, debug=debug, readonly=True)
            servodriver = ServoKit(
                channels=servo_channels, i2c=i2c_bus,
                address=servo_i2c_addr, frequency=servo_pwm)
            if not servo_methods.servodriver_setup(
                servo=servodriver, pwm_frequency=servo_pwm,
                i2c_address=servo_i2c_addr):
                raise RuntimeError("[BRINGUP] Servo driver not accessible")
            self.roboclaws = roboclaws
            self.motors = [motor for motor, _ in motor_cfgs]
            self.servodriver = servodriver
            self.configs = servo_cfgs
            self.servos = [servodriver.servo[config.channel] for config in servo_cfgs]
            # goBILDA servo range, as configured in main.py.
            for servo in self.servos:
                servo.actuation_range = 300
                servo.set_pulse_width_range(500, 2500)
            self.check_connections()
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            self.log("bringup", status="error", message=str(e))
            if debug:
                print(f"[RoverDrive] bringup failed: {e}")
            raise

    def log(self, action: str, status: str = "ok", message: str = "") -> Command:
        """Record one method result as a CSV row and return it. Write failures are ignored."""
        motors = getattr(self, "motors", [])
        command = self.Command(
            action=action, status=status, message=message,
            speeds=[motor.speed for motor in motors],
            actual_speeds=[motor.actual_speed for motor in motors],
            movement_state=self.movement_state, rotary_state=self.rotary_state,
            stop_counter=self.stop_counter)
        if self.log_writer is not None:
            log_sec, log_nanosec = divmod(command.timestamp_ns, 1_000_000_000)
            row = {
                "log_sec": log_sec, "log_nanosec": log_nanosec,
                "action": command.action, "status": command.status, "message": command.message,
                "movement_state": command.movement_state.name, "rotary_state": command.rotary_state.name,
                "stop_counter": command.stop_counter,
            }
            for motor in motors:
                prefix = f"motor_{motor.id}_{_column_name(motor.name)}"
                row[f"{prefix}_command_qpps"] = motor.speed
                row[f"{prefix}_readback_qpps"] = motor.actual_speed
            try:
                self.log_writer.writerow(row)
            except (OSError, ValueError):
                pass
        return command

    def stop(self) -> Command:
        """Bring all motors to zero speed through roboclaw_methods.

        Escalates to emergency_stop() on failure.

        Returns:
            The recorded Command.
        """
        try:
            self.set_motors(motor_indices = list(range(len(self.motors))), motor_speeds = [0] * len(self.motors))
            roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
            return self.log("stop")
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            self.log("stop", status="error", message=str(e))
            if self.debug:
                print(f"[RoverDrive] stop failed: {e}")
            self.emergency_stop()
            raise

    def emergency_stop(self) -> Command:
        """Command zero speed to every RoboClaw directly, bypassing roboclaw_methods.

        All units are attempted. The fault latches until the next bringup().

        Returns:
            The recorded Command.

        Raises:
            RuntimeError: If any unit did not acknowledge, after all were attempted.
        """
        failures : list[str] = []
        for roboclaw in self.roboclaws:
            try:
                if not roboclaw.unit.SpeedM1M2(roboclaw.address, 0, 0):
                    failures.append(roboclaw.name)
            except Exception as e:
                failures.append(f"{roboclaw.name}: {e}")

        for motor in self.motors:
            motor.speed = 0
        self.error = True
        if not self.error_message:
            self.error_message = "Emergency stop issued"

        if failures:
            message = f"[EMERGENCYSTOP] Units not stopped: {failures}"
            self.error_message = f"{self.error_message}; {message}"
            self.log("emergency_stop", status="error", message=self.error_message)
            raise RuntimeError(message)
        return self.log("emergency_stop", message=self.error_message)

    def set_state(self, movement_state : MovementState = MovementState.STOPPED, rotary_state: RotaryState = RotaryState.STOPPED):
        self.movement_state = movement_state
        self.rotary_state = rotary_state
        if self.debug:
            print(f"[RoverDrive] set_state: movement={movement_state}, rotary={rotary_state}")

    def set_motors(self, motor_indices: list[int], motor_speeds: list[int]):
        """Set stored speeds for the selected motors.

        Args:
            motor_indices: Indices into self.motors identifying motors to update.
            motor_speeds: Speeds corresponding to each motor index.

        Raises:
            ValueError: If the lists differ in length; all stored motor speeds
                are set to zero before raising.
            IndexError: If a motor index is out of range.
        """
        if len(motor_indices) != len(motor_speeds):
            for motor in self.motors:
                motor.speed = 0
            raise ValueError("motor_indices and motor_speeds must have the same length")

        for motor_index, speed in zip(motor_indices, motor_speeds):
            self.motors[motor_index].speed = speed


    def hold_servos(self) -> Command:
        """Rewrite the current angle of every steering servo and record the result.

        A failure triggers emergency_stop().
        """
        try:
            servo_methods.hold_angle(servo=self.servos[0], config=self.configs[0])
            servo_methods.hold_angle(servo=self.servos[1], config=self.configs[1])
            servo_methods.hold_angle(servo=self.servos[2], config=self.configs[2])
            servo_methods.hold_angle(servo=self.servos[3], config=self.configs[3])
            return self.log("hold_servos")
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            self.log("hold_servos", status="error", message=str(e))
            if self.debug:
                print(f"[HOLDSERVOS] holding servos failed: {e}")
            self.emergency_stop()
            raise

    def steer_servos(self, input : SteerInput, step: int = 0) -> Command:
        """Step the selected axle, or return all servos to straight.

        Ignored during an in-place turn. A failure triggers emergency_stop().

        Args:
            input: The steering selection.
            step: Degrees to step. Unused for STRAIGHT.

        Returns:
            The recorded Command.
        """
        # Case Statement here, logic is plainly simple:
        # Don't hijack the inplace turn:
        if self.rotary_state not in (RotaryState.STOPPED, RotaryState.NOT_TURN):
            return self.log("steer_servos", status="ignored", message="In-place turn active")
        try:
            match input:
                case SteerInput.STRAIGHT:
                    servo_methods.hold_zeroes(active_servos = self.servos, active_configs = self.configs)
                case SteerInput.FRONT_RIGHT:
                    servo_methods.turn_servos(active_servos=[self.servos[1], self.servos[2]], 
                        active_configs=[self.configs[1], self.configs[2]], step=step, step_up = True)
                    servo_methods.hold_angle(servo=self.servos[0], config=self.configs[0])
                    servo_methods.hold_angle(servo=self.servos[3], config=self.configs[3])
                case SteerInput.FRONT_LEFT:
                    servo_methods.turn_servos(active_servos=[self.servos[1], self.servos[2]], 
                        active_configs=[self.configs[1], self.configs[2]], step=step, step_down = True)
                    servo_methods.hold_angle(servo=self.servos[0], config=self.configs[0])
                    servo_methods.hold_angle(servo=self.servos[3], config=self.configs[3])             
                case SteerInput.BACK_RIGHT:
                    servo_methods.turn_servos(active_servos=[self.servos[0], self.servos[3]], 
                        active_configs=[self.configs[0], self.configs[3]], step=step, step_down = True)
                    servo_methods.hold_angle(servo=self.servos[1], config=self.configs[1])
                    servo_methods.hold_angle(servo=self.servos[2], config=self.configs[2])
                case SteerInput.BACK_LEFT:
                    servo_methods.turn_servos(active_servos=[self.servos[0], self.servos[3]], 
                        active_configs=[self.configs[0], self.configs[3]], step=step, step_up = True)
                    servo_methods.hold_angle(servo=self.servos[1], config=self.configs[1])
                    servo_methods.hold_angle(servo=self.servos[2], config=self.configs[2])
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            if self.debug:
                print(f"[STEERSERVOS] steering servos failed: {e}")
            self.log("steer_servos", status="error", message=str(e))
            self.emergency_stop()
            raise
        return self.log("steer_servos", message=input.name)

    def set_inplace_steer(self, 
                        i_front_r: int, sign_fr: int, 
                        i_front_l: int, sign_fl: int,
                        i_back_r: int,  sign_br: int,
                        i_back_l: int,  sign_bl: int,
                          ):

        """Offset each corner steering servo by 45 degrees for in-place turning.

        Each sign selects the direction: +1 steps up; -1 steps down.
        Angles are adjusted from their current positions, so repeated calls
        apply additional offsets. Servo and configuration indices must match.

        Args:
            i_front_r: Front-right servo index (0–3).
            sign_fr: Front-right direction (+1 or -1).
            i_front_l: Front-left servo index (0–3).
            sign_fl: Front-left direction (+1 or -1).
            i_back_r: Back-right servo index (0–3).
            sign_br: Back-right direction (+1 or -1).
            i_back_l: Back-left servo index (0–3).
            sign_bl: Back-left direction (+1 or -1).

        Raises:
            ValueError: If an index is outside 0–3, indices are duplicated,
                or a sign is not +1 or -1.
        """

        try:
            indices = (i_front_r, i_front_l, i_back_r, i_back_l)
            signs = (sign_fr, sign_fl, sign_br, sign_bl)

            if any(index not in range(4) for index in indices):
                raise ValueError(
                    f"[SETINPLACESTEER] Servo Index Out Of Range: {indices}."
                )

            if len({i_front_r, i_front_l, i_back_r, i_back_l}) != 4:
                raise ValueError(
                    f"[SETINPLACESTEER] Duplicate Servo Indices: "
                    f"{i_front_r}, {i_front_l}, {i_back_r}, {i_back_l}."
                )


            if any(sign not in (-1, 1) for sign in signs):
                raise ValueError(
                    f"[SETINPLACESTEER] Servo Sign Out Of Range: {signs}."
                )
        except ValueError as e:
            self.log("set_inplace_steer", status="rejected", message=str(e))
            raise

        try:
            for index, sign in (
                (i_front_r, sign_fr),
                (i_front_l, sign_fl),
                (i_back_r, sign_br),
                (i_back_l, sign_bl),
            ):
                if sign == 1:
                    servo_methods.turn_servos(
                        active_servos=[self.servos[index]],
                        active_configs=[self.configs[index]],
                        step=45,
                        step_up=True,
                    )
                else:
                    servo_methods.turn_servos(
                        active_servos=[self.servos[index]],
                        active_configs=[self.configs[index]],
                        step=45,
                        step_down=True,
                    )
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            if self.debug:
                print(f"[SETINPLACESTEER] steering servos failed: {e}")
            self.log("set_inplace_steer", status="error", message=str(e))
            self.emergency_stop()
            raise
        return self.log("set_inplace_steer")

    def rotary_motion(self, speeds : list[int], left_motors: list[int], right_motors: list[int], debug : bool = False):
        """Rotate in place using opposing left/right motor speeds.

        Args:
            speeds: One shared speed or one per motor, ordered as self.motors.
            left_motors: Left-side indices into self.motors and speeds.
            right_motors: Right-side indices into self.motors and speeds.
            debug: Overridden by self.debug.

        Motor index lists must be distinct, non-overlapping, and cover all motors.
        Ensure set_inplace_steer() is called, otherwise, interesting behavior may result!
        """
        debug = self.debug
        try:
            # Unitary setting:
            if len(speeds) == 1:
                speeds = speeds * len(self.motors)
            # Invalid speed input(s):
            elif len(speeds) != len(self.motors):
                raise ValueError("[ROTARYMOTION] Speed Array must be 1 or equal to the number of motors")
            # Invalid maximum speed:
            if any(speed < 0 or speed > self.safe_maximum_speed for speed in speeds):
                raise ValueError("[ROTARYMOTION] Unsafe Speed Entered")
            # Invalid motor sides:
            if sorted(left_motors + right_motors) != list(range(len(self.motors))):
                raise ValueError("[ROTARYMOTION] Motor Sides must be distinct and cover all motors")
        except ValueError as e:
            self.log("rotary_motion", status="rejected", message=str(e))
            raise

        try:
            if self.error or self.stop_counter <= 0 or (self.movement_state == MovementState.STOPPED) or (self.rotary_state not in (RotaryState.TURN_LEFT, RotaryState.TURN_RIGHT)):
                # set all to zero:
                self.set_motors(motor_indices = list(range(len(speeds))), motor_speeds = [0] * len(self.motors))
                roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
                if (debug):
                    print("[ROTARYMOTION] MOTION STOPPED")
                return self.log("rotary_motion", status="stopped")
        
            # State Management:
            if (debug): 
                print(f"[ROTARYMOTION] Movement State: {self.movement_state}")
                print(f"[ROTARYMOTION] Rotary State: {self.rotary_state}")
                print(f"[ROTARYMOTION] Speeds: {speeds}")

            match self.rotary_state:
                case RotaryState.TURN_LEFT:
                    if (debug):
                        print(f"[ROTARYMOTION] TURNING LEFT at {speeds}")
                    # left moves forward, right moves backward:
                    self.set_motors(motor_indices = left_motors, motor_speeds = [-1 * speeds[i] for i in left_motors])
                    self.set_motors(motor_indices = right_motors, motor_speeds = [1 * speeds[i] for i in right_motors])
                    roboclaw_methods.move_motors(motors=self.motors, roboclaws=self.roboclaws)
                case RotaryState.TURN_RIGHT:
                    if(debug):
                        print(f'[ROTARYMOTION] TURNING RIGHT at {speeds}')
                    # left moves backward, right moves forward:
                    self.set_motors(motor_indices = left_motors, motor_speeds = [1 * speeds[i] for i in left_motors])
                    self.set_motors(motor_indices= right_motors, motor_speeds = [-1 * speeds[i] for i in right_motors])
                    roboclaw_methods.move_motors(motors=self.motors, roboclaws=self.roboclaws)
                case _:
                    if(debug):
                        print(f'[ROTARYMOTION] NOT TURNING RIGHT OR LEFT')
            self.stop_counter = self.stop_counter - 1
                
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            if debug:
                print(f"[RoverDrive] rotary motion failed: {e}")
            self.log("rotary_motion", status="error", message=str(e))
            self.emergency_stop()
            raise
        return self.log("rotary_motion")
    
    # Linear motion should not turn in place:
    # Linear motion may save the state of a steer which is NOT a turn in place.
    # This means forward and backward.
    def linear_motion(
        self,
        speeds: list[int],
        debug: bool = False,
    ) -> Command:
        """ 
        Drive all motors according to the stored movement state.

        Args:
            speeds: Positive speed magnitudes in QPPS. One shared value, or one per motor.
            debug: Overridden by self.debug.

        Raises:
            ValueError: If the speed array is invalid; motor commands are unchanged.

        Returns:
            The recorded Command.
        """
        debug = self.debug
        try:
            # Unitary setting:
            if len(speeds) == 1:
                speeds = speeds * len(self.motors)
            # Invalid speed input(s):
            elif len(speeds) != len(self.motors):
                raise ValueError("[LINEARMOTION] Speed Array must be 1 or equal to the number of motors")
            # Invalid maximum speed:
            if any(speed < 0 or speed > self.safe_maximum_speed for speed in speeds):
                raise ValueError("[LINEARMOTION] Unsafe Speed Entered")
        except ValueError as e:
            self.log("linear_motion", status="rejected", message=str(e))
            raise

        try: 
            # State Management:
            if (debug): 
                print(f"[LINEARMOTION] Movement State: {self.movement_state}")
                print(f"[LINEARMOTION] Rotary State: {self.rotary_state}")
                print(f"[LINEARMOTION] Speeds: {speeds}")
            
            # Fail fast at invalid stop counter or STOPPED state, or when called somehow during an inplace turn:
            # DEFAULTS TO STOPPED, which either renders the rover immovable, or stopepd.
            if self.error or self.stop_counter <= 0 or (self.movement_state == MovementState.STOPPED) or (self.rotary_state != RotaryState.NOT_TURN):
                # set all to zero:
                self.set_motors(motor_indices = list(range(len(speeds))), motor_speeds = [0] * len(self.motors))
                roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
                if (debug):
                    print("[LINEARMOTION] MOTION STOPPED")
                return self.log("linear_motion", status="stopped")

            # If we are here, we are not stopped, and we are not turning in place:
            # We are moving linearly, and we need to set the motors to the desired speeds:
            # Backward motion is commanded by negative speeds at the driver.
            if self.movement_state == MovementState.BACKWARD:
                speeds = [-1 * speed for speed in speeds]
            self.set_motors(motor_indices = list(range(len(speeds))), motor_speeds = speeds)
            roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
            self.stop_counter = self.stop_counter - 1      
  
        except Exception as e:
                    self.error = True
                    self.error_message = str(e)
                    if debug:
                        print(f"[RoverDrive] linear motor movement failed: {e}")
                    self.log("linear_motion", status="error", message=str(e))
                    self.emergency_stop()
                    raise
        return self.log("linear_motion")
