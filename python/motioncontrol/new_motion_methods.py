"""
motion_methods.py
Integrated Methods for Setting up and Managing the Roboclaw MCU + PCA9685 Servo Driver.
Combines roboclaw_methods.py and servo_methods.py, adding the Phase 1 Integrated char_input Loop
with a counter-based dead-man stop and explicit 10 ms loop delay.
"""

''' System Imports: '''
import time
import tty
import termios
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
from pathlib import Path
from typing import Any
from datacollection.data_types import CommandResult
from motion_state import MotionHardware, MotionState, MotionTerminal

''' Servo Method Imports: '''
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

import servocontrol.servo_methods as servo_methods
from   servocontrol.servo_methods import ServoConfig

'''Motor Method Imports: '''
import motorcontrol.roboclaw_methods as roboclaw_methods
from motorcontrol.roboclaw_types import RoboclawUnit, Motor
import typing
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

# Movement actions accepted by RoverDrive.issue()
class CommandAction(IntEnum):
    STOP = 0
    EMERGENCY_STOP = 1
    FORWARD = 2
    BACKWARD = 3
    TURN_LEFT = 4
    TURN_RIGHT = 5
    STEER = 6
    INPLACE_STEER = 7
    HOLD = 8
    
CONTROL_LOOP_INTERVAL_SECONDS = 0.01
# Commands issued between connection checks; approximately 0.5 s at the loop interval.
CHECK_INTERVAL_COMMANDS = 50


# TODO: This class ONLY NEEDS TO BE MOVEMENT COMPLETE, DO NOT ADD ADDITIONAL METHODS WITHOUT ASKING
# TODO: Modifications should not rewrite existing code, unless for a safety critical, or bugfixing reason.
# TODO: Modifications ought to be minimal, not changing the underlying structure/syntax/style.
class RoverDrive:
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
    stop_counts : int = 0
    log_file : Any = None
    check_counter : int = 0

    @dataclass(frozen=True, slots=True)
    class Command:
        """One movement instruction for issue().

        Args:
            action: The movement to perform.
            speeds: Positive magnitudes in QPPS. One shared value, or one per motor.
            steer_input: Steering selection for STEER.
            step: Degrees for STEER.
            left_motors: Left-side motor indices for TURN_LEFT and TURN_RIGHT.
            right_motors: Right-side motor indices for TURN_LEFT and TURN_RIGHT.
            inplace_steer: The eight set_inplace_steer() arguments for INPLACE_STEER.
            refill: Refill the dead-man. A control loop repeating a command passes False.
            issued_ns: Issue time in nanoseconds.
        """
        action : CommandAction
        speeds : list[int] = field(default_factory=lambda: [0])
        steer_input : SteerInput = SteerInput.STRAIGHT
        step : int = 0
        left_motors : list[int] = field(default_factory=list)
        right_motors : list[int] = field(default_factory=list)
        inplace_steer : tuple[int, ...] = ()
        refill : bool = True
        issued_ns : int = field(default_factory=time.time_ns)

    def issue(self, command: Command):
        """Execute one movement command.

        STOP and EMERGENCY_STOP are always accepted. Other actions are refused while
        a fault is latched. Only commands with refill set are logged.

        Args:
            command: The command to execute.

        Raises:
            ValueError: If the command is invalid. The prior state remains in force.
            RuntimeError: If a fault is latched, or the hardware fails.
        """
        action = command.action
        if command.refill:
            self.log(f"COMMAND {action.name} speeds={command.speeds} issued_ns={command.issued_ns}")

        if action == CommandAction.EMERGENCY_STOP:
            self.emergency_stop()
            return
        if action == CommandAction.STOP:
            self.stop()
            return
        if self.error:
            raise RuntimeError(f"[ISSUE] Refused, fault latched: {self.error_message}")
        if action == CommandAction.INPLACE_STEER and len(command.inplace_steer) != 8:
            raise ValueError("[ISSUE] INPLACE_STEER requires eight arguments")

        prior_state = (self.movement_state, self.rotary_state, self.stop_counter)
        try:
            self.check_counter = self.check_counter + 1
            if self.check_counter >= CHECK_INTERVAL_COMMANDS:
                self.check_counter = 0
                self.check_connections()

            if command.refill and action in (CommandAction.FORWARD, CommandAction.BACKWARD, CommandAction.TURN_LEFT, CommandAction.TURN_RIGHT):
                self.stop_counter = self.stop_counts

            match action:
                case CommandAction.FORWARD:
                    self.set_state(MovementState.FORWARD, RotaryState.NOT_TURN)
                    self.linear_motion(speeds = command.speeds)
                case CommandAction.BACKWARD:
                    self.set_state(MovementState.BACKWARD, RotaryState.NOT_TURN)
                    self.linear_motion(speeds = command.speeds)
                case CommandAction.TURN_LEFT:
                    self.set_state(MovementState.TURN_INPLACE, RotaryState.TURN_LEFT)
                    self.rotary_motion(speeds = command.speeds, left_motors = command.left_motors, right_motors = command.right_motors)
                case CommandAction.TURN_RIGHT:
                    self.set_state(MovementState.TURN_INPLACE, RotaryState.TURN_RIGHT)
                    self.rotary_motion(speeds = command.speeds, left_motors = command.left_motors, right_motors = command.right_motors)
                case CommandAction.STEER:
                    self.steer_servos(input = command.steer_input, step = command.step)
                case CommandAction.INPLACE_STEER:
                    # Offsets are relative; zeroing first keeps a repeated command from accumulating.
                    servo_methods.hold_zeroes(active_servos = self.servos, active_configs = self.configs)
                    self.set_inplace_steer(*command.inplace_steer)
                case CommandAction.HOLD:
                    self.hold_servos()
        except ValueError:
            if not self.error:
                self.movement_state, self.rotary_state, self.stop_counter = prior_state
            raise
        except Exception as e:
            if not self.error:
                self.error = True
                self.error_message = str(e)
                self.emergency_stop()
            raise

    def __init__(self, *args, safe_maximum_speed: int = 100, **kwargs):
        """Set the speed limit and bring up the hardware.

        Args:
            safe_maximum_speed: Maximum speed magnitude in QPPS.
            *args, **kwargs: Arguments of bringup().
        """
        self.safe_maximum_speed = safe_maximum_speed
        self.bringup(*args, **kwargs)

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
            self.log(self.error_message)
            self.emergency_stop()
            raise RuntimeError(self.error_message)

    def bringup(self, 
                usb_ports: list[str], rc_addresses: list[int], rc_names: list[str], motor_cfgs: list[Tuple[Motor, int]],
                baud_rate: int, servo_channels: int, servo_pwm: int, servo_i2c_addr: int,
                servo_cfgs: list[ServoConfig], stop_counts: int,
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
            stop_counts: Motion ticks permitted per command before the dead-man stop.
            debug: Debug enable.
            i2c_bus: Optional I2CBus instance.
            log_path: Optional log file path. Defaults to a timestamped file in python/log.
        """
        self.error = False
        self.error_message = ""
        self.debug = debug
        self.stop_counts = stop_counts
        self.stop_counter = 0
        self.set_state()
        try:
            if self.log_file is not None:
                self.log_file.close()
            if log_path is None:
                log_name = datetime.now(timezone.utc).strftime("rover-%Y%m%dT%H%M%SZ.log")
                log_path = Path(__file__).resolve().parents[1] / "log" / log_name
            log_path = Path(log_path)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = open(log_path, "a", buffering=1)
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
            self.log(f"BRINGUP FAILED: {e}")
            if debug:
                print(f"[RoverDrive] bringup failed: {e}")
            raise

    def log(self, message: str):
        """Append one timestamped line to the log file. Write failures are ignored."""
        if self.log_file is None:
            return
        try:
            self.log_file.write(f"{time.time_ns()} {message}\n")
        except (OSError, ValueError):
            pass

    def stop(self):
        """Bring all motors to zero speed through roboclaw_methods.

        Escalates to emergency_stop() on failure.
        """
        try:
            self.set_state()
            self.stop_counter = 0
            self.set_motors(motor_indices = list(range(len(self.motors))), motor_speeds = [0] * len(self.motors))
            roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            self.log(f"STOP FAILED: {e}")
            if self.debug:
                print(f"[RoverDrive] stop failed: {e}")
            self.emergency_stop()
            raise

    def emergency_stop(self):
        """Command zero speed to every RoboClaw directly, bypassing roboclaw_methods.

        All units are attempted. The fault latches until the next bringup().

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
        self.stop_counter = 0
        self.set_state()
        self.error = True
        if not self.error_message:
            self.error_message = "Emergency stop issued"
        self.log(f"EMERGENCY STOP: {self.error_message}")

        if failures:
            message = f"[EMERGENCYSTOP] Units not stopped: {failures}"
            self.error_message = f"{self.error_message}; {message}"
            self.log(message)
            raise RuntimeError(message)

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


    def hold_servos(self):
        """Rewrite the current angle of every steering servo. A failure triggers emergency_stop()."""
        try:
            servo_methods.hold_angle(servo=self.servos[0], config=self.configs[0])
            servo_methods.hold_angle(servo=self.servos[1], config=self.configs[1])
            servo_methods.hold_angle(servo=self.servos[2], config=self.configs[2])
            servo_methods.hold_angle(servo=self.servos[3], config=self.configs[3])
        except Exception as e:
            self.error = True
            self.error_message = str(e)
            if self.debug:
                print(f"[HOLDSERVOS] holding servos failed: {e}")
            self.emergency_stop()
            raise

    def steer_servos(self, input : SteerInput, step: int = 0):
        """Step the selected axle, or return all servos to straight.

        Ignored during an in-place turn. A failure triggers emergency_stop().

        Args:
            input: The steering selection.
            step: Degrees to step. Unused for STRAIGHT.
        """
        # Case Statement here, logic is plainly simple:
        # Don't hijack the inplace turn:
        if self.rotary_state not in (RotaryState.STOPPED, RotaryState.NOT_TURN):
            return
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
            self.emergency_stop()
            raise
        return

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
            self.emergency_stop()
            raise

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

        try:
            if self.error or self.stop_counter <= 0 or (self.movement_state == MovementState.STOPPED) or (self.rotary_state not in (RotaryState.TURN_LEFT, RotaryState.TURN_RIGHT)):
                # set all to zero:
                self.set_motors(motor_indices = list(range(len(speeds))), motor_speeds = [0] * len(self.motors))
                roboclaw_methods.move_motors(motors = self.motors, roboclaws = self.roboclaws)
                if (debug):
                    print("[ROTARYMOTION] MOTION STOPPED")
                return
        
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
            self.emergency_stop()
            raise
        return
    
    # Linear motion should not turn in place:
    # Linear motion may save the state of a steer which is NOT a turn in place.
    # This means forward and backward.
    def linear_motion(
        self,
        speeds: list[int],
        debug: bool = False,
    ) -> None:
        """ 
        Drive all motors according to the stored movement state.

        Args:
            speeds: Positive speed magnitudes in QPPS. One shared value, or one per motor.
            debug: Overridden by self.debug.

        Raises:
            ValueError: If the speed array is invalid; motor commands are unchanged.
        """
        debug = self.debug
        # Unitary setting:
        if len(speeds) == 1:
            speeds = speeds * len(self.motors)
        # Invalid speed input(s):
        elif len(speeds) != len(self.motors):
            raise ValueError("[LINEARMOTION] Speed Array must be 1 or equal to the number of motors")
        # Invalid maximum speed:
        if any(speed < 0 or speed > self.safe_maximum_speed for speed in speeds):
            raise ValueError("[LINEARMOTION] Unsafe Speed Entered")

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
                return

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
                    self.emergency_stop()
                    raise
        return






"""
    CASES (movement_select): 
    0 = STOPPED
    1 = FORWARD
    2 = BACKWARD
    3 = LEFT FORWARD, RIGHT BACKWARD
    4 = LEFT BACKWARD, RIGHT FORWARD
    OTHER = STOPPED
"""
def linear_motor_movement(
    servos: list[Servo],
    configs: list[ServoConfig],
    roboclaws: list[RoboclawUnit],
    motors: list[Motor],
    speed: int,
    movement_select: int,
    stop_counter: int,
):
    """
    Provides control over linear motor motion, e.g. moves motors backwards and forwards.
    Sets Motor Speeds, and controls motors, logic is completely separate from steering.
    Args:
        servos (list[Servo]): Array of ServoDriver Servo Objects
        configs (list[ServoConfig]): Array of ServoConfig Objects
        roboclaws (list[RoboclawUnit]): Array of Roboclaw MCU Objects
        motors (list[Motor]): Array of Motor Objects
        speed (int): The desired speed in QPPS
        movement_select (int): The desired movement state selector
        stop_counter (int): The stop counter to decrement or reset
    """    

    # SAFETY STOP — robot stops immediately if watchdog expires.
    if stop_counter <= 0:
        macro_set_motor_speed(motors=motors, speed=0)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    # Hold current steering position during normal linear movement.
    if movement_select in (0, 1, 2, 3, 4):
        servo_methods.hold_angle(servo=servos[0], config=configs[0])
        servo_methods.hold_angle(servo=servos[1], config=configs[1])
        servo_methods.hold_angle(servo=servos[2], config=configs[2])
        servo_methods.hold_angle(servo=servos[3], config=configs[3])

    # STOP
    if movement_select == 0:
        macro_set_motor_speed(motors=motors, speed=0)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    # FORWARD
    if movement_select == 1:
        macro_set_motor_speed(motors=motors, speed=speed)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    # BACKWARD
    if movement_select == 2:
        macro_set_motor_speed(motors=motors, speed=-speed)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    # INPLACE TURN
    if movement_select in (3,4):
        # SAFETY: TURN SPEED LIMIT, DOES NOT MUTATE GLOBAL SPEED :)
        if speed > 1000:
            speed = 1000
        left_motors  = [motors[0], motors[2], motors[4]]
        right_motors = [motors[1], motors[3], motors[5]]
        if movement_select == 3:
            macro_set_motor_speed(left_motors, speed=-speed)
            macro_set_motor_speed(right_motors, speed=speed)
            roboclaw_methods.move_motors(motors=left_motors + right_motors, roboclaws=roboclaws)
        elif movement_select == 4:
            macro_set_motor_speed(left_motors, speed=speed)
            macro_set_motor_speed(right_motors, speed=-speed)
            roboclaw_methods.move_motors(motors=left_motors + right_motors, roboclaws=roboclaws)
        return

    # FAIL SAFE:
    # Any unsupported movement command stops the robot.
    macro_set_motor_speed(motors=motors, speed=0)
    roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
    return

"""
    CASES (movement_select): 
    0 = DON'T TURN
    1 = TURN FRONT TO RIGHT
    2 = TURN FRONT TO LEFT
    3 = TURN BACK TO RIGHT
    4 = TURN BACK TO LEFT
    5 = TURN IN PLACE LEFT
    6 = TURN IN PLACE RIGHT
    OTHER = DON'T TURN
"""
def turning_movement( 
    servos: list[Servo],
    configs: list[ServoConfig],
    roboclaws: list[RoboclawUnit],
    motors: list[Motor],
    speed: int,
    step_degrees: int,
    movement_select: int,
    stop_counter: int,
    inplace_turn: bool,
    ) -> bool:
    """ Provides servo movement and limited motor calls for inplace turning.
      This function depends on linear_motor_movement being properly integrated.

    Args:
        servos (list[Servo]): Array of ServoDriver Servo Objects
        configs (list[ServoConfig]): Array of ServoConfig Objects
        roboclaws (list[RoboclawUnit]): Array of Roboclaw MCU Objects
        motors (list[Motor]):  Array of Motor Objects
        speed (int): The desired speed in QPPS
        step_degrees (int): The turning step in degrees
        movement_select (int): The desired movement state selector
        stop_counter (int):  The stop counter to decrement or reset
        inplace_turn (bool): State toggle for inplace turning

    Returns:
        bool: _description_
    """
    # If we were previously turning in place, and now want to turn normally:
    if inplace_turn and movement_select in (1,2,3,4):
        servo_methods.hold_zeroes(active_servos=servos, active_configs=configs)
        inplace_turn = False
    # Turn Right:
    if movement_select == 1:
        servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_up = True)
        servo_methods.hold_angle(servo=servos[0], config=configs[0])
        servo_methods.hold_angle(servo=servos[3], config=configs[3])
        return False

    # Turn Left:
    if movement_select == 2:
        servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_down= True)
        servo_methods.hold_angle(servo=servos[0], config=configs[0])
        servo_methods.hold_angle(servo=servos[3], config=configs[3])
        return False

    # TODO: Turn back to the Right (verify)
    if movement_select == 3:
        servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_down=True)
        servo_methods.hold_angle(servo=servos[1], config=configs[1])
        servo_methods.hold_angle(servo=servos[2], config=configs[2])
        return False

    # TODO: Turn Back to the Left (verify)
    if movement_select == 4:
        servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_up=True)
        servo_methods.hold_angle(servo=servos[1], config=configs[1])
        servo_methods.hold_angle(servo=servos[2], config=configs[2])
        return False
    # Turning in place requires a reference from zero, it is simplest just to move to straight and then adjust from there, it is janky on the movement, but this is basic movement testing
    # This won't work without linear_motor_movement being updated, but, the behavior produced is just the rover will not move, which is safe enough.
    if movement_select == 5 or movement_select == 6:
        if not inplace_turn:
            servo_methods.hold_zeroes(active_servos=servos, active_configs=configs)
            servo_methods.turn_servos(active_servos=[servos[0], servos[2]], active_configs=[configs[0], configs[2]], step=45, step_up=True)
            servo_methods.turn_servos(active_servos=[servos[1], servos[3]], active_configs=[configs[1], configs[3]], step=45, step_down=True)
        if movement_select == 5: 
            linear_motor_movement(servos=servos, configs=configs, roboclaws=roboclaws, motors=motors,speed=speed, movement_select=3,stop_counter=stop_counter)
        elif movement_select == 6:
            linear_motor_movement(servos=servos, configs=configs, roboclaws=roboclaws, motors=motors,speed=speed, movement_select=4,stop_counter=stop_counter)
        return True
        
    return False



def command_result(state: MotionState, action: str, message: str, terminal: MotionTerminal, status: str = "ok", announce: bool = True) -> CommandResult:
    """Snapshot an outcome; CSV keeps full text while terminal output is capped."""
    result = CommandResult(action, status, message, state.selected_speed_qpps, state.drive_mode)
    if announce:
        terminal.report(message)
    return result


def shift_speed(command: str, state: MotionState, hardware: MotionHardware, terminal: MotionTerminal) -> CommandResult:
    """Change only the selected speed; do not send a motor command."""
    step = 1 if command == "t" else -1
    state.speed_index = max(0, min(state.speed_index + step, len(hardware.speeds) - 1))
    state.selected_speed_qpps = hardware.speeds[state.speed_index]
    return command_result(state, "shift_speed", f"Speed selected: {state.selected_speed_qpps} QPPS; applies on next drive command.", terminal)


def stop_drive(state: MotionState, hardware: MotionHardware) -> None:
    """Attempt zero speed on every controller; raise after collecting failures."""
    macro_set_motor_speed(hardware.motors, 0)
    state.watchdog_ticks = 0
    failures = []
    for controller in hardware.roboclaws:
        try:
            if not controller.unit.SpeedM1M2(controller.address, 0, 0):
                raise RuntimeError("stop not acknowledged")
        except Exception as error:
            failures.append(f"{controller.name}: {error}")
    state.watchdog_stopped = not failures
    state.drive_mode = "unknown" if failures else "stopped"
    if failures:
        raise RuntimeError("; ".join(failures))


def apply_motion_command(command: str, state: MotionState, hardware: MotionHardware, terminal: MotionTerminal) -> CommandResult:
    """Apply drive/steering/stop and update state; hardware errors reach cleanup."""
    if command in (" ", "c"):
        stop_drive(state, hardware)
        if command == "c":
            servo_methods.hold_zeroes(active_servos=hardware.servos, active_configs=hardware.configs)
            state.inplace_turn = False
            state.exit_requested = True
        return command_result(state, "stop" if command == " " else "exit", "Drive stopped." if command == " " else "Drive stopped; steering centered; exiting.", terminal)

    # Stop pivot motion before changing wheel geometry.
    if state.inplace_turn and command in ("a", "q", "d", "e", "b", "w", "s"):
        stop_drive(state, hardware)
        servo_methods.hold_zeroes(active_servos=hardware.servos, active_configs=hardware.configs)
        state.inplace_turn = False

    if command == "b":
        servo_methods.hold_zeroes(active_servos=hardware.servos, active_configs=hardware.configs)
        state.inplace_turn = False
        return command_result(state, "center", f"Steering centered; drive mode: {state.drive_mode}.", terminal)

    if command in ("a", "q", "d", "e"):
        selection = {"a": 1, "d": 2, "q": 3, "e": 4}[command]
        state.inplace_turn = turning_movement(servos=hardware.servos, configs=hardware.configs, roboclaws=hardware.roboclaws, motors=hardware.motors, speed=state.selected_speed_qpps, step_degrees=hardware.step_degrees, movement_select=selection, stop_counter=state.watchdog_ticks, inplace_turn=state.inplace_turn)
        return command_result(state, "steer", f"Steering command '{command}' applied.", terminal)

    state.watchdog_ticks = hardware.stop_counts // 4 if command in ("f", "g") else hardware.stop_counts
    if command in ("f", "g"):
        selection = 5 if command == "f" else 6
        state.inplace_turn = turning_movement(servos=hardware.servos, configs=hardware.configs, roboclaws=hardware.roboclaws, motors=hardware.motors, speed=state.selected_speed_qpps, step_degrees=hardware.step_degrees, movement_select=selection, stop_counter=state.watchdog_ticks, inplace_turn=state.inplace_turn)
    elif command in ("w", "s"):
        linear_motor_movement(servos=hardware.servos, configs=hardware.configs, roboclaws=hardware.roboclaws, motors=hardware.motors, speed=state.selected_speed_qpps, movement_select=1 if command == "w" else 2, stop_counter=state.watchdog_ticks)
    else:
        raise ValueError(f"Unsupported motion command: {command!r}")
    state.watchdog_stopped = state.watchdog_ticks <= 0
    state.drive_mode = "stopped" if state.watchdog_stopped else {"w": "forward", "s": "reverse", "f": "inplace_left", "g": "inplace_right"}[command]
    return command_result(state, "motion", f"Drive mode: {state.drive_mode}.", terminal)


def check_watchdog(state: MotionState, hardware: MotionHardware, terminal: MotionTerminal) -> CommandResult | None:
    """Stop once when ticks expire; never depend on steering I/O to stop."""
    if state.watchdog_ticks > 0 or state.watchdog_stopped:
        return None
    state.input_command = "watchdog_stop"
    state.command_timestamp_ns = time.time_ns()
    stop_drive(state, hardware)
    return command_result(state, "watchdog_stop", "Watchdog stop: drive commanded to zero.", terminal)


def record_telemetry(state: MotionState, hardware: MotionHardware, collector, logger, terminal: MotionTerminal, result: CommandResult | None = None, announce: bool = True) -> None:
    """Queue each result, even without an IMU; sample-only rows require new data."""
    if logger is None or not state.logging_enabled:
        return
    try:
        sample = collector.latest(max_age_seconds=0.25) if collector is not None else None
    except Exception:
        sample = None
    if result is None and (sample is None or sample.sequence == state.last_imu_sequence):
        return
    try:
        if logger.last_error is not None:
            raise RuntimeError("CSV writer failed")
        accepted = logger.submit(sample, command_timestamp_ns=state.command_timestamp_ns, input_command=state.input_command, drive_mode=state.drive_mode if result is None else result.drive_mode, selected_speed_qpps=state.selected_speed_qpps if result is None else result.selected_speed_qpps, watchdog_ticks_remaining=state.watchdog_ticks, inplace_turn=state.inplace_turn, motors=hardware.motors, action="sample" if result is None else result.action, status="ok" if result is None else result.status, message="" if result is None else result.message)
        if not accepted and not state.queue_warning_sent and announce:
            terminal.report("Telemetry queue full; records dropped.")
            state.queue_warning_sent = True
        elif accepted:
            state.queue_warning_sent = False
        if accepted and sample is not None:
            state.last_imu_sequence = sample.sequence
    except Exception:
        state.logging_enabled = False
        if announce:
            terminal.report("Telemetry logging failed; logging disabled.")


def shutdown_motion(state: MotionState, hardware: MotionHardware, terminal: MotionTerminal) -> CommandResult:
    """Attempt all motor stops before centering or reporting; preserve failures."""
    stop_drive(state, hardware)
    servo_methods.hold_zeroes(active_servos=hardware.servos, active_configs=hardware.configs)
    state.inplace_turn = False
    return command_result(state, "shutdown", "Shutdown: drive stopped; steering centered.", terminal)


def motion_movement_loop(servodriver: ServoKit, servos: list[Servo], configs: list[ServoConfig], step_degrees: int, roboclaws: list[RoboclawUnit], motors: list[Motor], speeds: list[int] = [0], stop_counts: int = 0, debug: bool = False, imu_collector: Any = None, telemetry_logger: Any = None, terminal=None):
    """Poll keys, dispatch actions, log outcomes, and sleep explicitly for 10 ms.

    r/t select speed; a/q/d/e steer; w/s drive; f/g pivot; space stops;
    b centers; c or input disconnect exits. A shared terminal provides fileno,
    poll, and report; MotionTerminal caps output at 79 visible characters.
    Hardware failures are logged and raised after best-effort shutdown.
    Watchdog ticks include sleep plus processing time, not a fixed deadline.
    debug is retained for caller compatibility; feedback is event-based.
    """
    if not speeds or stop_counts < 0:
        raise ValueError("Provide at least one speed and a nonnegative watchdog count")
    terminal = MotionTerminal() if terminal is None else terminal
    hardware = MotionHardware(servodriver, servos, configs, roboclaws, motors, step_degrees, stop_counts, speeds)
    state = MotionState(selected_speed_qpps=speeds[0], command_timestamp_ns=time.time_ns())
    fd = terminal.fileno()
    original_settings = termios.tcgetattr(fd)
    failure = None
    try:
        tty.setraw(fd)
        while True:
            result = check_watchdog(state, hardware, terminal)
            if result is not None:
                record_telemetry(state, hardware, imu_collector, telemetry_logger, terminal, result)
            command = terminal.poll()
            if command is not None:
                state.input_command = "disconnect" if command == "" else "space" if command == " " else command
                state.command_timestamp_ns = time.time_ns()
            match command:
                case "r" | "t":
                    result = shift_speed(command, state, hardware, terminal)
                case "a" | "q" | "d" | "e" | "b" | "w" | "s" | "f" | "g" | " " | "c" | "":
                    result = apply_motion_command("c" if command == "" else command, state, hardware, terminal)
                case None:
                    result = None
                case _:
                    result = command_result(state, "ignored", f"Unknown command: {command!r}", terminal, status="ignored")
            record_telemetry(state, hardware, imu_collector, telemetry_logger, terminal, result)
            if state.exit_requested:
                break
            state.watchdog_ticks = max(0, state.watchdog_ticks - 1)
            time.sleep(CONTROL_LOOP_INTERVAL_SECONDS)
    except BaseException as error:
        failure = error
        state.drive_mode = "unknown"
        result = command_result(state, "control_error", f"{type(error).__name__}: {error}", terminal, status="error", announce=False)
        record_telemetry(state, hardware, imu_collector, telemetry_logger, terminal, result, announce=False)
        raise
    finally:
        try:
            state.input_command = "shutdown"
            state.command_timestamp_ns = time.time_ns()
            try:
                result = shutdown_motion(state, hardware, terminal)
            except Exception as error:
                result = command_result(state, "shutdown", f"Shutdown failed: {error}", terminal, status="error")
                record_telemetry(state, hardware, imu_collector, telemetry_logger, terminal, result)
                if failure is None:
                    raise
            else:
                record_telemetry(state, hardware, imu_collector, telemetry_logger, terminal, result)
            if failure is not None:
                terminal.report(f"Control failed: {failure}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_settings)


# TODO:
# NEW API
#
