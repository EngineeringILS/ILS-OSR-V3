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


CONTROL_LOOP_INTERVAL_SECONDS = 0.01


def unified_bringup(candidate_ports : list[str], expected_addresses : list[int], roboclaw_names : list[str], motor_configs : list[Tuple[Motor, int]],
                    baud_rate : int, channels : int, pwm_frequency: int, i2c_address: int,
                    debug : bool, i2c_bus: Any = None
                    ) -> Tuple[list[RoboclawUnit], ServoKit]:

    ''' Build the Objects, run the setups, hope they work (prayer emoji): '''
    roboclaws: list[RoboclawUnit] = []
    roboclaws = roboclaw_methods.roboclaw_setup(candidate_ports=candidate_ports, expected_addresses=expected_addresses, names=roboclaw_names, baud_rate=baud_rate, debug=debug)
    roboclaw_methods.motor_setup(motor_configs=motor_configs, roboclaws=roboclaws, debug=debug, readonly=True)
    servodriver = ServoKit(channels=channels, i2c=i2c_bus, address=i2c_address, frequency=pwm_frequency)
    servo_methods.servodriver_setup(servo = servodriver, pwm_frequency=pwm_frequency, i2c_address=i2c_address)
    return roboclaws, servodriver

def macro_set_motor_speed(motors: list[Motor], speed: int):
    for motor in motors:
        motor.speed = speed
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

    if movement_select == 2:
        servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_down= True)
        servo_methods.hold_angle(servo=servos[0], config=configs[0])
        servo_methods.hold_angle(servo=servos[3], config=configs[3])
        return False

    if movement_select == 3:
        servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_down=True)
        servo_methods.hold_angle(servo=servos[1], config=configs[1])
        servo_methods.hold_angle(servo=servos[2], config=configs[2])
        return False
    
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
