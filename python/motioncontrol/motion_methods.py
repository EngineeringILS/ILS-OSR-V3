"""
motion_methods.py
Integrated Methods for Setting up and Managing the Roboclaw MCU + PCA9685 Servo Driver.
Combines roboclaw_methods.py and servo_methods.py, adding the Phase 1 Integrated Movement Loop
with a control-input watchdog (gradual dead-man decel) and symmetric accel/decel speed ramping.
"""
import time
# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw
# Servo Driver Base Library
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

# FSM State Curves and Control:
# Think ECEN-248 + ECEN-214 Type Logic:
from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
from roboclaw_types import MotorCurveLUT, populate_curve_lut, MovementState, MovementSubState, SpeedConfig, MotorCurveLUTConfig, MotorControllerState
from servo_types import ServoConfig

# Fixing Function Input Types:
import typing

# For Active Input:
import sys
import tty
import termios
# For Timed/Nonblocking Input (Watchdog-Compatible):
import select


""" ------------------------------- Setup Methods ------------------------------- """

def roboclaw_setup(roboclaw : Roboclaw, serial_port: str, baud_rate: int, controller_address: int) -> bool:
    """
    Set up the Roboclaw, testing connection-open, attempt to get firmware address.

    :param roboclaw: The Roboclaw MCU Object (Basicmicro Alias)
    :type roboclaw: Roboclaw
    :param serial_port: The Linux Serial port for the Roboclaw Serial Connection.
    :type serial_port: str
    :param baud_rate: The Serial BAUD Rate for the Serial Connection.
    :type baud_rate: int
    :param controller_address: The Hex Address of the Roboclaw Motor Controller.
    :type controller_address: int 
    :return: True if the Roboclaw Connection was opened and Firmware Version returned.
    :rtype: bool | None
    """
    print(f"Starting Setup with Port: {serial_port}, BAUD Rate: {baud_rate}, Controller Address: {controller_address}.")
    print(f"Attempting to Connect to Roboclaw.")
    # Attempt Connection:
    roboclaw.Open()
    try:
        firmware_version = roboclaw.ReadVersion(controller_address)
        if (firmware_version[0]):
            print(f"Connected to Roboclaw with firmware version: {firmware_version[1]}.")
            return True
        else:
            print(f"Failed to connect to Roboclaw: (Firmware Read Failed).")
            return False
    except Exception as error:
        print(f"Error Connecting to Roboclaw with Port: {serial_port}, BAUD Rate: {baud_rate}, Controller Address: {controller_address}")
        print(str(error))


def servodriver_setup(servo : ServoKit, pwm_frequency: int, i2c_address: int) -> bool:
    """
    Configure the PCA9685 servo driver and verify that it is accessible.

    :param servo: The Adafruit ServoKit driver object.
    :type servo: ServoKit
    :param pwm_frequency: The PWM frequency used to drive the servos, in hertz.
    :type pwm_frequency: int
    :param i2c_address: The I2C address of the PCA9685 servo driver.
    :type i2c_address: int
    :return: True if the servo driver is configured successfully.
    :rtype: bool
    """
    print(f"Starting Setup with Port: {i2c_address}, PWM Frequency: {pwm_frequency}.")
    print(f"Attempting to Connect to Servo Driver.")
    # Attempt Connection:
    try:
        servo.servo[0].angle = None
        print(f"Connected to PCA9685 servo driver at I2C address {i2c_address:#04x}.")
        return True

    except Exception as error:
        print(f"Failed to connect to PCA9685 servo driver at I2C address {i2c_address:#04x}.")
        print(str(error))
        return False


""" ---------------------- Servo Helper Functions ---------------------- """
## ONLY USE WITH GOBILDA SERVOS:

def servodriver_setzeroes(servodriver: ServoKit, s0_pos : float, s1_pos : float, s2_pos : float, s3_pos : float):
    servos =  [servodriver.servo[0], servodriver.servo[1], servodriver.servo[2], servodriver.servo[3]]
    for servo in servos:
        servo.actuation_range = 300
    servos[0].angle = s0_pos
    servos[1].angle = s1_pos
    servos[2].angle = s2_pos
    servos[3].angle = s3_pos

    i = 0
    for servo in servos:
        print(f"Servo: {i} | Position: {servo.angle}")
        i += 1


def hold_straight(servo : Servo, config : ServoConfig):
    servo.angle = config.straight
    return


def front_turn_left(leftservo : Servo, leftconfig : ServoConfig, rightservo: Servo, rightconfig : ServoConfig, step : float):
    left_angle, right_angle = leftservo.angle, rightservo.angle

    if left_angle is None:
        left_angle = leftconfig.straight
    if right_angle is None:
        right_angle = rightconfig.straight 
    
    left_angle, right_angle = leftconfig.step_up(left_angle, step), rightconfig.step_up(right_angle, step)

    leftservo.angle, rightservo.angle = left_angle, right_angle
    return


def front_turn_right(leftservo : Servo, leftconfig : ServoConfig, rightservo: Servo, rightconfig : ServoConfig, step : float):
    left_angle, right_angle = leftservo.angle, rightservo.angle

    if left_angle is None:
        left_angle = leftconfig.straight
    if right_angle is None:
        right_angle = rightconfig.straight 
    
    left_angle, right_angle = leftconfig.step_down(left_angle, step), rightconfig.step_down(right_angle, step)

    leftservo.angle, rightservo.angle = left_angle, right_angle
    return


""" Basic Movement Functions, Separated for Future Integration, e.g. More Specific Checking/Differentials for different modes."""

def move_forward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
    return


def move_backward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=-speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
    return


def rotate_right(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None: 
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=-speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
    return


def rotate_left(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)-> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed", end='\r\n')
    return


def brake(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)->None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed", end='\r\n')
    return


""" ---------------------- Old Scuffed Testing Loops (Kept, Modularly Replaceable) ---------------------- """

def roboclaw_movement_loop(roboclaw : Roboclaw, speed: float, controller_address: float, debug: bool):
    """
    Basic Tank Drive Motor Control Loop (WASD)

    :param roboclaw: The Roboclaw MCU Object (Basicmicro Alias)
    :type roboclaw: Roboclaw
    :param speed: The desired set speed for the connected Motors.
    :type speed: float
    :param debug: Enable Debug Printouts in the Motor Control Loop itself.
    :type debug: bool
    """
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)

    movement_loop = True
    try:
        if (debug):
            print("Beginning Movement Loop (C/c to Cancel): ")
        tty.setraw(fd)
        while (movement_loop):
            movement = sys.stdin.read(1)
            if (debug): 
                print(movement[0], end='\r\n')
                sys.stdin.flush()
            movement = movement.lower()
            if movement == ("w"):
                # Move Forward
                move_forward(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
            elif movement == ("a"):
                # Rotate Right
                rotate_right(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
            elif movement == ("s"):
                # Move Backward
                move_backward(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
            elif movement == ("d"):
                # Rotate Left
                rotate_left(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
            elif movement == (" "):
                brake(roboclaw=roboclaw, speed=0, controller_address=controller_address, debug=debug)
                if (debug):
                    print("BRAKING", end='\r\n')
            elif movement == ("c"):
                roboclaw.SpeedM1M2(address=controller_address, m1=0, m2=0)
                break
            else:
                print ("C to break.", end='\r\n')
                pass
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)


def testing_servo_movement_loop(servo: ServoKit, channel: int, step_degrees, max_angle, min_angle, set_start_point,  debug: bool):
    """
    A/D moves the servo left and right.

    Space centers the servo.

    C disables PWM output and exits.

    :param servo: The Adafruit ServoKit driver object.
    :type servo: ServoKit
    :param channel: The PCA9685 channel connected to the servo.
    :type channel: int
    :param step_degrees: Degrees moved per keypress.
    :type step_degrees: float
    :param debug: Enable debug printouts.
    :type debug: bool
    """
    
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)

    selected_servo = servo.servo[channel]
    movement_loop = True
    selected_servo.actuation_range = max_angle
    current_angle = selected_servo.angle
    if current_angle is None:
        current_angle = 90
        selected_servo.angle = current_angle
    
    try:
        if (debug):
            print(f"Controlling servo channel {channel}. ")
            print("Beginning Movement Loop (C/c to Cancel): ")
        tty.setraw(fd)
        while movement_loop:
            movement = sys.stdin.read(1).lower()

            if debug:
                print(movement, end="\r\n")
                sys.stdout.flush()

            if movement == "a":
                current_angle = max(min_angle, current_angle - step_degrees)
                selected_servo.angle = current_angle

            elif movement == "d":
                current_angle = min(max_angle, current_angle + step_degrees)
                selected_servo.angle = current_angle

            elif movement == " ":
                current_angle = set_start_point
                selected_servo.angle = current_angle

            elif movement == "f":
                selected_servo.angle = None
                current_angle = 0
                if (debug): 
                    print(f"PWM DISABLED — NEXT COMMAND STARTS FROM {current_angle}", end="\r\n")

            elif movement == "c":
                selected_servo.angle = None
                break

            if (debug):
                    print(f"SERVO ANGLE: {selected_servo.angle}")
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)


def servo_movement_loop(servodriver : ServoKit, servos: list[Servo], configs: list[ServoConfig] , step_degrees, debug: bool):
    """
    A/D moves the servo left and right.

    Space centers the servo.

    C disables PWM output and exits.

    :param servo: The Adafruit ServoKit driver object.
    :type servo: ServoKit
    :param channel: The PCA9685 channel connected to the servo.
    :type channel: int
    :param step_degrees: Degrees moved per keypress.
    :type step_degrees: float
    :param debug: Enable debug printouts.
    :type debug: bool
    """
    
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)
    movement_loop = True
    
    try:
        if (debug):
            print(f"Controlling servos {configs[0].name}, {configs[1].name}. ")
            print("Beginning Movement Loop (C/c to Cancel): ")
        tty.setraw(fd)
        while movement_loop:
            movement = sys.stdin.read(1).lower()

            if debug:
                print(movement, end="\r\n")
                sys.stdout.flush()

            if movement == "a":
                front_turn_right(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                hold_straight(servo=servos[0], config=configs[0])
                hold_straight(servo=servos[2], config=configs[2])

            elif movement == "d":
                front_turn_left(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                hold_straight(servo=servos[0], config=configs[0])
                hold_straight(servo=servos[2], config=configs[2])

            elif movement == " ":
                servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)

            elif movement == "c":
                servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
                break

            if (debug):
                print("SERVO ANGLES:", end="\r\n")
                print(f"SERVO [0]: {servos[0].angle}", end="\r\n")
                print(f"SERVO [1]: {servos[1].angle}", end="\r\n")
                print(f"SERVO [2]: {servos[2].angle}", end="\r\n")
                print(f"SERVO [3]: {servos[3].angle}", end="\r\n")
                sys.stdout.flush()
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)


""" ------------------------ Phase 1 Integrated Movement Loop ------------------------ """

def integrated_movement_loop(
    roboclaw : Roboclaw,
    speed: float,
    controller_address: int,
    servodriver : ServoKit,
    servos: list[Servo],
    configs: list[ServoConfig],
    step_degrees: float,
    debug: bool,
    watchdog_timeout: float = 10,
    control_period: float = 0.05,
    deceleration_step: float = 3,
    acceleration_step: float = 3
):
    """
    Phase 1 Integrated Drive + Steer Control Loop (WASD) with Control-Input Watchdog
    and Symmetric Accel/Decel Speed Ramping.

    W/S: Latch a forward/reverse TARGET (persists after keypress). The commanded
         speed ramps toward the target by acceleration_step per tick, rather than
         snapping to full speed. Reversing direction mid-motion (W while reversing,
         S while driving forward) decelerates through zero first, then accelerates
         the other way - kind to the drivetrain, kind to the regolith.
    A/D: Step the front steering servos (does NOT touch motor speed/direction).
    Space: Immediate motor stop + recenter steering servos (bypasses the ramp).
    C: Zero motors, recenter servos, restore terminal, exit.

    Watchdog: ANY valid operator command (W/A/S/D) renews last_valid_input_time.
    If no valid command arrives within watchdog_timeout, the TARGET speed is set to
    zero and the same ramp logic gradually decelerates the motors (one
    deceleration_step per control_period). Space remains the abrupt manual stop.

    :param roboclaw: The Roboclaw MCU Object (Basicmicro Alias)
    :type roboclaw: Roboclaw
    :param speed: The desired set speed for the connected Motors.
    :type speed: float
    :param controller_address: The Hex Address of the Roboclaw Motor Controller.
    :type controller_address: int
    :param servodriver: The Adafruit ServoKit driver object.
    :type servodriver: ServoKit
    :param servos: The four steering Servo objects [0=BR, 1=FR, 2=BL, 3=FL].
    :type servos: list[Servo]
    :param configs: The matching ServoConfig list (channel order must match servos).
    :type configs: list[ServoConfig]
    :param step_degrees: Degrees moved per steering keypress.
    :type step_degrees: float
    :param debug: Enable Debug Printouts in the Motor Control Loop itself.
    :type debug: bool
    :param watchdog_timeout: Seconds without a valid W/A/S/D input before decel begins.
    :type watchdog_timeout: float
    :param control_period: The select() timeout, i.e. the loop tick period in seconds.
    :type control_period: float
    :param deceleration_step: Speed magnitude removed per tick while ramping down
                              (watchdog decel, target reductions, and reversals).
                              If <= 0, defaults to a ~1 second ramp from full speed.
    :type deceleration_step: float
    :param acceleration_step: Speed magnitude added per tick while ramping up toward
                              a latched W/S target. If <= 0, defaults to a ~0.5
                              second ramp from zero to full speed.
    :type acceleration_step: float
    """
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)

    # Default watchdog decel: full commanded speed -> 0 over ~1 second of ticks.
    if deceleration_step <= 0:
        deceleration_step = max(1.0, speed * control_period / 1.0)
    # Default accel: 0 -> full commanded speed over ~0.5 seconds of ticks.
    if acceleration_step <= 0:
        acceleration_step = max(1.0, speed * control_period / 0.5)

    # Persistent Control State:
    current_speed = 0.0                        # Commanded speed MAGNITUDE actually sent (always >= 0).
    drive_direction = 0                        # Direction of current_speed: 1 = fwd, -1 = rev, 0 = stopped.
    target_speed = 0.0                         # Speed MAGNITUDE the ramp is heading toward.
    target_direction = 0                       # Direction the operator has latched with W/S.
    watchdog_active = False                    # True while the watchdog is ramping us down.
    last_sent = (0, 0)                         # (direction, int(speed)) last commanded - avoids serial spam.
    last_valid_input_time = time.monotonic()   # Renewed by any valid W/A/S/D input.

    movement_loop = True
    try:
        if (debug):
            print("Beginning Integrated Movement Loop (C/c to Cancel): ")
        tty.setraw(fd)
        while (movement_loop):
            # Timed input: wait up to control_period for a keypress.
            # If nothing arrives, fall through to the watchdog check instead of blocking.
            readable, _, _ = select.select([sys.stdin], [], [], control_period)
            movement = ""
            if readable:
                movement = sys.stdin.read(1)
                if (debug):
                    print(movement[0], end='\r\n')
                    sys.stdout.flush()
                movement = movement.lower()

            if movement == ("w"):
                # Latch Forward TARGET - the ramp section below walks current_speed up to it.
                target_direction = 1
                target_speed = speed
                watchdog_active = False
                last_valid_input_time = time.monotonic()

            elif movement == ("s"):
                # Latch Reverse TARGET - the ramp section below walks current_speed up to it.
                target_direction = -1
                target_speed = speed
                watchdog_active = False
                last_valid_input_time = time.monotonic()

            elif movement == ("a"):
                # Steer only - MUST NOT modify motor speed/direction. Renews the watchdog.
                # NOTE: A -> front_turn_right / D -> front_turn_left preserves the exact
                # mapping that passed the Phase 1 servo hardware test in servo_movement_loop.
                last_valid_input_time = time.monotonic()
                front_turn_right(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step=step_degrees)
                hold_straight(servo=servos[0], config=configs[0])
                hold_straight(servo=servos[2], config=configs[2])

            elif movement == ("d"):
                # Steer only - MUST NOT modify motor speed/direction. Renews the watchdog.
                last_valid_input_time = time.monotonic()
                front_turn_left(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step=step_degrees)
                hold_straight(servo=servos[0], config=configs[0])
                hold_straight(servo=servos[2], config=configs[2])

            elif movement == (" "):
                # Manual Stop: immediate motor zero + recenter steering. BYPASSES the ramp.
                drive_direction = 0
                target_direction = 0
                current_speed = 0.0
                target_speed = 0.0
                watchdog_active = False
                last_valid_input_time = time.monotonic()
                brake(roboclaw=roboclaw, speed=0, controller_address=controller_address, debug=debug)
                last_sent = (0, 0)
                servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
                if (debug):
                    print("BRAKING", end='\r\n')
                continue

            elif movement == ("c"):
                # Safe Exit: zero motors, recenter servos, break (terminal restored in finally).
                roboclaw.SpeedM1M2(address=controller_address, m1=0, m2=0)
                servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
                break

            elif movement != "":
                print("C to break.", end='\r\n')

            else:
                # No input this tick: Control-Input Watchdog check.
                # Only acts while motion is commanded; steering-only sequences (W, A, A, D...)
                # stay alive because A/D renewed last_valid_input_time above.
                if ((target_speed > 0.0) or (current_speed > 0.0)) and ((time.monotonic() - last_valid_input_time) > watchdog_timeout):
                    # Watchdog expired: zero the TARGET; the shared ramp below does the decel.
                    target_speed = 0.0
                    watchdog_active = True

            # --- Speed Ramp Update (runs every tick, one shared path for accel/decel/reversal) ---
            if drive_direction != target_direction:
                # Direction change (or first launch from stop): decelerate through zero first.
                if current_speed > 0.0:
                    current_speed = max(0.0, current_speed - deceleration_step)
                if current_speed <= 0.0:
                    current_speed = 0.0
                    drive_direction = target_direction
            elif current_speed < target_speed:
                # Ramping up toward the latched target.
                current_speed = min(target_speed, current_speed + acceleration_step)
            elif current_speed > target_speed:
                # Ramping down (watchdog decel or lowered target).
                current_speed = max(0.0, current_speed - deceleration_step)

            # Fully stopped with no motion requested: clear directions so the watchdog idles.
            if (current_speed <= 0.0) and (target_speed <= 0.0):
                drive_direction = 0
                target_direction = 0
                if watchdog_active:
                    watchdog_active = False
                    if (debug):
                        print("WATCHDOG STOPPED", end='\r\n')

            # --- Command Output (only resend when the commanded value actually changed) ---
            command = (drive_direction, int(current_speed))
            if command != last_sent:
                if drive_direction == -1:
                    move_backward(roboclaw=roboclaw, speed=current_speed, controller_address=controller_address, debug=debug)
                else:
                    move_forward(roboclaw=roboclaw, speed=current_speed, controller_address=controller_address, debug=debug)
                last_sent = command
                if (debug):
                    if watchdog_active:
                        print(f"WATCHDOG DECEL: {int(current_speed)}", end='\r\n')
                    elif current_speed != target_speed or drive_direction != target_direction:
                        print(f"SPEED RAMP: {int(current_speed * drive_direction)}", end='\r\n')

    finally:
        # Belt-and-suspenders: never leave the loop with motors commanded.
        try:
            roboclaw.SpeedM1M2(address=controller_address, m1=0, m2=0)
        except Exception as error:
            print(f"Failed to Zero Motors on Exit: {str(error)}", end='\r\n')
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)
