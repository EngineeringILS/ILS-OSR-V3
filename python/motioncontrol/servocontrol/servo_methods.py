"""
servo_methods.py
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
# Roboclaw Base Library
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

# FSM State Curves and Control:
# Think ECEN-248 + ECEN-214 Type Logic:
# from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
# from servo_types import MotorCurveLUT, populate_curve_lut, MovementState, MovementSubState, SpeedConfig, MotorCurveLUTConfig, MotorControllerState
# Fixing Function Input Types:
import typing
from typing import Any
from .servo_types import ServoConfig

# For Active Input:
import sys
import tty
import termios


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
                turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_up = True)
                #front_turn_right(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                hold_angle(servo=servos[0], config=configs[0])
                hold_angle(servo=servos[3], config=configs[3])

            if movement == "q":
                turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_down=True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                hold_angle(servo=servos[1], config=configs[1])
                hold_angle(servo=servos[2], config=configs[2])
                
            elif movement == "d":
                turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_down=True)
                #front_turn_left(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                hold_angle(servo=servos[0], config=configs[0])
                hold_angle(servo=servos[3], config=configs[3])

            elif movement == "e":
                turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_up = True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                hold_angle(servo=servos[1], config=configs[1])
                hold_angle(servo=servos[2], config=configs[2])

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
## ONLY USE WITH GOBILDA SERVOS:

def servodriver_setzeroes(servodriver: ServoKit, s0_pos : float, s1_pos : float, s2_pos : float, s3_pos : float):
    servos =  [servodriver.servo[0], servodriver.servo[1], servodriver.servo[2], servodriver.servo[3]]
    for servo in servos:
        servo.actuation_range = 300

    # Pylint sucks:
    s0_pos_ : Any = s0_pos
    s1_pos_ : Any = s1_pos
    s2_pos_ : Any = s2_pos
    s3_pos_ : Any = s3_pos
    servos[0].angle = s0_pos_
    servos[1].angle = s1_pos_
    servos[2].angle = s2_pos_
    servos[3].angle = s3_pos_

    i = 0
    for servo in servos:
        print(f"Servo: {i} | Position: {servo.angle}")
        i += 1

def hold_straight(servo : Servo, config : ServoConfig):
    # Shut up Pylint:
    angle : Any = config.straight
    servo.angle = angle
    return

def hold_angle(servo: Servo, config: ServoConfig):
    angle : Any = servo.angle
    servo.angle = angle
    return

def turn_servos(active_servos : list[Servo], active_configs : list[ServoConfig], step : float = 0.0, step_up : bool = False, step_down : bool = False):
    # Add error handling later for stepping up and down. 
    angles : list[float] = []
    # TODO: Load in angles, from left to right according to the order of inputted servos and their corresponding configurations.
    # If the angles are incorrect, ensure that active_servos and active_configs have the same order of servos.
    for i in range(len(active_servos)):
        servo, config = active_servos[i], active_configs[i]
        # If there is an existing angle, load it:
        if servo.angle is not None :
            angles.append(servo.angle)
        # Else, default to straight
        else:
            angles.append(config.straight)

    # Step the Angles:
    for i in range(len(active_servos)):
        config = active_configs[i]
        angle = angles[i]
        if (step_up):
             angle = config.step_up(angle, step)
        elif (step_down):
            angle = config.step_down(angle, step)
        else:
            angle = angle
        angles[i] = angle

    # Write the angles:
    for i in range(len(active_servos)):
        servo = active_servos[i]
        angle = angles[i]
        if angle is not None:
            angle : Any = angle
            servo.angle = angle

    return

def hold_servos(active_servos : list[Servo], active_configs : list[ServoConfig]):
    angles : list[float] = []

    # Read and append angles:
    for i in range(len(active_servos)):
        servo, config = active_servos[i], active_configs[i]
        if servo.angle is not None:
            angles.append(servo.angle)
        else:
            angles.append(config.straight)

    # Write the angles:
    for i in range(len(active_servos)):
        servo, angle = active_servos[i], angles[i]
        angle : Any = angles[i]
        servo.angle = angle

    return

def hold_zeroes(active_servos : list[Servo], active_configs : list[ServoConfig]):
    angles : list[float] = []

    # Read and append angles:
    for i in range(len(active_servos)):
        servo, config = active_servos[i], active_configs[i]
        angles.append(config.straight)
           

    # Write the angles:
    for i in range(len(active_servos)):
        servo, angle = active_servos[i], angles[i]
        angle : Any = angles[i]
        servo.angle = angle
        
    return


