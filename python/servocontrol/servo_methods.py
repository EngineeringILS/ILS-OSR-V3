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

from servo_types import ServoConfig

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

