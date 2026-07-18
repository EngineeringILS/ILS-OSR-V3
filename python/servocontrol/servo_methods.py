"""
roboclaw_methods.py
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
# Roboclaw Base Library
from adafruit_servokit import ServoKit

# FSM State Curves and Control:
# Think ECEN-248 + ECEN-214 Type Logic:
# from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
# from servo_types import MotorCurveLUT, populate_curve_lut, MovementState, MovementSubState, SpeedConfig, MotorCurveLUTConfig, MotorControllerState
# Fixing Function Input Types:
import typing

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


def servo_movement_loop(servo: ServoKit, channel: int, step_degrees, debug: bool):
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
                current_angle = max(0, current_angle - step_degrees)
                selected_servo.angle = current_angle

            elif movement == "d":
                current_angle = min(180, current_angle + step_degrees)
                selected_servo.angle = current_angle

            elif movement == " ":
                current_angle = 90
                selected_servo.angle = current_angle
                if debug:
                    print("CENTERING", end="\r\n")

            elif movement == "c":
                selected_servo.angle = None
                break

            else:

                print("A/D to move, Space to center, C to break.", end="\r\n")
            if (debug):
                    print(f"SERVO ANGLE: {selected_servo.angle}")
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)

""" Basic Movement Functions, Separated for Future Integration, e.g. More Specific Checking/Differentials for different modes."""
# def move_forward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
#     speed = int(speed)
#     status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
#     if (not status) and debug:
#         print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
#     return

# def move_backward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
#     speed = int(speed)
#     status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=-speed)
#     if (not status) and debug:
#         print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
#     return

# def rotate_right(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None: 
#     speed = int(speed)
#     status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=-speed)
#     if (not status) and debug:
#         print(f"Roboclaw Set Speed on M1/M2 Failed" , end='\r\n')
#     return

# def rotate_left(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)-> None:
#     speed = int(speed)
#     status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=speed)
#     if (not status) and debug:
#         print(f"Roboclaw Set Speed on M1/M2 Failed", end='\r\n')
#     return

# def brake(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)->None:
#     speed = int(speed)
#     status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
#     if (not status) and debug:
#         print(f"Roboclaw Set Speed on M1/M2 Failed", end='\r\n')
#     return
