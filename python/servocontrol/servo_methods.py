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
    print(f"Attempting to Connect to Servo Driver.")\
    # Attempt Connection:
    try:
        servo.servo[0].angle = None
        print(f"Connected to PCA9685 servo driver at I2C address {i2c_address:#04x}.")
        return True

    except Exception as error:
        print(f"Failed to connect to PCA9685 servo driver at I2C address {i2c_address:#04x}.")
        print(str(error))
        return False


# def roboclaw_movement_loop(roboclaw : Roboclaw, speed: float, controller_address: float, debug: bool):
#     """
#     Basic Tank Drive Motor Control Loop (WASD)

#     :param roboclaw: The Roboclaw MCU Object (Basicmicro Alias)
#     :type roboclaw: Roboclaw
#     :param speed: The desired set speed for the connected Motors.
#     :type speed: float
#     :param debug: Enable Debug Printouts in the Motor Control Loop itself.
#     :type debug: bool
#     """
#     # Set Termios Raw Nonblocking Char input:
#     fd = sys.stdin.fileno()
#     org_term_settings = termios.tcgetattr(fd)

#     movement_loop = True
#     try:
#         if (debug):
#             print("Beginning Movement Loop (C/c to Cancel): ")
#         tty.setraw(fd)
#         while (movement_loop):
#             movement = sys.stdin.read(1)
#             if (debug): 
#                 print(movement[0], end='\r\n')
#                 sys.stdin.flush()
#             movement = movement.lower()
#             if movement == ("w"):
#                 # Move Forward
#                 move_forward(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
#             elif movement == ("a"):
#                 # Rotate Right
#                 rotate_right(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
#             elif movement == ("s"):
#                 # Move Backward
#                 move_backward(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
#             elif movement == ("d"):
#                 # Rotate Left
#                 rotate_left(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
#             elif movement == (" "):
#                 brake(roboclaw=roboclaw, speed=0, controller_address=controller_address, debug=debug)
#                 if (debug):
#                     print("BRAKING", end='\r\n')
#             elif movement == ("c"):
#                 roboclaw.SpeedM1M2(address=controller_address, m1=0, m2=0)
#                 break
#             else:
#                 print ("C to break.", end='\r\n')
#                 pass
    
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)

# """ Basic Movement Functions, Separated for Future Integration, e.g. More Specific Checking/Differentials for different modes."""
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
