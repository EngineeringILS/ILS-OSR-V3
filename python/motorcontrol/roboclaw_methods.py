"""
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw

# Fixing Function Input Types:
import typing

# For Active Input:
import sys
import tty
import termios

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
    print(f"Attempting to Connect to Roboclaw.")\
    # Attempt Connection:
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
            elif movement == ("c"):
                break
            else:
                print ("C to break.", end='\r\n')
                pass
    
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)

""" Basic Movement Functions, Separated for Future Integration, e.g. More Specific Checking/Differentials for different modes."""
def move_forward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed")
    return

def move_backward(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=-speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed")
    return

def rotate_right(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool) -> None: 
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=-speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed")
    return

def rotate_left(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)-> None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=-speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed")
    return