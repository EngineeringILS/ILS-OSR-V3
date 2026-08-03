"""
roboclaw_methods.py
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
import sys
import tty
import termios
import typing

# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw
from roboclaw_types import RoboclawUnit, MotorConfig

def roboclaw_setup(roboclaws: list[RoboclawUnit], addresses : list[int], baud_rate: int, debug : bool = False) -> bool:
    ordered_roboclaws = [None, None, None]
            
    for i in range(len(roboclaws)):
        if (debug):
            print(f"Setting up {roboclaws[i].name} with Port: {roboclaws[i].serial_port}, BAUD Rate: {baud_rate}, Varying Addresses:")
        roboclaw = Roboclaw(comport=roboclaws[i].serial_port, rate=baud_rate)
        
        for j in range(len(addresses)):
            address = addresses[j]
            firmware = roboclaw.ReadVersion(address=address)
            if (firmware[0]):
                if(debug):
                    print(f"Matched Roboclaw: {j}")
                ordered_roboclaws[i] = roboclaw
                return True
    




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
            elif movement == ("s"):
                # Move Backward
                move_backward(roboclaw=roboclaw, speed=speed, controller_address=controller_address, debug=debug)
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



def brake(roboclaw: Roboclaw, speed: float, controller_address: int, debug: bool)->None:
    speed = int(speed)
    status = roboclaw.SpeedM1M2(address=controller_address, m1=speed, m2=speed)
    if (not status) and debug:
        print(f"Roboclaw Set Speed on M1/M2 Failed", end='\r\n')
    return
