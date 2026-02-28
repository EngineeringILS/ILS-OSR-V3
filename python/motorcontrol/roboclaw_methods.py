"""
roboclaw_methods.py
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw

# FSM State Curves and Control:
# Think ECEN-248 + ECEN-214 Type Logic:
from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
from roboclaw_types import MotorCurveLUT, populate_curve_lut, MovementState, MovementSubState, SpeedConfig, MotorCurveLUTConfig, MotorControllerState
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

class RoboclawControlLoop:
    """
    State-machine driven controller for the Roboclaw MCU with strict safety boundaries.
    """
    def __init__(self, serial_port: str, baud_rate: int, controller_address: int, speed_config: SpeedConfig, debug: bool):
        # Static Environmental Parameters:
        self.serial_port: str = serial_port
        self.baud_rate: int = baud_rate
        self.controller_address: int = controller_address
        self.debug: bool = debug

        # Populate LUTs
        self.curveLutConfig: MotorCurveLUTConfig = MotorCurveLUTConfig(
            # 5s @ 0.1 smooth ramp
            forward_accelerate_lut=MotorCurveLUT(speed_config.min_forward, speed_config.max_forward, 100, 0.1),
            # 2s @ 0.15 smooth decay
            forward_decelerate_lut=MotorCurveLUT(speed_config.min_forward, speed_config.max_forward, 40, 0.15),
            # 2.5s @ 0.1 smooth ramp (half the speed to cover)
            reverse_accelerate_lut=MotorCurveLUT(speed_config.min_reverse, speed_config.max_reverse, 50, 0.1),
            # 1s @ 0.15 smooth decay 
            reverse_decelerate_lut=MotorCurveLUT(speed_config.min_reverse, speed_config.max_reverse, 20, 0.15),
            # This is just flat, at whatever the turning speed is.
            turning_lut=MotorCurveLUT(speed_config.turn, speed_config.turn, 10, 1),
            # This decay aggressively takes whatever the current speed is, and damps it to zero in 1 second at a 0.5 decay.
            general_braking_lut=MotorCurveLUT(speed_config.brake, speed_config.max_forward, 20, 0.5),
            # This is just flat, at 0
            stopped_lut=MotorCurveLUT(0,0,10,1)
        )
        # Populate the LUTs.
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.forward_accelerate_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.forward_decelerate_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.reverse_accelerate_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.reverse_decelerate_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.turning_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.general_braking_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.stopped_lut)

        # Dynamic Roboclaw MCU Object, State Machine Driver:
        self.is_connected = False
        self.roboclaw = Roboclaw(self.serial_port, self.baud_rate)
        self.State: MotorControllerState = MotorControllerState()

    def connect(self) -> bool:
        """Opens the Roboclaw Serial Connection and verifies firmware."""
        print(f"Attempting to Connect to Roboclaw at {self.serial_port} (BAUD: {self.baud_rate}).")
        try:
            self.roboclaw.Open()
            firmware_version = self.roboclaw.ReadVersion(self.controller_address)
            if firmware_version[0]:
                print(f"Connected to Roboclaw. Firmware: {firmware_version[1]}.")
                self.is_connected = True
                # STOP immediately if for some reason we connect and the default state on the Roboclaw is moving. 
                self.roboclaw.SpeedM1M2(address=self.controller_address, 
                                        speed1=self.curveLutConfig.stopped_lut.array[0], 
                                        speed2=self.curveLutConfig.stopped_lut.array[0])
                self.is_connected = True
                return True
            else:
                print("Failed to connect: Firmware read failed.")
                return False
        except Exception as error:
            print(f"Error Connecting to Roboclaw: {str(error)}")
            return False