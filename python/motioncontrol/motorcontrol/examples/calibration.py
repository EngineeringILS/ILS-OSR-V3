"""
Calibration Script for Roboclaw MCU's for the ILS-OSR-V3 Platforms. Sequentially Drives Each Motor to ensure configuration remains in the correct ordering.
"""


import sys
import pathlib
# Relative python path objectively sucks - this language sucks - this never is a problem with Cmake... 
# This finds the directory of the current script (magic code, user tests work easily)
current_dir = pathlib.Path(__file__).parent 
sys.path.append(str(current_dir))
sys.path.append(str(current_dir.parent))

import roboclaw_methods as methods
from roboclaw_types import RoboclawUnit, Motor
import typing
from typing import Tuple

from roboclaw_config import serial_ports, baud_rate, controller_addreses

'''
Static Definitions and Configuration:
'''
FRONT_LEFT   = Motor(index=0, channel=0, id=0, name="Front Left", encoder_reversed=True)
FRONT_RIGHT  = Motor(index=1, channel=0, id=1, name="Front Right", encoder_reversed=False) 
MIDDLE_LEFT  = Motor(index=0, channel=1, id=2, name="Middle Left", encoder_reversed=True)
MIDDLE_RIGHT = Motor(index=1, channel=1, id=3, name="Middle Right", encoder_reversed=False) 
BACK_LEFT    = Motor(index=2, channel=1, id=4, name="Back Left", encoder_reversed=True)
BACK_RIGHT   = Motor(index=2, channel=0, id=5, name="Back Right", encoder_reversed=False) 

FRONT_LEFT_QPPS   = 3200
FRONT_RIGHT_QPPS  = 3200
MIDDLE_LEFT_QPPS  = 3200
MIDDLE_RIGHT_QPPS = 3200
BACK_LEFT_QPPS    = 3200
BACK_RIGHT_QPPS   = 3200

speed = 100
motor_configs = [(FRONT_LEFT,   FRONT_LEFT_QPPS), 
                 (FRONT_RIGHT,  FRONT_RIGHT_QPPS),
                 (MIDDLE_LEFT,  MIDDLE_LEFT_QPPS),
                 (MIDDLE_RIGHT, MIDDLE_RIGHT_QPPS),
                 (BACK_LEFT,    BACK_LEFT_QPPS),
                 (BACK_RIGHT,   BACK_RIGHT_QPPS),]

motor_configs : list[Tuple[Motor, int]]


motors = [FRONT_LEFT,
          FRONT_RIGHT, 
          MIDDLE_LEFT, 
          MIDDLE_RIGHT, 
          BACK_LEFT, 
          BACK_RIGHT]

motors : list[Motor]

'''
END Static Defintions and Configuration:
'''

def calibration():
    roboclaws: list[RoboclawUnit] = []
    roboclaws = methods.roboclaw_setup(candidate_ports=serial_ports, expected_addresses=controller_addreses, names = ["Front", "Left", "Right"],baud_rate=baud_rate, debug=True)
    
    methods.motor_setup(motor_configs=motor_configs, roboclaws=roboclaws, debug=True, readonly=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=0, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=1, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=2, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=3, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=4, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=5, speed=speed, debug=True)

if __name__ == "__main__": 
    calibration()