"""
Calibration Script for PCA9685 Servo Driver specifically for Gobilda 300 Degree Servos, connected to the ILS-OSR-V3 Platform.
"""
import roboclaw_methods as methods
from roboclaw_types import RoboclawUnit, Motor
import typing

from config import serial_ports, baud_rate, controller_addreses

FRONT_LEFT   = Motor(index=0, channel=0, id=0, name="Front Left")
FRONT_RIGHT  = Motor(index=0, channel=1, id=1, name="Front Right") 
MIDDLE_LEFT  = Motor(index=1, channel=0, id=2, name="Middle Left")
MIDDLE_RIGHT = Motor(index=2, channel=0, id=3, name="Middle Right") 
BACK_LEFT    = Motor(index=1, channel=1, id=4, name="Back Left")
BACK_RIGHT   = Motor(index=2, channel=1, id=5, name="Back Right") 

speed = 100
motors : list[Motor] = [FRONT_LEFT, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT]

def calibration():
    roboclaws: list[RoboclawUnit] = []
    roboclaws = methods.roboclaw_setup(candidate_ports=serial_ports, expected_addresses=controller_addreses, names = ["Front", "Left", "Right"],baud_rate=baud_rate, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=0, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=1, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=2, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=3, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=4, speed=speed, debug=True)
    methods.testing_motor_movement_loop(roboclaws=roboclaws, motors=motors, selected_motor=5, speed=speed, debug=True)

if __name__ == "__main__": 
    calibration()
