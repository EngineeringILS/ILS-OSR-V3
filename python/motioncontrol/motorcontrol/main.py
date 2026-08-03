"""
TAMU Lunabotics 2026 Phase 1 Roboclaw 2x15A Motor Control Harness
"""
import roboclaw_methods as methods
from basicmicro import Basicmicro as Roboclaw
from roboclaw_types import RoboclawUnit, Motor
# from config import serial_port, baud_rate, controller_address



'''
STATIC CONFIGURATION SETUP:
'''

FRONT_LEFT   = Motor(index=0, channel=0, id=0, name="Front Left")
FRONT_RIGHT  = Motor(index=0, channel=1, id=1, name="Front Right") 
MIDDLE_LEFT  = Motor(index=1, channel=0, id=2, name="Middle Left")
MIDDLE_RIGHT = Motor(index=2, channel=0, id=3, name="Middle Right") 
BACK_LEFT    = Motor(index=1, channel=1, id=4, name="Back Left")
BACK_RIGHT   = Motor(index=2, channel=1, id=5, name="Back Right") 

motors : list[Motor] = [FRONT_LEFT, FRONT_RIGHT, MIDDLE_LEFT, MIDDLE_RIGHT, BACK_LEFT, BACK_RIGHT]

# def main():
#     roboclaw = Roboclaw(serial_port, baud_rate)
#     methods.roboclaw_setup(roboclaw=roboclaw, serial_port=serial_port, baud_rate=baud_rate, controller_address=controller_address)
#     methods.roboclaw_movement_loop(roboclaw=roboclaw, speed=0.5, controller_address=controller_address, debug=True)
# if __name__ == "__main__":
#     main()

