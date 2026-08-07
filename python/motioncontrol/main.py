# """
# TAMU Lunabotics 2026 Phase 1 Integrated Motion Control Harness
# (Roboclaw 2x15A Propulsion + PCA9685 Front-Steer Servos)
# """


from basicmicro import Basicmicro as Roboclaw
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

''' Servo Method Imports: '''
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

import servocontrol.servo_methods as servo_methods
from   servocontrol.servo_types import ServoConfig

from servocontrol.servo_config import i2c_address, channel_count, pwm_frequency, zeroes, step_degrees
from servocontrol.servo_config import s0_params, s1_params, s2_params, s3_params

'''Motor Method Imports: '''
import motorcontrol.roboclaw_methods as roboclaw_methods
from motorcontrol.roboclaw_types import RoboclawUnit, Motor
import typing
from typing import Tuple

from motorcontrol.roboclaw_config import serial_ports, baud_rate, controller_addreses

'''Unified Imports: '''
import new_motion_methods as methods


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

speed = 1000
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

# Configuration:
servodriver = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)

servo_configs : list[ServoConfig] = [
    ServoConfig(channel=0, min_safe=s0_params[0],  straight=s0_params[1], max_safe=s0_params[2], name="Back Right"),
    ServoConfig(channel=1, min_safe=s1_params[0],  straight=s1_params[1], max_safe=s1_params[2], name="Front Right"),
    ServoConfig(channel=2, min_safe=s2_params[0],  straight=s2_params[1], max_safe=s2_params[2], name="Front Left"),
    ServoConfig(channel=3, min_safe=s3_params[0],  straight=s3_params[1], max_safe=s3_params[2], name="Back Left"),
    ]

turning_servos : list[Servo] = [servodriver.servo[0], servodriver.servo[1], servodriver.servo[2], servodriver.servo[3]]


def main():
    roboclaws : list[RoboclawUnit]
    servodriver: ServoKit
    roboclaws, servodriver = methods.unified_bringup(candidate_ports=serial_ports,expected_addresses=controller_addreses, roboclaw_names=["Front", "Left", "Right"], motor_configs=motor_configs, baud_rate=baud_rate, 
                            channels=channel_count, pwm_frequency=pwm_frequency, i2c_address=i2c_address, 
                            debug=True)
    for servo in turning_servos:
        servo.actuation_range = 300

    methods.motion_movement_loop(servodriver=servodriver, servos=turning_servos, configs=servo_configs, step_degrees=step_degrees, roboclaws=roboclaws, motors=motors, speed=speed, debug=True)










# # Global Configs
# zeroes = [177, 123, 155, 84]
# step_degrees = 3.0
# servodriver = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)
# roboclaw = Roboclaw(serial_port, baud_rate)

# # Watchdog / Control-Loop Configs (same naming scheme, tune on hardware):
# watchdog_timeout = 1.5   # Seconds without any valid W/A/S/D before decel begins.
# control_period = 0.05       # Loop tick period (select() timeout), 20 Hz.
# deceleration_step = 10.0     # 0.0 --> auto: ramp full speed to zero over ~1 second.
# acceleration_step = 10.0



# turning_servos : list[Servo] = [servodriver.servo[0], servodriver.servo[1], servodriver.servo[2], servodriver.servo[3]]


# def main():
#     # Servo Driver Bringup:
#     methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
#     for servo in turning_servos:
#         servo.actuation_range = 300

#     # Roboclaw Bringup:
#     roboclaw_ready = methods.roboclaw_setup(roboclaw=roboclaw, serial_port=serial_port, baud_rate=baud_rate, controller_address=controller_address)
#     if not roboclaw_ready:
#         print("Roboclaw Setup Failed - Aborting Integrated Loop.")
#         return

#     # Phase 1 Integrated Drive + Steer Loop with Control-Input Watchdog:
#     methods.integrated_movement_loop(
#         roboclaw=roboclaw,
#         speed=motor_speed,
#         controller_address=controller_address,
#         servodriver=servodriver,
#         servos=turning_servos,
#         configs=servo_configs,
#         step_degrees=4,
#         debug=True,
#         watchdog_timeout=watchdog_timeout,
#         control_period=control_period,
#         acceleration_step=acceleration_step,
#         deceleration_step=deceleration_step,
#     )


if __name__ == "__main__":
    main()
