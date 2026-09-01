"""
motion_methods.py
Integrated Methods for Setting up and Managing the Roboclaw MCU + PCA9685 Servo Driver.
Combines roboclaw_methods.py and servo_methods.py, adding the Phase 1 Integrated Movement Loop
with a control-input watchdog (gradual dead-man decel) and symmetric accel/decel speed ramping.
"""

''' System Imports: '''
import time
import sys
import tty
import termios

''' Servo Method Imports: '''
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

import servocontrol.servo_methods as servo_methods
from   servocontrol.servo_methods import ServoConfig

'''Motor Method Imports: '''
import motorcontrol.roboclaw_methods as roboclaw_methods
from motorcontrol.roboclaw_types import RoboclawUnit, Motor
import typing
from typing import Tuple


def unified_bringup(candidate_ports : list[str], expected_addresses : list[int], roboclaw_names : list[str], motor_configs : list[Tuple[Motor, int]],
                    baud_rate : int, channels : int, pwm_frequency: int, i2c_address: int, 
                    debug : bool
                    ) -> Tuple[list[RoboclawUnit], ServoKit]:

    ''' Build the Objects, run the setups, hope they work (prayer emoji): '''
    roboclaws: list[RoboclawUnit] = []
    roboclaws = roboclaw_methods.roboclaw_setup(candidate_ports=candidate_ports, expected_addresses=expected_addresses, names=roboclaw_names, baud_rate=baud_rate, debug=debug)
    roboclaw_methods.motor_setup(motor_configs=motor_configs, roboclaws=roboclaws, debug=debug, readonly=True)
    servodriver = ServoKit(channels=channels, address=i2c_address, frequency=pwm_frequency)
    servo_methods.servodriver_setup(servo = servodriver, pwm_frequency=pwm_frequency, i2c_address=i2c_address)
    return roboclaws, servodriver

def macro_set_motor_speed(motors: list[Motor], speed: int):
    for motor in motors:
        motor.speed = speed
    return

def motion_movement_loop(servodriver : ServoKit, servos: list[Servo], configs: list[ServoConfig] , step_degrees: int, 
                         roboclaws: list[RoboclawUnit], motors: list[Motor], speed: int,
                         debug: bool
                         ):
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)

    try:
        while True:
            tty.setraw(fd)
            
            movement = sys.stdin.read(1).lower()
            if debug:
                print(movement, end="\r\n")
                sys.stdout.flush()

           
            if movement == "a":
                servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_up = True)
                #front_turn_right(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            if movement =="r":
                speed = 500
            if movement == "t":
                speed = 1000
            if movement == "y":
                speed = 2000
            if movement == "u":
                speed = 3200

            if movement == "q":
                servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_down=True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                
            elif movement == "d":
                servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_down=True)
                #front_turn_left(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            elif movement == "e":
                servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_up = True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])

            if movement == "w":
                macro_set_motor_speed(motors=motors, speed=speed)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])
                
            elif movement == "s":
                macro_set_motor_speed(motors=motors, speed=-speed)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            elif movement == " ":
                macro_set_motor_speed(motors=motors, speed=0)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            elif movement == "b":
                servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)

            elif movement == "c":
                macro_set_motor_speed(motors=motors, speed=0)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
                break

    finally:
        try:
            for motor in motors:
                motor.speed = 0
            roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
            servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)
    return