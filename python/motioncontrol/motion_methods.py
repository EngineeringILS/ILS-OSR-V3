"""
motion_methods.py
Integrated Methods for Setting up and Managing the Roboclaw MCU + PCA9685 Servo Driver.
Combines roboclaw_methods.py and servo_methods.py, adding the Phase 1 Integrated char_input Loop
with a control-input watchdog (gradual dead-man decel) and symmetric accel/decel speed ramping.
"""

''' System Imports: '''
import time
import sys
import tty
import termios
import select

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

"""
    CASES (movement_select): 
    0 = STOPPED
    1 = FORWARD
    2 = BACKWARD
    3 = LEFT FORWARD, RIGHT BACKWARD
    4 = LEFT BACKWARD, RIGHT FORWARD
    OTHER = STOPPED
"""
def linear_motor_movement(
    servos: list[Servo],
    configs: list[ServoConfig],
    roboclaws: list[RoboclawUnit],
    motors: list[Motor],
    speed: int,
    movement_select: int,
    stop_counter: int,
):  
    # SAFETY STOP, ROBOT WILL STOP IF IT COUNTS DOWN TO ZERO!

    servo_methods.hold_angle(servo=servos[0], config=configs[0])
    servo_methods.hold_angle(servo=servos[1], config=configs[1])
    servo_methods.hold_angle(servo=servos[2], config=configs[2])
    servo_methods.hold_angle(servo=servos[3], config=configs[3])

   
    if (stop_counter <= 0 or movement_select ==0):
        macro_set_motor_speed(motors=motors,speed=0)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    if (movement_select == 1):
        macro_set_motor_speed(motors=motors, speed=speed)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    if (movement_select == 2):
        macro_set_motor_speed(motors=motors, speed=-speed)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return

    # if (movement_select == 3):
    #     # PROJECT SPECIFIC!!!
    #     # Max Safe Turn Speed, hardcoded for convenience (relevant to OSR V3's YellowJacket Motors)
    #     if (speed > 1000):
    #         speed = 1000
    #     macro_set_motor_speed()
    #     return 
    
    # if (movement_select == 4):
    #     # PROJECT SPECIFIC!!!
    #     # Max Safe Turn Speed, hardcoded for convenience (relevant to OSR V3's YellowJacket Motors)
    #     if (speed > 1000):
    #         speed = 1000
    #     macro_set_motor_speed()
    #     return 
    # KEEP MOVING:
    if (stop_counter > 0):
        return
    
    else:
        macro_set_motor_speed(motors=motors,speed=0)
        roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
        return
    
    return




def motion_movement_loop(servodriver : ServoKit, servos: list[Servo], configs: list[ServoConfig] , step_degrees: int, 
                         roboclaws: list[RoboclawUnit], motors: list[Motor], speeds: list[int] = [0], stop_counts: int = 0,
                         debug: bool = False
                         ):

    # Set terminal to raw mode for immediate single-character input
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    speed = 0
    speed_counter = 0
    stop_counter = 0
    try:
        while True:
            char_input = None
            speed = speeds[speed_counter]

            # Dead Mans STOP:
            linear_motor_movement(servos=servos, configs=configs, roboclaws=roboclaws, motors=motors, speed=speed,  movement_select=0, stop_counter=stop_counter)
            if select.select([sys.stdin], [], [], 0)[0]:
                char_input = sys.stdin.read(1).lower()

            if debug:
                print(char_input, end="\r\n")
                sys.stdout.flush()

            # Handle Speed Shifting:
            if char_input == "r":
                if speed_counter > 0:
                    speed_counter -= 1
                else:
                    speed_counter = 0
            
            if char_input == "t":
                if speed_counter < len(speeds) - 1:
                    speed_counter += 1
                    
            if char_input == "a":
                servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_up = True)
                #front_turn_right(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            if char_input == "q":
                servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_down=True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                
            elif char_input == "d":
                servo_methods.turn_servos(active_servos=[servos[1], servos[2]], active_configs=[configs[1], configs[2]], step=step_degrees, step_down=True)
                #front_turn_left(leftservo=servos[3], leftconfig=configs[3], rightservo=servos[1], rightconfig=configs[1], step = step_degrees)
                # hold_straight(servo=servos[0], config=configs[0])
                # hold_straight(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            elif char_input == "e":
                servo_methods.turn_servos(active_servos=[servos[0], servos[3]], active_configs=[configs[0], configs[3]], step=step_degrees, step_up = True)
                # hold_straight(servo=servos[1], config=configs[1])
                # hold_straight(servo=servos[3], config=configs[3])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])

            if char_input == "w":
                linear_motor_movement(servos=servos, configs=configs, roboclaws=roboclaws, motors=motors, speed=speed, movement_select=1, stop_counter=stop_counter)
                stop_counter = stop_counts
                
            elif char_input == "s":
                linear_motor_movement(servos=servos, configs=configs, roboclaws=roboclaws, motors=motors, speed=speed, movement_select=2, stop_counter=stop_counter)
                stop_counter = stop_counts

            elif char_input == " ":
                macro_set_motor_speed(motors=motors, speed=0)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.hold_angle(servo=servos[0], config=configs[0])
                servo_methods.hold_angle(servo=servos[1], config=configs[1])
                servo_methods.hold_angle(servo=servos[2], config=configs[2])
                servo_methods.hold_angle(servo=servos[3], config=configs[3])

            elif char_input == "b":
                servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)

            elif char_input == "c":
                macro_set_motor_speed(motors=motors, speed=0)
                roboclaw_methods.move_motors(roboclaws=roboclaws, motors=motors)
                servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
                break

            if stop_counts != 0 and stop_counts > 0:
                stop_counts -= 1
            elif stop_counts < 0:
                stop_counts = 0


            
    finally:
        try:
            for motor in motors:
                motor.speed = 0
            roboclaw_methods.move_motors(motors=motors, roboclaws=roboclaws)
            servo_methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=configs[0].straight, s1_pos=configs[1].straight, s2_pos=configs[2].straight, s3_pos=configs[3].straight)
        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                org_term_settings
            )
    return

