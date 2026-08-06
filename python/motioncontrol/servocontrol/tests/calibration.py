"""
Calibration Script for PCA9685 Servo Driver specifically for Gobilda 300 Degree Servos, connected to the ILS-OSR-V3 Platform.
"""

import sys
import pathlib
# Relative python path objectively sucks - this language sucks - this never is a problem with Cmake... 
# This finds the directory of the current script (magic code, user tests work easily)
current_dir = pathlib.Path(__file__).parent 
sys.path.append(str(current_dir))
sys.path.append(str(current_dir.parent))

import servo_methods as methods
from servo_methods import ServoConfig

from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

from servo_config import i2c_address, channel_count, pwm_frequency
from calibration_config import s0_params, s1_params, s2_params, s3_params, step_degrees

servodriver = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)

def calibration():
    methods.servodriver_setup(servo=servodriver, pwm_frequency=pwm_frequency, i2c_address=i2c_address)
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=s0_params[1], s1_pos=s1_params[1], s2_pos=s2_params[1], s3_pos=s3_params[1])
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=s0_params[1], channel=0, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=s1_params[1], channel=1, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=s2_params[1], channel=2, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=s3_params[1], channel=3, debug=True)
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=s0_params[1], s1_pos=s1_params[1], s2_pos=s2_params[1], s3_pos=s3_params[1])


if __name__ == "__main__":
    calibration()
