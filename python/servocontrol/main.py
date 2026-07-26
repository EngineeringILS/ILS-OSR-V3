"""
TAMU Lunabotics 2026 Phase 1 Roboclaw 2x15A Motor Control Harness
"""
import servo_methods as methods
from servo_methods import ServoConfig

from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

from config import i2c_address, channel_count, pwm_frequency, servo_number

# Servo0 | Zeropoint ~~179 ~~177
# Servo1 | Zeropoint ~~124 ~~123
# Servo2 | Zeropoint ~~155 ~~153
# Servo3 | Zeropoint ~~084

# Servo# | [Max Safe, Straight, Min Safe] | Comment:
# Servo0 | [052, 177, 300] | 052/300 Are Front/Back Perpindicular to Straight
# Servo1 | [000, 123, 180] | 000 is Perpindicular to Straight, 180 is the maixmum safe interior turn
# Servo2 | [030, 153, 278] | 052/300 Are Front/Back Perpindicular to Straight
# Servo3 | [020, 084, 215] | 020 is the maixmum safe interior turn, 215 is Perpindicular to Straight, 

# Global Configs
zeroes = [177, 123, 31, 84]
step_degrees = 3.0
servodriver = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)

servo_configs : list[ServoConfig] = [
    ServoConfig(channel=0, min_safe=52, straight=177, max_safe=300, name="Back Right"),
    ServoConfig(channel=1, min_safe=0,  straight=123, max_safe=180, name="Front Right"),
    ServoConfig(channel=2, min_safe=30, straight=153, max_safe=278, name="Back Left"),
    ServoConfig(channel=3, min_safe=20, straight=84,  max_safe=215, name="Front Left")
    ]

turning_servos : list[Servo] = [servodriver.servo[0], servodriver.servo[1], servodriver.servo[2], servodriver.servo[3]]

def broke_mans_calibration():
    methods.servodriver_setup(servo=servodriver, pwm_frequency=pwm_frequency, i2c_address=i2c_address)
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=zeroes[0], channel=0, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=zeroes[1], channel=1, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=zeroes[2], channel=2, debug=True)
    methods.testing_servo_movement_loop(servo=servodriver, step_degrees=step_degrees, max_angle=300, min_angle=0, set_start_point=zeroes[3], channel=3, debug=True)
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])

def main():
    for servo in turning_servos:
            servo.actuation_range = 300
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
    broke_mans_calibration()
    

    # methods.servo_movement_loop(servodriver=servodriver, servos=turning_servos, configs=servo_configs, step_degrees=4, debug=True)



if __name__ == "__main__":
    main()
