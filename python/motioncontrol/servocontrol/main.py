"""
TAMU Lunabotics 2026 Phase 1 Roboclaw 2x15A Motor Control Harness
"""
import servo_methods as methods
from servo_methods import ServoConfig

from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

from servo_config import i2c_address, channel_count, pwm_frequency, zeroes, step_degrees
from servo_config import s0_params, s1_params, s2_params, s3_params

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
    for servo in turning_servos:
            servo.actuation_range = 300
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
    methods.servo_movement_loop(servodriver=servodriver, servos=turning_servos, configs=servo_configs, step_degrees=4, debug=True)


if __name__ == "__main__":
    main()
