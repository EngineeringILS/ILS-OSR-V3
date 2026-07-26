"""
TAMU Lunabotics 2026 Phase 1 Integrated Motion Control Harness
(Roboclaw 2x15A Propulsion + PCA9685 Front-Steer Servos)
"""
import motion_methods as methods

from servo_types import ServoConfig
from basicmicro import Basicmicro as Roboclaw
from adafruit_servokit import ServoKit
from adafruit_motor.servo import Servo

from config import i2c_address, channel_count, pwm_frequency, servo_number
# NOTE: Add these to config.py in the same style as the servo entries above:
# serial_port = "/dev/ttyS0" (or /dev/ttywwM0 etc.), baud_rate = 38400, controller_address = 0x80, motor_speed = <safe QPPS>
from config import serial_port, baud_rate, controller_address, motor_speed

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
zeroes = [177, 123, 155, 84]
step_degrees = 3.0
servodriver = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)
roboclaw = Roboclaw(serial_port, baud_rate)

# Watchdog / Control-Loop Configs (same naming scheme, tune on hardware):
watchdog_timeout = 1.5   # Seconds without any valid W/A/S/D before decel begins.
control_period = 0.05       # Loop tick period (select() timeout), 20 Hz.
deceleration_step = 10.0     # 0.0 --> auto: ramp full speed to zero over ~1 second.
acceleration_step = 10.0
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

def scuffed_servo_only_test():
    # Old Phase 1 steering-only harness (passed on hardware) - kept for modular fallback.
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
    for servo in turning_servos:
        servo.actuation_range = 300

    methods.servo_movement_loop(servodriver=servodriver, servos=turning_servos, configs=servo_configs, step_degrees=4, debug=True)

def scuffed_roboclaw_only_test():
    # Old tank-drive motor-only harness - kept for modular fallback.
    methods.roboclaw_setup(roboclaw=roboclaw, serial_port=serial_port, baud_rate=baud_rate, controller_address=controller_address)
    methods.roboclaw_movement_loop(roboclaw=roboclaw, speed=motor_speed, controller_address=controller_address, debug=True)

def main():
    # broke_mans_calibration()
    # scuffed_servo_only_test()
    # scuffed_roboclaw_only_test()

    # Servo Driver Bringup:
    methods.servodriver_setzeroes(servodriver=servodriver, s0_pos=zeroes[0], s1_pos=zeroes[1], s2_pos=zeroes[2], s3_pos=zeroes[3])
    for servo in turning_servos:
        servo.actuation_range = 300

    # Roboclaw Bringup:
    roboclaw_ready = methods.roboclaw_setup(roboclaw=roboclaw, serial_port=serial_port, baud_rate=baud_rate, controller_address=controller_address)
    if not roboclaw_ready:
        print("Roboclaw Setup Failed - Aborting Integrated Loop.")
        return

    # Phase 1 Integrated Drive + Steer Loop with Control-Input Watchdog:
    methods.integrated_movement_loop(
        roboclaw=roboclaw,
        speed=motor_speed,
        controller_address=controller_address,
        servodriver=servodriver,
        servos=turning_servos,
        configs=servo_configs,
        step_degrees=4,
        debug=True,
        watchdog_timeout=watchdog_timeout,
        control_period=control_period,
        acceleration_step=acceleration_step,
        deceleration_step=deceleration_step,
    )


if __name__ == "__main__":
    main()
