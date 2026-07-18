"""
TAMU Lunabotics 2026 Phase 1 Roboclaw 2x15A Motor Control Harness
"""
import servo_methods as methods
from adafruit_servokit import ServoKit
from config import i2c_address, channel_count, pwm_frequency, servo_number

def main():
    servo = ServoKit(channels=channel_count, address=i2c_address, frequency=pwm_frequency)
    methods.servodriver_setup(servo=servo, pwm_frequency=pwm_frequency, i2c_address=i2c_address)
    methods.servo_movement_loop(servo=servo, step_degrees=1.0, channel=servo_number, debug=True)
if __name__ == "__main__":
    main()
