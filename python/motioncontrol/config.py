"""
Configuration for the Roboclaw MCU (DO NOT COMMIT)
"""
from roboclaw_types import SpeedConfig
# Static Definitions - Migrate to Configuration Later:
serial_port = "/dev/ttyACM0"
baud_rate = 38400
controller_address = 0x80

# Vestigial, powers old logic.
min_speed = 1000
max_speed = 5000
motor_speed = 2000

# New Logic:
speed_config: SpeedConfig = SpeedConfig(
    max_forward = 3000,
    min_forward = 500,
    max_reverse = 2000,
    min_reverse = 500,
    turn = 1000,
    brake = 250,
    stopped = 0
)

"""
Configuration for the PCA9685 Servo Driver (DO NOT COMMIT)
"""

# Static Definitions - Migrate to Configuration Later:
i2c_address = 0x40
channel_count = 16
pwm_frequency = 50

# Testing Definitions:
servo_number = 1

min_angle = 0
max_angle = 180