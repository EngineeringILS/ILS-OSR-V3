"""
Configuration for the Roboclaw MCU (DO NOT COMMIT)
"""
from roboclaw_types import SpeedConfig
# Static Definitions - Migrate to Configuration Later:
serial_port = "/dev/tty/ACM0"
baud_rate = 38400
controller_address = 0x80

# Vestigial, powers old logic.
min_speed = 1000
max_speed = 5000

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
