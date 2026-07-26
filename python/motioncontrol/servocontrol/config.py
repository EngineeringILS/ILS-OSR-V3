"""
Configuration for the PCA9685 Servo Driver (DO NOT COMMIT)
"""

# Calibration Values:
zeroes : list[int] = [177, 123, 31, 84]

# Parameters: DETERMINE USING CALIBRATION SCRIPT - THESE VALUES WERE OURS USED FOR TESTING.
# [min, zero, max] 
s0_params : list[int] = [52, 177, 300]
s1_params : list[int] = [0, 123, 180]
s2_params : list[int] = [30, 153, 278]
s3_params : list[int] = [20, 84, 215]

step_degrees = 3

# Static Definitions - Migrate to Configuration Later:
i2c_address = 0x40
channel_count = 16
pwm_frequency = 50

