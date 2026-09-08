"""
Configuration for the PCA9685 Servo Driver (DO NOT COMMIT)
"""

# Calibration Values:
zeroes : list[int] = [90, 180, 80, 165]

# Parameters: DETERMINE USING CALIBRATION SCRIPT - THESE VALUES WERE OURS USED FOR TESTING.
# [min, zero, max] 
s0_params : list[int] = [0, 130, 210]
s1_params : list[int] = [130, 180, 300]
s2_params : list[int] = [0, 130, 195]
s3_params : list[int] = [45, 165, 300]

step_degrees = 3

# Static Definitions - Migrate to Configuration Later:
i2c_address = 0x40
channel_count = 16
pwm_frequency = 50

