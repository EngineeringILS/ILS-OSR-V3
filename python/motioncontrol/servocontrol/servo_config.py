"""
Configuration for the PCA9685 Servo Driver (DO NOT COMMIT)
"""

# Calibration Values:
zeroes : list[int] = [176, 122, 87, 164]

# Parameters: DETERMINE USING CALIBRATION SCRIPT - THESE VALUES WERE OURS USED FOR TESTING.
# [min, zero, max] 
s0_params : list[int] = [0, 176, 300]
s1_params : list[int] = [0, 122, 185]
s2_params : list[int] = [18, 87, 216]
s3_params : list[int] = [0, 164, 300]

step_degrees = 3

# Static Definitions - Migrate to Configuration Later:
i2c_address = 0x40
channel_count = 16
pwm_frequency = 50

