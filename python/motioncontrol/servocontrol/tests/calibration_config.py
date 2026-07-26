
"""
Sample Calibration Configuration for the PCA9685 Servo Driver specifically for Gobilda 300 Degree Servos.
"""

# Parameters: DETERMINE USING CALIBRATION SCRIPT - THESE VALUES WERE OURS USED FOR TESTING.
# [min, zero, max] 
s0_params : list[int] = [0, 150, 300]
s1_params : list[int] = [0, 150, 300]
s2_params : list[int] = [0, 150, 300]
s3_params : list[int] = [0, 150, 300]

step_degrees = 3
