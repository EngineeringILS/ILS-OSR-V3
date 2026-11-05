"""
rover_test.py
Behavior tests for RoverDrive and BlockedMovement.
"""

import os
import time
import unittest
from typing import Tuple
import sys
import traceback
import csv

from datetime import datetime, timezone
from pathlib import Path

from blocked_movement import BlockedMovement
from datacollection import create_i2c_tiny_usb_bus
from datacollection.imu_wrapper import IMUWrapper, StateChange
from motorcontrol.roboclaw_config import baud_rate, controller_addreses, serial_ports
from motorcontrol.roboclaw_types import Motor
from new_motion_methods import MovementState, RotaryState, RoverDrive, SteerInput
from servocontrol.servo_config import channel_count, i2c_address, pwm_frequency
from servocontrol.servo_config import s0_params, s1_params, s2_params, s3_params
from servocontrol.servo_types import ServoConfig

'''
Temporary inline IMU helper method.
'''
def save_imu_samples(path: Path, change: StateChange, clock_offset_ns: int):
    """Write the samples of one movement to CSV, stamped in wall-clock time."""
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["log_ns", "ax_mps2", "ay_mps2", "az_mps2", "gx_radps", "gy_radps", "gz_radps"])
        for sample in change.samples:
            acceleration = sample.linear_acceleration
            rate = sample.angular_velocity
            writer.writerow([sample.header.stamp.to_nanoseconds() + clock_offset_ns,
                             acceleration.x, acceleration.y, acceleration.z,
                             rate.x, rate.y, rate.z])


'''
Static Definitions and Configuration:
'''
# Physical wiring and forward-command signs used by the embodied-control harness.
# Controller indices follow controller_addreses: RC1=0x80, RC2=0x81, RC3=0x82.
# channel=0 is M1; channel=1 is M2. encoder_reversed negates commands/readback.
FRONT_LEFT   = Motor(index=0, channel=0, id=0, name="Front Left",   encoder_reversed=False)
FRONT_RIGHT  = Motor(index=2, channel=0, id=1, name="Front Right",  encoder_reversed=True)
MIDDLE_LEFT  = Motor(index=0, channel=1, id=2, name="Middle Left",  encoder_reversed=True)
MIDDLE_RIGHT = Motor(index=2, channel=1, id=3, name="Middle Right", encoder_reversed=False)
BACK_LEFT    = Motor(index=1, channel=1, id=4, name="Back Left",    encoder_reversed=False)
BACK_RIGHT   = Motor(index=1, channel=0, id=5, name="Back Right",   encoder_reversed=True)

# Tumbler Configuration:
FRONT_LEFT_QPPS   = 3200
FRONT_RIGHT_QPPS  = 3200
MIDDLE_LEFT_QPPS  = 3200
MIDDLE_RIGHT_QPPS = 3200
BACK_LEFT_QPPS    = 3200
BACK_RIGHT_QPPS   = 3200

motor_configs = [(FRONT_LEFT,   FRONT_LEFT_QPPS), 
                 (FRONT_RIGHT,  FRONT_RIGHT_QPPS),
                 (MIDDLE_LEFT,  MIDDLE_LEFT_QPPS),
                 (MIDDLE_RIGHT, MIDDLE_RIGHT_QPPS),
                 (BACK_LEFT,    BACK_LEFT_QPPS),
                 (BACK_RIGHT,   BACK_RIGHT_QPPS),]

motor_configs : list[Tuple[Motor, int]]

"""
MOTORS:
FRONT_LEFT   --> 0
FRONT_RIGHT  --> 1
MIDDLE_LEFT  --> 2
MIDDLE_RIGHT --> 3
BACK_LEFT    --> 4
BACK_RIGHT   --> 5
"""
motors = [
    FRONT_LEFT,
    FRONT_RIGHT, 
    MIDDLE_LEFT, 
    MIDDLE_RIGHT, 
    BACK_LEFT, 
    BACK_RIGHT
    ]

motors : list[Motor] # Pylance Type the Motor Array

# Servo Configuration:
servo_configs : list[ServoConfig] = [
    ServoConfig(channel=0, min_safe=s0_params[0],  straight=s0_params[1], max_safe=s0_params[2], name="Back Right"),
    ServoConfig(channel=1, min_safe=s1_params[0],  straight=s1_params[1], max_safe=s1_params[2], name="Front Right"),
    ServoConfig(channel=2, min_safe=s2_params[0],  straight=s2_params[1], max_safe=s2_params[2], name="Front Left"),
    ServoConfig(channel=3, min_safe=s3_params[0],  straight=s3_params[1], max_safe=s3_params[2], name="Back Left"),
    ]

"""
DYNAMIC CONFIGURATION:
"""

run_name = "MOV_TRL_7" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
log_dir = Path(__file__).resolve().parents[1] / "log"

def move_calibration(Rover : RoverDrive):
    BlockedMovement.moveforward(Rover, 200, 2.0)
    BlockedMovement.movebackward(Rover, 200, 2.0)
    BlockedMovement.rotateleft(Rover, 200, 2.0)
    BlockedMovement.rotateright(Rover, 200, 2.0)

def imu_calibration(Rover : RoverDrive, logName : str):
    pass

def rover_initialize():
    # I2C Setup for NVIDIA Jetson:
    print("[I2CSETUP] Begin")
    try:
        configured_bus_id = os.environ.get("I2C_BUS_ID")
        i2c_bus_id = int(configured_bus_id) if configured_bus_id is not None else None
        i2c_bus = create_i2c_tiny_usb_bus(i2c_bus_id, frequency=400_000)

        bus_path = f"/dev/i2c-{i2c_bus_id}" if i2c_bus_id is not None else None
    except Exception as e:
        print(f"[I2CSETUP] Failed to create I2C bus: {e}\n{traceback.format_exc()}")
        return 
    print("[I2CSETUP] End")

    print("[IMUSETUP] Begin:")
    bus_path = f"/dev/i2c-{i2c_bus_id}" if i2c_bus_id is not None else None
    try:
        imu = IMUWrapper(bus_path=bus_path)
    except Exception as e:
        print(f"[IMUSETUP] Failed to create IMU: {e}\n{traceback.format_exc()}")
        return
    print("[IMUSETUP] End")

    Rover : RoverDrive = RoverDrive(
        3200, 
    )

    Rover.debug = True 
    print("[ROVER]  RoverDrive debug mode enabled.")
    print("[ROVER] RoverDrive initialization begin.")
    print("[INFO] Rover initialization may make up to 30 seconds.")
    try:
        Rover.bringup(
            usb_ports=serial_ports,
            rc_addresses=controller_addreses,
            rc_names=["RC1 FL/ML", "RC2 BR/BL", "RC3 FR/MR"],
            motor_cfgs=motor_configs,
            baud_rate=baud_rate,
            servo_channels=channel_count,
            servo_pwm=pwm_frequency,
            servo_i2c_addr=i2c_address,
            servo_cfgs=servo_configs,
            debug=False,
            i2c_bus=i2c_bus,
            log_path=log_dir / f"rover-{run_name}.csv",
        )
    except Exception as e:
        print(f"[ROVER] Failed to bring up rover: {e}\n{traceback.format_exc()}")
        return 

    print(f"[ROVER] RoverDrive initialization complete.")
    return Rover

def main():
    Rover = rover_initialize()




if __name__ == "__main__":
    Rover = rover_initialize()