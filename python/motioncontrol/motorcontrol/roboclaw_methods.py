"""
roboclaw_methods.py
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
import sys
import tty
import termios
import typing
from typing import Tuple
# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw
from .roboclaw_types import RoboclawUnit, Motor

# Take the Linux Ports, find whatever responds as a Roboclaw to the given address and baud rate:
def discover_serial(ports: list[str], address: int, baud_rate: int, debug: bool = False) -> Tuple[Roboclaw | None, str | None]:
    for port in ports:
        claw = Roboclaw(comport=port, rate=baud_rate)
        if (debug):
            print(f"Trying to set up Roboclaw: {address}@{port}")
        try:
            if claw.Open() and claw.ReadVersion(address)[0]:
                return claw, port
        except Exception:
            pass

    return None, None


def roboclaw_setup(candidate_ports : list[str], expected_addresses : list[int], names: list[str], baud_rate : int, debug : bool = False) -> list[RoboclawUnit]:
    ordered_roboclaws: list[RoboclawUnit | None] = [None] * len(expected_addresses)
    for i in range(len(expected_addresses)):
        address = expected_addresses[i]
        roboclaw, port = discover_serial(ports=candidate_ports, address=address, baud_rate=baud_rate, debug=debug)
        if (roboclaw is not None) and (port is not None):
            ordered_roboclaws[i] = RoboclawUnit(unit = roboclaw, serial_port=port, address=address, name=names[i])
        else:
            if (debug):
                print(f"Roboclaw: {address} failed to resolve over any serial port.")
    if any(roboclaw is None for roboclaw in ordered_roboclaws):
        raise RuntimeError("One or more RoboClaws were not discovered")
    return typing.cast(list[RoboclawUnit], ordered_roboclaws)

def motor_setup(motor_configs : list[Tuple[Motor, int]], roboclaws : list[RoboclawUnit], debug : bool = False, readonly : bool = False):
    staged_qpps : list[list[int]] = [[0, 0] for i in range(len(roboclaws))]
    staged_pid : list[list[Tuple]] = [[(), ()] for i in range(len(roboclaws))]
    staged_readback : list[list[Tuple]] = [[(), ()] for i in range(len(roboclaws))]

    # Stage QPPS:
    for i in range(len(motor_configs)):
        motor = motor_configs[i][0]
        motor_qpps = motor_configs[i][1]
        staged_qpps[motor.index][motor.channel] = motor_qpps
        if (debug):
            print(f"Staging Motor: {motor.name}, Roboclaw Index: {motor.index}, Channel: {motor.channel}, QPPS: {motor_qpps}")

    # Read Existing PID:
    for i in range(len(roboclaws)):
        roboclaw = roboclaws[i]
        if (debug):
            print(f"Reading Existing PID Configuration: {roboclaw.name}@{roboclaw.address}")

        staged_pid[i][0] = roboclaw.unit.ReadM1VelocityPID(roboclaw.address)
        staged_pid[i][1] = roboclaw.unit.ReadM2VelocityPID(roboclaw.address)

        if not staged_pid[i][0][0] or not staged_pid[i][1][0]:
            raise RuntimeError(f"{roboclaw.name} failed to read motor PID configuration")

        if (debug):
            print(f"Existing PID Configuration Read Successfully: {roboclaw.name}@{roboclaw.address}")

    # Output Existing PID and QPPS:
    if (readonly):
        for i in range(len(motor_configs)):
            motor = motor_configs[i][0]
            motor_pid = staged_pid[motor.index][motor.channel]
            print(f"Motor: {motor.name}, P: {motor_pid[1]}, I: {motor_pid[2]}, D: {motor_pid[3]}, QPPS: {motor_pid[4]}")
        return

    # Write QPPS:
    for i in range(len(roboclaws)):
        roboclaw = roboclaws[i]
        m1_pid = staged_pid[i][0]
        m2_pid = staged_pid[i][1]

        if (debug):
            print(f"Writing QPPS Configuration: {roboclaw.name}@{roboclaw.address}, M1: {staged_qpps[i][0]}, M2: {staged_qpps[i][1]}")

        if not roboclaw.unit.SetM1VelocityPID(roboclaw.address, m1_pid[1], m1_pid[2], m1_pid[3], staged_qpps[i][0]):
            raise RuntimeError(f"{roboclaw.name} failed to write M1 QPPS")

        if not roboclaw.unit.SetM2VelocityPID(roboclaw.address, m2_pid[1], m2_pid[2], m2_pid[3], staged_qpps[i][1]):
            raise RuntimeError(f"{roboclaw.name} failed to write M2 QPPS")

        if (debug):
            print(f"QPPS Configuration Written Successfully: {roboclaw.name}@{roboclaw.address}")

    # Read QPPS:
    for i in range(len(roboclaws)):
        roboclaw = roboclaws[i]
        if (debug):
            print(f"Reading QPPS Configuration: {roboclaw.name}@{roboclaw.address}")

        staged_readback[i][0] = roboclaw.unit.ReadM1VelocityPID(roboclaw.address)
        staged_readback[i][1] = roboclaw.unit.ReadM2VelocityPID(roboclaw.address)

        if not staged_readback[i][0][0] or not staged_readback[i][1][0]:
            raise RuntimeError(f"{roboclaw.name} failed to read motor PID configuration")

        if (debug):
            print(f"QPPS Readback: {roboclaw.name}@{roboclaw.address}, M1: {staged_readback[i][0][4]}, M2: {staged_readback[i][1][4]}")

    # Verify Readback:
    for i in range(len(roboclaws)):
        if staged_readback[i][0][4] != staged_qpps[i][0] or staged_readback[i][1][4] != staged_qpps[i][1]:
            raise RuntimeError(f"{roboclaws[i].name} QPPS verification failed")

        if (debug):
            print(f"QPPS Configuration Verified Successfully: {roboclaws[i].name}@{roboclaws[i].address}")


def move_motors(motors : list[Motor], roboclaws: list[RoboclawUnit]):
    staged_movement : list[list[int]] = [[0, 0] for i in range(len(roboclaws))]
    staged_readback : list[Tuple[bool, int, int]] = [(False, 0, 0)] * len(roboclaws)

    # Stage Movement:
    for motor in motors:
        if (motor.encoder_reversed):
            staged_movement[motor.index][motor.channel] = -1 * motor.speed
        else:
            staged_movement[motor.index][motor.channel] = motor.speed

    # Write Movement:
    for i in range(len(roboclaws)):
        roboclaw = roboclaws[i]
        if not roboclaw.unit.SpeedM1M2(roboclaw.address, staged_movement[i][0], staged_movement[i][1]):
            raise RuntimeError(f"{roboclaw.name} failed to write motor speeds")

    # Read Movement:
    for i in range(len(roboclaws)):
        roboclaw = roboclaws[i]
        staged_readback[i] = roboclaw.unit.GetISpeeds(roboclaw.address)
        if not staged_readback[i][0]:
            raise RuntimeError(f"{roboclaw.name} failed to read motor speeds")

    # Apply Readback:
    for motor in motors:
        motor.actual_speed = staged_readback[motor.index][motor.channel + 1]

def testing_motor_movement_loop(roboclaws: list[RoboclawUnit], motors: list[Motor], selected_motor: int, speed: int, debug: bool):
    # Set Termios Raw Nonblocking Char input:
    fd = sys.stdin.fileno()
    org_term_settings = termios.tcgetattr(fd)

    try:
        if (debug):
            print(f"Controlling Motor {motors[selected_motor]}. ")
            print(f"Beginning Movement Loop (C/c to Cancel): ")
        tty.setraw(fd)
        while True:
            movement = sys.stdin.read(1).lower()
            if debug:
                print(movement, end="\r\n")
                sys.stdout.flush()

            if movement == "w":
                motors[selected_motor].speed = speed
                move_motors(roboclaws=roboclaws, motors=motors)

            elif movement == "s":
                motors[selected_motor].speed = -speed
                move_motors(roboclaws=roboclaws, motors=motors)

            elif movement == " ":
                motors[selected_motor].speed = 0
                move_motors(roboclaws=roboclaws, motors=motors)

            elif movement == "c":
                motors[selected_motor].speed = 0
                move_motors(roboclaws=roboclaws, motors=motors)
                break

    finally:
        try:
            motors[selected_motor].speed = 0
            move_motors(roboclaws=roboclaws, motors=motors)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, org_term_settings)
        
    
