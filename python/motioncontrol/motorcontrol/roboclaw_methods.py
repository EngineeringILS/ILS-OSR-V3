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
from roboclaw_types import RoboclawUnit, Motor

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

def move_motors(motors : list[Motor], roboclaws: list[RoboclawUnit]):
    staged_movement : list[list[int]] = [[0, 0] for i in range(len(roboclaws))]
    staged_readback : list[Tuple[bool, int, int]] = [(False, 0, 0)] * len(roboclaws)

    # Stage Movement:
    for motor in motors:
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
        
    
