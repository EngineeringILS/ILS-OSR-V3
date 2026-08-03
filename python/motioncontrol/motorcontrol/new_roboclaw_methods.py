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
from roboclaw_types import RoboclawUnit, MotorConfig

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


def roboclaw_setup(candidate_ports : list[str], expected_addresses : list[int], names: list[str], baud_rate : int, debug : bool = False) -> list[RoboclawUnit | None]:
    if len(names) != len(expected_addresses):
        raise ValueError("Each RoboClaw address must have a name")

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
    return ordered_roboclaws    


        

