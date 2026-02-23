"""
Basic Methods for Setting up and Managing the Roboclaw MCU
"""
import time
from basicmicro import Basicmicro as Roboclaw
import typing

def roboclaw_setup(roboclaw : Roboclaw, serial_port: str, baud_rate: int, controller_address: int):
    print(f"Starting Setup with Port: {serial_port}, BAUD Rate: {baud_rate}, Controller Address: {controller_address}.")
    print(f"Attempting to Connect to Roboclaw.")\
    # Attempt Connection:
    try:
        firmware_version = roboclaw.ReadVersion(controller_address)
        if (firmware_version[0]):
            print(f"Connected to Roboclaw with firmware version: {firmware_version[1]}.")
            return True
        else:
            print(f"Failed to connect to Roboclaw: (Firmware Read Failed).")
            return False
    except Exception as error:
        print(f"Error Connecting to Roboclaw with Port: {serial_port}, BAUD Rate: {baud_rate}, Controller Address: {controller_address}")
        print(str(error))


