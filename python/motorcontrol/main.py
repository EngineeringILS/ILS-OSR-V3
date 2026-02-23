"""
TAMU Lunabotics 2026 Phase 1 Roboclaw 2x15A Motor Control Harness
"""
import roboclaw_methods as methods
from basicmicro import Basicmicro as Roboclaw
from config import serial_port, baud_rate, controller_address

def main():
    roboclaw = Roboclaw(serial_port, baud_rate)
    methods.roboclaw_setup(roboclaw=roboclaw, serial_port=serial_port, baud_rate=baud_rate, controller_address=controller_address)

if __name__ == "__main__":
    main()
