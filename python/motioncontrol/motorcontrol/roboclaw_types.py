"""
roboclaw_types.py
Defined Structures to Handle Acceleration/Deceleration LUTs.
"""

from typing import Callable
from basicmicro import Basicmicro as Roboclaw
from dataclasses import dataclass, field
from enum import Enum
import numpy as np

@dataclass(slots=True)
class RoboclawUnit:
    unit : Roboclaw
    serial_port : str 
    address: int
    name : str = ""
    
@dataclass(slots=True)
class Motor:
    id : int
    index : int
    channel : int
    encoder : float = 0.0
    speed : int = 0
    actual_speed : int = 0
    encoder_reversed : bool = False
    name : str = ""
    def setEncoderState(self, encoder_reversed : bool) -> None:
        self.encoder_reversed = encoder_reversed