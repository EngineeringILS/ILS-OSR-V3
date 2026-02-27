"""
Defined Structures to Handle Acceleration/Deceleration LUTs.
"""
from typing import Callable
from dataclasses import dataclass, field
import numpy as np

@dataclass(slots=True)
# Basic LUT container for motor control curves, with defined length, steepness, and holding the actual LUT object.
class CurveLUT:
    length: int = 0
    steepness: float = 0
    array: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))

def populate_lut(method: Callable[[int, float], np.ndarray], lut: CurveLUT) -> None:
    lut.array = method(lut.length, lut.steepness)
    return