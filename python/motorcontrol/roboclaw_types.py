"""
Defined Structures to Handle Acceleration/Deceleration LUTs.
"""

from typing import Callable
from dataclasses import dataclass, field
from enum import Enum
import numpy as np


@dataclass(slots=True)
# Basic LUT container for basis motor control curves, with defined length, steepness, and holding the actual LUT object.
class CurveLUT:
    """
    Basically, the CurveLUT object represents a container that holds the desired length and steepness, 
    and the normalized coefficients for a continued keypress (WASD) in a given drive mode that determines 
    the response behavior.
    """
    length: int = 0
    steepness: float = 0
    array: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))

def populate_lut(method: Callable[[int, float], np.ndarray], lut: CurveLUT) -> None:
    lut.array = method(lut.length, lut.steepness)
    return


@dataclass(slots=True)
# Advanced LUT container for holding a motor control LUT based on a specific min/max speed.
class MotorCurveLUT:
    """
    The MotorCurveLUT takes the CurveLUT obect and defines it with a set minimum and maximum speed and 
    uses the normalized array to create a curve that perfectly matches the desired shape between the desired
    minimum and maximum speeds.
    With the defined populate_curve_lut method, it proves to be quite convenient to swap out different algorithms
    to handle motor acceleration, deceleration, braking etc...
    """
    min_speed: int = 0
    max_speed: int = 0
    length: int = 0
    steepness: float = 0
    array: np.ndarray =  field(default_factory=lambda: np.array([], dtype=float))

# Populate the Motor Curve Lut, this will FAIL if the incoming array is not properly set up.
def populate_curve_lut(method: Callable[[int, float], np.ndarray], lut: MotorCurveLUT):
    normalized_LUT = CurveLUT(lut.length, lut.steepness)
    normalized_LUT.array = method(lut.length, lut.steepness)
    lut.array = lut.min_speed + (normalized_LUT.array * (lut.max_speed - lut.min_speed))


class movement_state(Enum):
    """
    The movement state is useful here because it allows different states of motion that are deterministic
    our autonomy algorithms will find it easier to define tasks as it separates drive motion on the tracked base
    from rotational motion on that same base. Additionally, the STOPPED and BRAKE states allow for 
    return-to-zero motion during runs, avoiding runawway with the robot.
    """
    STOPPED = 0
    BRAKE = 1
    FORWARD_DRIVE = 2
    REVERSE_DRIVE = 3
    TURN_DRIVE = 4

@dataclass(slots=True)
class SpeedConfig:
    """
    Sets the configured max speeds for each state, various limitations - see comments.
    """
    # Defines the maximum forward speed (pick something safe, don't want to crash and destroy the expensive bot).
    max_forward: int = 0
    # Defines the minimum forward speed.
    min_forward: int = 0 
    # Defines the maximum reverse speed (likely slower, think about the robots center of mass and inertial response).
    max_reverse: int = 0
    # Defines the minimum reverse speed.
    min_reverse: int = 0
    # For Phase 1, disregard differential control in favor of the simplest possible turning regime.
    turn: int = 0
    # Brake state speed may not be zero, considering QPPM > 0 may still not overcome the motors transient.
    brake: int = 0
    # Stopped speed must always be zero.
    stopped: int = 0

@dataclass(slots=True)
class MotorCurveLUTConfig:
    """
    Holds the preconfigured LUTs for each movement state to be popoulated from a SpeedConfig and through an appropriate methods, various limitations, see comments.
    """
    # Defines the forward acceleration curve
    forward_accelerate_lut: MotorCurveLUT
    # Defines the forward deceleration curve e.g. as robot returns to the braked/stopped state
    forward_decelerate_lut: MotorCurveLUT
     # Defines the reverse acceleration curve
    reverse_accelerate_lut: MotorCurveLUT
    # Defines the reverse deceleration curve e.g. as robot returns to the braked/stopped state
    reverse_decelerate_lut: MotorCurveLUT
    # Defines the basic turning curve, again this will likely be constant, and in the future, there is likely multiple different LUTs to handle differential drive.
    turning_lut: MotorCurveLUT 
    # Defines the general braking regime regardless of state as the default response to some error.
    general_braking_lut: MotorCurveLUT
    # Defines an absolute zero stopped state.
    stopped_lut: MotorCurveLUT





