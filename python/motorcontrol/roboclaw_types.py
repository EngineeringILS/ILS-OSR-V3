"""
roboclaw_types.py
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
    array: np.ndarray =  field(default_factory=lambda: np.array([], dtype=int))

# Populate the Motor Curve Lut, this will FAIL if the incoming array is not properly set up.
def populate_curve_lut(method: Callable[[int, float], np.ndarray], lut: MotorCurveLUT):
    normalized_LUT = CurveLUT(lut.length, lut.steepness)
    normalized_LUT.array = method(lut.length, lut.steepness)
    lut.array = (lut.min_speed + (normalized_LUT.array * (lut.max_speed - lut.min_speed))).astype(int)

def reverse_index_lut(lut: MotorCurveLUT, speed: int) -> int:
    """Reverse index against the next-mode LUT to handle substate transitions - see FSM diagram in CDH Google Drive (04/FSM - Concept) """
    # LUT arrays won't be none/0 - if anyone preprograms the arrays to be zero - it's not an idea any human would implement.
    # The only valid case is to return -1, in which case we call the ESTOP() method, because if the array has become empty - there is no recoverable path.
    if lut.array is None or len(lut.array) == 0:
        return -1
    
    # Find the best insertion point based on the consistently shaped LUT:
    idx = np.searchsorted(lut.array, speed)

    # Handle Boundary Conditions:
    if idx == 0:
        return 0
    if idx == len(lut.array):
        return len(lut.array) - 1

    # Nearest Neighbor check
    # If speed is closer to 'before' than 'after', return idx-1
    if speed - lut.array[idx - 1] < lut.array[idx] - speed:
        return int(idx - 1)
    return int(idx)
    
class MovementState(Enum):
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

class MovementSubState(Enum):
    """
    Each movement state can be broken down into the following substates
    """
    STOPPED = 0  # Sends to the global movement state once achieved
    ACCELERATING = 1 # Obeying some acceleration LUT map
    DECELERATING = 2 # Obeying some deceleration LUT map
    CONSTANT_VELOCITY = 3 # If at the peak of the acceleration curve
    BRAKING  = 4 # Obeying the brake curve

""" Movement intuition: 1st input (W) --> FORWARD, in the forward substate until the set of null-inputs accumulates to a set value --> STOPPED
    While in FORWARD: W keep accelerating, null start decelerating, S start braking, space/interrupt E-STOP --> STOPPED
    This logic is pretty much true in all cases, but if W be held down, eventually a constant velocity is established.
"""


@dataclass(slots=True)
class MotorControllerState:
    """
    Holds the real-time state of a single motor's controller FSM.
    This boject persists and is updated in every control loop cycle.
    """
    state: MovementState = MovementState.STOPPED
    substate: MovementSubState = MovementSubState.STOPPED
    # Ideal speed that was set by the last comamnd in the control loop.
    # Generalized Speed for FWD/REV
    straight_current_speed: int = 0
    # Specialized Speeds engaged in FWD/REV, but specifically important for turning drive, and future differential drive (if added).
    m1_current_speed: int = 0
    m2_current_speed: int = 0
    # The current index of the LUT
    lut_index: int = 0 
    # Default last_input to null --> braking or stopped.
    last_input: str = None
    # The number of continuous inputs that have been inputted repeatedly.
    num_inputs: int = 0 
    # WARNING! Do not assume that num_inputs matches the lut_index, if dynamic deceleration and acceleration are engaged in the same mode, then these are probably different.
    # Movement Error - Arises from failed motor motion calls:
    movement_err: bool = False

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
    forward_accelerate_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
    # Defines the forward deceleration curve e.g. as robot returns to the braked/stopped state
    forward_decelerate_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
     # Defines the reverse acceleration curve
    reverse_accelerate_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
    # Defines the reverse deceleration curve e.g. as robot returns to the braked/stopped state
    reverse_decelerate_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
    # Defines the basic turning curve, again this will likely be constant, and in the future, there is likely multiple different LUTs to handle differential drive.
    turning_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
    # Defines the general braking regime regardless of state as the default response to some error.
    general_braking_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)
    # Defines an absolute zero stopped state.
    stopped_lut: MotorCurveLUT = field(default_factory=MotorCurveLUT)





