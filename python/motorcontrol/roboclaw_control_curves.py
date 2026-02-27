"""
Advanced Methods for defining loop-speed curves.
"""

# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw

# Numpy
import numpy as np 

def normalized_logistic_array(length: int, steepness: float):
    """
    Constructs an array of length N with discrete logistic S-curve normalized coefficients.

    Args:
        length (int): The discrete array legnth (N)
        steepness (float): The steepness parameter of the logistic curve (k).
    Returns:
        np.ndarray: Vector entries following an S-curve, normalized to sum to 1
    """
    if length <= 0:
        return np.array([])
    if length == 1:
        return np.array([])
    
    indices = np.arange(length)

    # Center the S-curve at the midpoint of the array
    midpoint = (length - 1) / 2.0

    # Raw logistic values
    raw_values = 1 / (1 + np.exp(-steepness * (indices - midpoint)))

    # Min/Max Scale:
    min_val = np.min(raw_values)
    max_val = np.max(raw_values)

    # Normalize Logistic Coefficients
    scaled_coefficients = (raw_values - min_val) / (max_val - min_val)
    return scaled_coefficients

def normalized_decay_array(length: int, steepness: float):
    """
    Constructs an array of length N with discrete exponential decay normalized coefficients.

    Args:
        length (int): The discrete array legnth (N)
        steepness (float): The steepness parameter of the logistic curve (k).
    Returns:
        np.ndarray: Vector entries following an exponential decay, minmax normalized to [0,1]
    """
    if length <= 0:
        return np.array([])
    if length == 1:
        return np.array([1.0])
    
    indices = np.arange(length)

    # Raw logistic values
    raw_values = np.exp(-steepness * indices)

    # Min/Max Scale:
    min_val = np.min(raw_values)
    max_val = np.max(raw_values)

    # Normalize Logistic Coefficients
    scaled_coefficients = (raw_values - min_val) / (max_val - min_val)
    return scaled_coefficients

    