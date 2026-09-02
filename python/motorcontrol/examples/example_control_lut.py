"""
Test Behavior of roboclaw_control_curves and roboclaw_types.py
"""

import sys
import pathlib
# Relative python path objectively sucks - this language sucks - this never is a problem with Cmake... 
# This finds the directory of the current script (magic code, user tests work easily)
current_dir = pathlib.Path(__file__).parent 
sys.path.append(str(current_dir))
sys.path.append(str(current_dir.parent))

from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
from roboclaw_types import CurveLUT, populate_lut
import matplotlib.pyplot as plt
import numpy as np
# Visualize Control Curve Behavior (migrate later):

lut_steepness: np.ndarray = np.linspace(0.1,0.5, 9)
lut_len: int = 50

# for slope in lut_steepness:
#     logi_curve_lut = CurveLUT(lut_len, slope)
#     decay_curve_lut = CurveLUT(lut_len, slope)
#     populate_lut(normalized_logistic_array, logi_curve_lut)
#     populate_lut(normalized_decay_array, decay_curve_lut)
#     plt.scatter(np.arange(0, lut_len), logi_curve_lut.array)
#     plt.scatter(np.arange(0, lut_len), decay_curve_lut.array)
#     plt.show()
    
# Calculate grid dimensions
num_plots = len(lut_steepness)
cols = int(np.ceil(np.sqrt(num_plots)))
rows = int(np.ceil(num_plots / cols))

fig, axes = plt.subplots(rows, cols, figsize=(cols * 4, rows * 4), constrained_layout=True)
axes_flat = axes.flatten() if num_plots > 1 else [axes]

for i, slope in enumerate(lut_steepness):
    ax = axes_flat[i]
    
    # Logic from your original loop
    logi_curve_lut = CurveLUT(lut_len, slope)
    decay_curve_lut = CurveLUT(lut_len, slope)
    populate_lut(normalized_logistic_array, logi_curve_lut)
    populate_lut(normalized_decay_array, decay_curve_lut)
    
    # Plotting to the specific subplot axis
    x_vals = np.arange(0, lut_len)
    ax.scatter(x_vals, logi_curve_lut.array, s=10, label='Logistic')
    ax.scatter(x_vals, decay_curve_lut.array, s=10, label='Decay', alpha=0.7)
    
    ax.set_title(f"Slope: {slope}")
    ax.legend()

# Hide any unused subplots in the grid
for j in range(i + 1, len(axes_flat)):
    axes_flat[j].axis('off')

plt.show()