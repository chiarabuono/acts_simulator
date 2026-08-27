"""
Central configuration for the ACTS simulator
"""

import numpy as np

# ------ Ground cables tension bounds ---------------------------------
TAU_MIN = 5.5
TAU_MAX = 150.0  

# ------ Controller gains ---------------------------------------------
kt = 5.5e-6
kd = 3.299e-7
kp = 28.0
kr_xy = 4.0
MAX_ROTOR_VELOCITY = 1666.0

# ------ Drone thrust bounds ------------------------------------------
THRUST_MIN = 1.0
THRUST_MAX = 4 * kt * MAX_ROTOR_VELOCITY**2 

D_SAFE_DRONE = 0.4      # Collision safety margins
D_SAFE_CABLE = 0.05     # Rubbing safety margins
MODE = "tau_optimal"    # ("tau_min" vs "tau_optimal")

# ------ Optimization parameters  --------------------------------------
OPTIMIZATION_FREQUENCY = 500
RENDER_EVERY_N_STEPS = 50
ITERATION_COLLECTION = 50  # Iteration at which indices are collected
CHECK_RUB_FREQUENCY = 500

MAX_WINCH_SPEED = 1.50  # m/s


# ------ Pose-reached thresholds ----------------------------------------
POS_TOLERANCE = 0.1              # Target position error threshold (in meters)
ROT_TOLERANCE = np.deg2rad(3.0)  # Target orientation error threshold (in radians or norm)