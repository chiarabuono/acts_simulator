import numpy as np
TAU_MIN = 5.5
TAU_MAX = 150.0  

D_SAFE_DRONE = 0.4
D_SAFE_CABLE = 0.1
MODE = "tau_optimal" # tau_min or tau_optimal

# ------ Optimization parameters  ------------------------------------------------------------
OPTIMIZATION_FREQUENCY = 500
RENDER_EVERY_N_STEPS = 50
ITERATION_COLLECTION = 30  # Iteration at which indices are collected

MAX_WINCH_SPEED = 1.00  # m/s

MAX_ROTOR_VELOCITY = 1666.0
kt = 5.5e-6
kd = 3.299e-7
max_thrust = 4 * kt * MAX_ROTOR_VELOCITY**2 