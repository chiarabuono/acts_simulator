import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk
from scipy.spatial.transform import Rotation as R

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from acts_simulator.utils_control import CableTetheredDrone

with open("mujoco/simpler_cases/311_model_real.xml", "r") as f:
    xml_model = f.read()

model = mujoco.MjModel.from_xml_string(xml_model)
data  = mujoco.MjData(model)
drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

CABLE_LENGTH_L = model.tendon_range[0][1]
b      = np.array([0.0, 0.0, 0.0])     # cable anchor
f_star = np.array([2.0, 0.0, 8.0])     # desired cable force vector

drone = CableTetheredDrone(
    model, 
    drone_name="drone", 
    anchor_b= b,
    cable_len=CABLE_LENGTH_L,
    f_star= f_star
)

# ---------------- Actuators and winch ids ---------------------------------------------
thrust_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "thrust")
roll_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "roll")
pitch_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "pitch")
yaw_id    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "yaw")

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        step_start = time.time()

        p = data.xpos[drone_id].copy()
        v = data.qvel[0:3].copy()
        
        # Continuous rotation state extraction
        R_mat = data.xmat[drone_id].reshape(3, 3)
        current_quat = R.from_matrix(R_mat).as_quat()
        
        # Local body angular velocities
        angular_vel = R_mat.T @ data.qvel[3:6]


        u_star   = f_star / np.linalg.norm(f_star)              # Eq 3.11
        a_star   = b + CABLE_LENGTH_L * u_star                  # Eq 3.12
        a_star_dot = np.zeros(3)

        wrench, R_des = drone.compute_motor_wrenches(a_star, p, a_star_dot, v, current_quat, angular_vel)

        data.ctrl[thrust_id] = wrench[0]  # Thrust
        data.ctrl[roll_id]   = wrench[1]  # Roll torque
        data.ctrl[pitch_id]  = wrench[2]  # Pitch torque
        data.ctrl[yaw_id]    = wrench[3]  # Yaw torque

        # Step simulation physics
        mujoco.mj_step(model, data)
        viewer.sync()

        # Keep real-time pace
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)