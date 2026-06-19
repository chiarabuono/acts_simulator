from utils_control import CableTetheredDrone
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk
from scipy.spatial.transform import Rotation as R

with open("mujoco/311_model_real.xml", "r") as f:
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

        data.xfrc_applied[drone_id, 0:3] = wrench[0] * R_mat[:, 2] # Apply collective thrust along body Z axis
        data.xfrc_applied[drone_id, 3:6] = R_mat @ wrench[1:4]     # Map roll/pitch/yaw moments to global frame

        # Step simulation physics
        mujoco.mj_step(model, data)
        viewer.sync()

        # Keep real-time pace
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)