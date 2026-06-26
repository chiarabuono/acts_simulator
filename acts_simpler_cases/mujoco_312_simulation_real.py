import mujoco
import numpy as np
import tkinter as tk
import threading
import mujoco.viewer
import time
from scipy.spatial.transform import Rotation as R

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from acts_simulator.utils_control import PayloadControlDrone


with open("mujoco/simpler_cases/312_model_real.xml", "r") as f:
    xml_model = f.read()

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
drone_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

payload_dof_offset = model.body_dofadr[payload_id]
drone_dof_offset   = model.body_dofadr[drone_id] 

cable_1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_1")
cable_2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_2")

CABLE_1_MAX_L = model.tendon_range[cable_1_id][1]       # payload to drone
CABLE_2_MAX_L = model.tendon_range[cable_2_id][1]       # payload to ground

m_payload = model.body("payload").mass[0]
m_drone = model.body("drone").mass[0] 
e3 = np.array([0.0, 0.0, 1.0])
a2 = np.array([0.0, 0.0, 0.0])                  # anchor point

gains_platform = {
    'Kp_pl': 15.0, 
    'Kd_pl': 6.0
}

drone = PayloadControlDrone(
    model, 
    drone_name="drone", 
    payload_mass=m_payload, 
    gains_platform=gains_platform
)

MODE = "force_control"

def run_tuning_gui():
    root = tk.Tk()
    root.title("Gain Tuner")
    root.geometry("340x380")
    payload_sliders = [        
        ("Platform Kp",  'Kp_pl',  0, 30),      #0.2
        ("Platform Kd",  'Kd_pl',  0, 10)       #0.8
    ]

    for label, key, lo, hi in payload_sliders:
        tk.Label(root, text=label).pack()
        s = tk.Scale(root, from_=lo, to=hi, resolution=0.1, orient='horizontal',
                     command=lambda v, k=key: gains_platform.update({k: float(v)}))
        s.set(gains_platform[key])
        s.pack(fill='x', padx=10)

    root.mainloop()


threading.Thread(target=run_tuning_gui, daemon=True).start()

if MODE == "payload_position_control":
    p_star = np.copy(data.mocap_pos[0])
elif MODE == "force_control":
    f_star = np.array([1.0, 2.0, 10.0])
    f_star_norm = np.linalg.norm(f_star)
    u2_star = f_star / f_star_norm                       # Eq 3.11            
    p_star = a2 + CABLE_2_MAX_L * u2_star                # Eq 3.21

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        data.mocap_pos[0] = p_star
            
        p_star_dot = np.array([0.0, 0.0, 0.0])
        p_star_ddot = np.array([0.0, 0.0, 0.0])

        p = data.xpos[payload_id].copy()
        p_dot = data.qvel[payload_dof_offset : payload_dof_offset+3].copy()   
        drone.update_payload_pose(p, p_dot)
        
        R_mat = data.xmat[drone_id].reshape(3, 3)
        current_quat = R.from_matrix(R_mat).as_quat()
        
        angular_vel = R_mat.T @ data.qvel[drone_dof_offset+3 : drone_dof_offset+6]  

        # (Fa,2)
        vec_cable2 = p - a2
        dist_cable2 = np.linalg.norm(vec_cable2)
        u2 = vec_cable2 / dist_cable2 if dist_cable2 > 0.001 else -e3

        tau2 = -data.efc_force[0] if data.efc_force.size > 0 else 0.0
        tau2 = max(0.0, tau2)    
        Fa_2 = tau2 * u2

        drone.set_payload_states(p_star_ddot, Fa_2, p_star, p_star_dot)

        a = data.xpos[drone_id].copy()
        a_star = p_star + np.array([0.0, 0.0, CABLE_1_MAX_L])
        a_star_dot = np.array([0.0, 0.0, 0.0])
        a_dot = data.qvel[drone_dof_offset : drone_dof_offset+3].copy()   

        wrench, R_des = drone.compute_motor_wrenches(a_star, a, a_star_dot, a_dot, current_quat, angular_vel)

        # Map computed wrench allocations directly to drone center of mass
        data.xfrc_applied[drone_id, 0:3] = wrench[0] * R_des[:, 2] 
        data.xfrc_applied[drone_id, 3:6] = R_des @ wrench[1:4]     

        # Step simulation physics
        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)