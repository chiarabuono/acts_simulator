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
from acts_simulator.utils_control import FreeFlightDrone


with open("mujoco/simpler_cases/single_drone.xml", "r") as f:
    xml_model = f.read()

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)
drone = FreeFlightDrone(model, drone_name="drone")

drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

# Shared state parameters for the tuning GUI
ctrl_params = {
    'px': 1.0, 'py': -1.0, 'pz': 1.5,
}

def run_tuning_gui():
    root = tk.Tk()
    root.title("ROS 2 -> MuJoCo Adapted Tuning Panel")
    root.geometry("380x450")

    def update_val(key, val):
        ctrl_params[key] = float(val)

    tk.Label(root, text="Target Assignment Coordinates", font=('Helvetica', 10, 'bold')).pack(pady=5)
    for p in ['px', 'py', 'pz']:
        tk.Label(root, text=f"Target {p.upper()}").pack()
        s = tk.Scale(root, from_=-4.0 if p!='pz' else 0.2, to=4.0, resolution=0.05, orient='horizontal', command=lambda v, k=p: update_val(k, v))
        s.set(ctrl_params[p])
        s.pack(fill='x', padx=15)

    root.mainloop()


threading.Thread(target=run_tuning_gui, daemon=True).start()

# --- Core Physics Simulation Loop ---
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():
        step_start = time.time()

        # 1. State extraction directly from MuJoCo structures
        p = data.xpos[drone_id].copy()
        v = data.qvel[0:3].copy()
        
        # Continuous rotation state extraction
        R_mat = data.xmat[drone_id].reshape(3, 3)
        current_quat = R.from_matrix(R_mat).as_quat()
        
        # Local body angular velocities
        angular_vel = R_mat.T @ data.qvel[3:6]

        # 2. Map GUI references to Target Trajectory state arrays
        pd = np.array([ctrl_params['px'], ctrl_params['py'], ctrl_params['pz']])
        vd = np.zeros(3)

        # 3. Calculate system wrenches using the ROS 2 node's algorithmic logic
        wrench, R_des = drone.compute_motor_wrenches(pd, p, vd, v, current_quat, angular_vel)

        # 4. Map body wrench array values to MuJoCo applied forces vectors
        data.xfrc_applied[drone_id, 0:3] = wrench[0] * R_mat[:, 2] # Apply collective thrust along body Z axis
        data.xfrc_applied[drone_id, 3:6] = R_mat @ wrench[1:4]     # Map roll/pitch/yaw moments to global frame

        # Step simulation physics
        mujoco.mj_step(model, data)
        viewer.sync()

        # Keep real-time pace
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)