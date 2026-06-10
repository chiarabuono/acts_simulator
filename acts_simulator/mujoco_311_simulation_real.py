from utils_control import Drone
import mujoco
import numpy as np
import tkinter as tk
import threading
import mujoco.viewer

with open("mujoco/311_model_real.xml", "r") as f:
    xml_model = f.read()

model = mujoco.MjModel.from_xml_string(xml_model)
data  = mujoco.MjData(model)

CABLE_LENGTH_L = model.tendon_range[0][1]
b      = np.array([0.0, 0.0, 0.0])     # cable anchor
f_star = np.array([4.0, 2.0, 8.0])     # desired cable force vector


drone = Drone(model, data, "drone")
drone.set_desired(f_star, b, CABLE_LENGTH_L)

def run_tuning_gui():
    
    root = tk.Tk()
    root.title("Quadrotor Gain Tuner")
    root.geometry("340x380")
    sliders = [
        ("Position Kp",  'Kp_pos', 0, 30),
        ("Position Kd",  'Kd_pos', 0, 10),
        ("Attitude Kp",  'Kp_att', 0, 30),
        ("Attitude Kd",  'Kd_att', 0, 10),
    ]
    for label, key, lo, hi in sliders:
        tk.Label(root, text=label).pack()
        s = tk.Scale(root, from_=lo, to=hi, resolution=0.1, orient='horizontal',
                     command=lambda v, k=key: drone.gains.update({k: float(v)}))
        s.set(drone.gains[key])
        s.pack(fill='x', padx=10)
    root.mainloop()

threading.Thread(target=run_tuning_gui, daemon=True).start()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        drone.step(data, drone.a_star, drone.a_star_dot)
        mujoco.mj_step(model, data)
        viewer.sync()

        # drone.step(data, drone.a_star, drone.a_star_dot)

        # a_ddot     = (data.qvel[0:3] - prev_qvel) / model.opt.timestep
        # prev_qvel  = data.qvel[0:3].copy()

        # F_prop  = data.xfrc_applied[drone.drone_id, 0:3]
        # F_g     = np.array([0.0, 0.0, -drone.mass * drone.g])
        # f_hat   = F_prop + F_g - drone.mass * a_ddot

        # tau_hat = np.linalg.norm(f_hat)
        # a       = data.xpos[drone.drone_id].copy()
        # u_act   = (a - b) / np.linalg.norm(a - b)

        # mujoco.mj_step(model, data)
        # viewer.sync()

        # print(f"desired  |f*|: {drone.tau_star:.3f} N | "
        #       f"actual   |f|: {tau_hat:.3f} N | "
        #       f"dir error: {np.linalg.norm(u_act - drone.u_star):.4f}")