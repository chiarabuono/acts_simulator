from utils_control import Drone
import mujoco
import numpy as np
import tkinter as tk
import threading
import mujoco.viewer

with open("mujoco/312_model_real.xml", "r") as f:
    xml_model = f.read()

model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
drone_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

payload_dof_offset = model.body_dofadr[payload_id]
drone_dof_offset   = model.body_dofadr[drone_id]   

CABLE_1_MAX_L = model.tendon_range[1][1]        # payload to drone
CABLE_2_MAX_L = model.tendon_range[0][1]        # payload to ground

m_payload = model.body("payload").mass[0]
m_drone = model.body("drone").mass[0]  
g = 9.81            
e3 = np.array([0.0, 0.0, 1.0])
a2 = np.array([0.0, 0.0, 0.0])                  # anchor point

gains_platform = {
    'Kp_pl': 0.2,
    'Kd_pl': 0.1,

}

MODE = "force_control"

drone = Drone(model, data, "drone", qvel_offset=drone_dof_offset)
drone.set_desired(np.array([0, 0, 1]), data.xpos[payload_id], CABLE_1_MAX_L)

def run_tuning_gui():
    
    root = tk.Tk()
    root.title("Quadrotor Gain Tuner")
    root.geometry("340x380")
    drone_sliders = [
        ("Position Kp",  'Kp_pos', 0, 30),      #0.1
        ("Position Kd",  'Kd_pos', 0, 10),      #0.1
        ("Attitude Kp",  'Kp_att', 0, 30),      #0.1
        ("Attitude Kd",  'Kd_att', 0, 10),      #0.1
    ]
    payload_sliders = [        
        ("Platform Kp",  'Kp_pl',  0, 30),      #0.2
        ("Platform Kd",  'Kp_pl',  0, 10)       #0.8
    ]
    
    for label, key, lo, hi in drone_sliders:
        tk.Label(root, text=label).pack()
        s = tk.Scale(root, from_=lo, to=hi, resolution=0.1, orient='horizontal',
                     command=lambda v, k=key: drone.gains.update({k: float(v)}))
        s.set(drone.gains[key])
        s.pack(fill='x', padx=10)

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
    tau_star = f_star_norm                               # Eq 3.10
    u2_star = f_star / f_star_norm                       # Eq 3.11            
    p_star = a2 + CABLE_2_MAX_L * u2_star                # Eq 3.21

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        dist_from_anchor = np.linalg.norm(p_star)
        data.mocap_pos[0] = p_star
            
        p_star_dot = np.array([0.0, 0.0, 0.0])
        p_star_ddot = np.array([0.0, 0.0, 0.0])

        p  = data.xpos[payload_id]
        p_dot = data.qvel[0:3]   
        
        a1 = data.xpos[drone_id]
        a1_dot = data.qvel[6:9]  

        prev_p_dot = data.qvel[0:3].copy()

        # (Fa,2)
        vec_cable2 = p - a2
        dist_cable2 = np.linalg.norm(vec_cable2)
        u2 = vec_cable2 / dist_cable2 if dist_cable2 > 0.001 else e3

        tau2 = -data.efc_force[0] if data.efc_force.size > 0 else 0.0
        tau2 = max(0.0, tau2)    
        Fa_2 = tau2 * u2

        F_p_star = m_payload * (p_star_ddot + g * e3) + \
                    gains_platform['Kp_pl'] * (p_star - p) + gains_platform['Kd_pl'] * (p_star_dot - p_dot) # Eq. 3.19

        F_a1_star = F_p_star - Fa_2 # Eq. 3.20

        drone.set_desired(F_a1_star, p, CABLE_1_MAX_L)
        drone.step(data, drone.a_star, drone.a_star_dot)

        prev_p_dot = data.qvel[0:3].copy()
        mujoco.mj_step(model, data)

        # Monitoring only (after mj_step)
        p_ddot       = (data.qvel[0:3] - prev_p_dot) / model.opt.timestep
        Fp_act       = m_payload * (p_ddot + g * e3)
        Fa_2_monitor = Fp_act - F_a1_star
        tau2_monitor = np.linalg.norm(Fa_2_monitor)
        print(f"tau2 control: {tau2:.3f} | tau2 monitor: {tau2_monitor:.3f}")
        viewer.sync()