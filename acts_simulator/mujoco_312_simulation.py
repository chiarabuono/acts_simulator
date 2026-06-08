import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk

with open("mujoco/312_model.xml", "r") as f:
    xml_model = f.read()

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

CABLE_1_MAX_L = model.tendon_range[1][1]
CABLE_2_MAX_L = model.tendon_range[0][1]

m_payload = model.body("payload").mass[0]
m_drone = model.body("drone").mass[0]  
g = 9.81            
e3 = np.array([0.0, 0.0, 1.0])
a2 = np.array([0.0, 0.0, 0.0])

gains = {
    'Kp_p': 1.0,
    'Kd_p': 1.0,
    'Kp_d': 6.0,
    'Kd_d': 2.0
}

def run_tuning_gui():
    root = tk.Tk()
    root.title("Gain Tuner (Section 3.1.2)")
    root.geometry("320x280")
    
    def update_val(key, val):
        gains[key] = float(val)

    tk.Label(root, text="Payload Position Tuning", font=('Helvetica', 10, 'bold')).pack(pady=5)
    
    tk.Label(root, text="Payload Kp").pack()
    s1 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kp_p', v))
    s1.set(gains['Kp_p'])
    s1.pack(fill='x', padx=10) # Fixed px -> padx

    tk.Label(root, text="Payload Kd").pack()
    s2 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kd_p', v))
    s2.set(gains['Kd_p'])
    s2.pack(fill='x', padx=10) # Fixed px -> padx

    tk.Label(root, text="Drone Inner Loop Tuning", font=('Helvetica', 10, 'bold')).pack(pady=5)

    tk.Label(root, text="Drone Kp").pack()
    s3 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kp_d', v))
    s3.set(gains['Kp_d'])
    s3.pack(fill='x', padx=10) # Fixed px -> padx

    tk.Label(root, text="Drone Kd").pack()
    s4 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kd_d', v))
    s4.set(gains['Kd_d'])
    s4.pack(fill='x', padx=10) # Fixed px -> padx

    root.mainloop()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()


MODE = "force_control"
print(f"Launching Section 3.1.2 Controller. MODE {MODE}")


with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        viewer.sync() 
        if MODE == "payload_position_control":
            p_star = np.copy(data.mocap_pos[0])
        elif MODE == "force_control":
            f_star = np.array([1.0, 2.0, 10.0])
            f_star_norm = np.linalg.norm(f_star)
            tau_star = f_star_norm                               # Eq 3.10
            u2_star = f_star / f_star_norm                       # Eq 3.11            
            p_star = a2 + CABLE_2_MAX_L * u2_star                # Eq 3.21

        dist_from_anchor = np.linalg.norm(p_star)
        data.mocap_pos[0] = p_star
            
        p_star_dot = np.array([0.0, 0.0, 0.0])
        p_star_ddot = np.array([0.0, 0.0, 0.0])

        # A. Read current states
        p = data.xpos[2]         # Index 2 = payload body
        p_dot = data.qvel[0:3]   
        
        a1 = data.xpos[3]        # Index 3 = drone body
        a1_dot = data.qvel[6:9]  

        # (Fa,2)
        vec_cable2 = p - a2
        dist_cable2 = np.linalg.norm(vec_cable2)
        u2 = vec_cable2 / dist_cable2 if dist_cable2 > 0.001 else e3
        
        tau2 = -data.efc_force[0] if data.efc_force.size > 0 else 0.0
        tau2 = max(0.0, tau2)    
        Fa_2 = tau2 * u2

        F_p_star = m_payload * (p_star_ddot + g * e3) + \
                   gains['Kp_p'] * (p_star - p) + gains['Kd_p'] * (p_star_dot - p_dot) # Eq. 3.19

        F_a1_star = F_p_star - Fa_2 # Eq. 3.20
        
        # Eq. 3.12
        F_a1_star_norm = np.linalg.norm(F_a1_star)
        tau1_star = F_a1_star_norm
        u1_star = F_a1_star / F_a1_star_norm if F_a1_star_norm > 0.001 else e3
        
        a1_star = p + CABLE_1_MAX_L * u1_star
        a1_star_dot = np.array([0.0, 0.0, 0.0]) 

        # Eq. 3.13
        vec_cable1 = a1 - p
        dist_cable1 = np.linalg.norm(vec_cable1)
        u1 = vec_cable1 / dist_cable1 if dist_cable1 > 0.001 else e3

        F_prop = m_drone * g * e3 + \
                 gains['Kp_d'] * (a1_star - a1) + gains['Kd_d'] * (a1_star_dot - a1_dot) + \
                 tau1_star * u1

        data.qfrc_applied[6:9] = F_prop

        mujoco.mj_step(model, data)

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)