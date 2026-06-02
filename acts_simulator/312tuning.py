import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk

# ==========================================
# 1. SYSTEM DEFINITION (MJCF XML format)
# ==========================================
xml_model = """
<mujoco model="section_3_1_2_interactive_system">
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid_mat" texture="grid" texrepeat="10 10"/>
    </asset>

    <worldbody>
        <light pos="0 0 10" dir="0 0 -1"/>
        <geom type="plane" size="10 10 0.1" material="grid_mat"/>

        <body name="target_marker" mocap="true" pos="1 3 4">
            <geom type="sphere" size="0.06" rgba="1 0.5 0 0.4" contype="0" conaffinity="0"/>
        </body>

        <site name="ground_anchor" pos="0 0 0" size="0.05" rgba="1 0 0 1"/>

        <body name="payload" pos="0 0 2.5">
            <freejoint name="payload_joint"/>
            <geom type="sphere" size="0.08" rgba="0.9 0.1 0.1 1" mass="1.0"/>
            <site name="payload_top" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
            <site name="payload_bottom" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
        </body>

        <body name="drone" pos="0 0 5.5">
            <freejoint name="drone_joint"/>
            <geom type="sphere" size="0.15" rgba="0 0.7 0.9 1" mass="2.0"/>
            <site name="drone_attachment" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>
        </body>
    </worldbody>

    <tendon>
        <spatial name="cable_2" limited="true" range="0 5.0" width="0.015" rgba="0.7 0.7 0.7 1">
            <site site="ground_anchor"/>
            <site site="payload_bottom"/>
        </spatial>
        
        <spatial name="cable_1" limited="true" range="0 4.0" width="0.015" rgba="0 0.8 0 1">
            <site site="payload_top"/>
            <site site="drone_attachment"/>
        </spatial>
    </tendon>
</mujoco>
"""

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

CABLE_1_MAX_L = model.tendon_range[1][1]
m_drone = 2.0       
m_payload = 1.0     
g = 9.81            
e3 = np.array([0.0, 0.0, 1.0])
a2 = np.array([0.0, 0.0, 0.0])

# Runtime Tuning Parameters
gains = {
    'Kp_p': 1.0,
    'Kd_p': 1.0,
    'Kp_d': 6.0,
    'Kd_d': 2.0
}

# ==========================================
# 2. SEPARATE GUI WINDOW THREAD (Tkinter)
# ==========================================
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

# Spin up the slider window in the background
gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

# ==========================================
# 3. CONTROLLER & PHYSICS EXECUTION LOOP
# ==========================================
print("Launching Section 3.1.2 Controller. Use the separate window to adjust gains!")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        viewer.sync() 
        
        p_star = np.copy(data.mocap_pos[0])
        dist_from_anchor = np.linalg.norm(p_star)
        # if dist_from_anchor >= 4.95:
        #     p_star = (p_star / dist_from_anchor) * 4.95
        data.mocap_pos[0] = p_star
            
        p_star_dot = np.array([0.0, 0.0, 0.0])
        p_star_ddot = np.array([0.0, 0.0, 0.0])

        # A. Read current states
        p = data.xpos[2]         # Index 2 = payload body
        p_dot = data.qvel[0:3]   
        
        a1 = data.xpos[3]        # Index 3 = drone body
        a1_dot = data.qvel[6:9]  

        # B. Get current Ground Cable 2 unit vector and force (Fa,2)
        vec_cable2 = p - a2
        dist_cable2 = np.linalg.norm(vec_cable2)
        u2 = vec_cable2 / dist_cable2 if dist_cable2 > 0.001 else e3
        
        tau2 = -data.efc_force[0] if data.efc_force.size > 0 else 0.0
        tau2 = max(0.0, tau2)    
        Fa_2 = tau2 * u2

        # C. Apply Eq. 3.19 using live thread-safe dictionary updates
        F_p_star = m_payload * (p_star_ddot + g * e3) + \
                   gains['Kp_p'] * (p_star - p) + gains['Kd_p'] * (p_star_dot - p_dot)

        # D. Apply Eq. 3.20: Remaining force allocation
        F_a1_star = F_p_star - Fa_2
        
        # E. Map target force vector to geometric position for drone (Eq. 3.12)
        F_a1_star_norm = np.linalg.norm(F_a1_star)
        tau1_star = F_a1_star_norm
        u1_star = F_a1_star / F_a1_star_norm if F_a1_star_norm > 0.001 else e3
        
        a1_star = p + CABLE_1_MAX_L * u1_star
        a1_star_dot = np.array([0.0, 0.0, 0.0]) 

        # F. Apply Eq. 3.13 with real-time slider outputs
        vec_cable1 = a1 - p
        dist_cable1 = np.linalg.norm(vec_cable1)
        u1 = vec_cable1 / dist_cable1 if dist_cable1 > 0.001 else e3

        F_prop = m_drone * g * e3 + \
                 gains['Kp_d'] * (a1_star - a1) + gains['Kd_d'] * (a1_star_dot - a1_dot) + \
                 tau1_star * u1

        # G. Inject calculated control force into drone Degrees of Freedom
        data.qfrc_applied[6:9] = F_prop

        # H. Step forward physics
        mujoco.mj_step(model, data)

        # I. Frame pacing
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)