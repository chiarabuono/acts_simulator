import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk

with open("mujoco/311_model_real.xml", "r") as f:
    xml_model = f.read()

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)


CABLE_LENGTH_L = model.tendon_range[0][1]

drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")
m_i = model.body(drone_id).mass[0] # Drone mass (mi)
g = 9.81
K_p = 25.0
K_d = 6.0

gains = {
    'K_p': 6.0,
    'K_d': 2.0
}

def run_tuning_gui():
    root = tk.Tk()
    root.title("Gain Tuner (Section 3.1.2)")
    root.geometry("320x280")
    
    def update_val(key, val):
        gains[key] = float(val)

    tk.Label(root, text="Drone Tuning", font=('Helvetica', 10, 'bold')).pack(pady=5)
    
    tk.Label(root, text="Drone Kp").pack()
    s1 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kp_p', v))
    s1.set(gains['K_p'])
    s1.pack(fill='x', padx=10) # Fixed px -> padx

    tk.Label(root, text="Drone Kd").pack()
    s2 = tk.Scale(root, from_=0, to=20, orient='horizontal', command=lambda v: update_val('Kd_p', v))
    s2.set(gains['K_d'])
    s2.pack(fill='x', padx=10) # Fixed px -> padx


    root.mainloop()

# Spin up the slider window in the background
gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

b = np.array([0.0, 0.0, 0.0])  # Anchor coordinates (b)
f_star = np.array([4.0, 2.0, 8.0])

print("Launching Section 3.1.1 MuJoCo Controller...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    f_star_norm = np.linalg.norm(f_star)
    tau_star = f_star_norm                               # Eq 3.10
    u_star = f_star / f_star_norm                        # Eq 3.11
    a_star = b + CABLE_LENGTH_L * u_star                 # Eq 3.12
    a_star_dot = np.array([0.0, 0.0, 0.0])               
    
    max_tension_observed = 0.0

    while viewer.is_running():
        step_start = time.time()

        # Current state
        a = data.xpos[drone_id].copy()
        a_dot = data.qvel[0:3].copy()
        
        dist_cable = np.linalg.norm(a - b)
        u = (a - b) / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0])

        gravity_compensation = m_i * g * np.array([0.0, 0.0, 1.0])
        pd_feedback = gains['K_p'] * (a_star - a) + gains['K_d'] * (a_star_dot - a_dot)
        feedforward_tension = tau_star * u
        F_prop_world = gravity_compensation + pd_feedback + feedforward_tension     # Eq 3.13

        R = data.xmat[drone_id].reshape(3, 3)
        body_z = R[:, 2]

        thrust_scalar = np.dot(F_prop_world, body_z)
        thrust_scalar = max(0.0, thrust_scalar)

        data.xfrc_applied[drone_id, 0:3] = thrust_scalar * body_z
        data.xfrc_applied[drone_id, 3:6] = np.zeros(3)

        mujoco.mj_step(model, data)

        # Read cable tension
        if data.efc_force.size > 0:
            cable_tension = -data.efc_force[0]
            if cable_tension > max_tension_observed:
                max_tension_observed = cable_tension

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)