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

CABLE_LENGTH_L = model.tendon_range[0][1]
drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")

# ── Physical constants read directly from your XML ──────────────────────────
m_i  = model.body(drone_id).mass[0]                
I_xx = model.body(drone_id).inertia[0]            
I_yy = model.body(drone_id).inertia[1]             
I_zz = model.body(drone_id).inertia[2]             
g    = np.abs(model.opt.gravity[2])                
dt   = model.opt.timestep                          

# ── Quadrotor motor parameters ───────────────────────────────────────────────
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "drone")
L_arm = model.geom_size[geom_id][0]

# kt: each motor must support  (m*g / 4)  at hover
# hover rpm ≈ 8000 → ω_hover ≈ 838 rad/s  →  kt = (m*g/4) / ω_hover²
omega_hover = 838.0                                 
kt = (m_i * g / 4.0) / (omega_hover ** 2)         
kd = 0.016 * kt                                    # torque/thrust ratio (typical)

# Maximum motor speed (2× hover for headroom)
omega_max_sq = (2.0 * omega_hover) ** 2

# ── Allocation matrix  mixer : ω² → [F, τ_roll, τ_pitch, τ_yaw] ────────────────
# Motor positions (top view):
#   1: front-left  (+x, +y)  CCW  →  +roll, +pitch, +yaw
#   2: front-right (+x, -y)  CW   →  -roll, +pitch, -yaw
#   3: rear-right  (-x, -y)  CCW  →  -roll, -pitch, +yaw
#   4: rear-left   (-x, +y)  CW   →  +roll, -pitch, -yaw
mixer = np.array([
    [ kt,        kt,        kt,        kt       ],  # total thrust
    [ kt*L_arm, -kt*L_arm, -kt*L_arm,  kt*L_arm],  # roll  τ_x
    [ kt*L_arm,  kt*L_arm, -kt*L_arm, -kt*L_arm],  # pitch τ_y
    [ kd,       -kd,        kd,        -kd      ],  # yaw   τ_z
])
inv_mixer = np.linalg.pinv(mixer)

gains = {
    'Kp_pos': 2.1,  'Kd_pos': 2.8,
    'Kp_att': 11.0,  'Kd_att': 6.6,
}

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
                     command=lambda v, k=key: gains.update({k: float(v)}))
        s.set(gains[key])
        s.pack(fill='x', padx=10)
    root.mainloop()

threading.Thread(target=run_tuning_gui, daemon=True).start()



def rot_to_euler_zyx(R):
    """ZYX Euler angles (roll φ, pitch θ, yaw ψ) from 3x3 rotation matrix."""
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    roll  = np.arctan2(R[2, 1], R[2, 2])
    yaw   = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])

def desired_R_from_force(F_des, yaw_des=0.0):
    F_norm = np.linalg.norm(F_des)
    if F_norm < 1e-6:
        return np.array([0.0, 0.0, yaw_des])
    z_des = F_des / F_norm
    x_c   = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
    y_des = np.cross(z_des, x_c)
    norm_y = np.linalg.norm(y_des)

    if norm_y < 1e-6:          # degenerate: z_des ≈ x_c, use fallback
        x_c = np.array([0.0, 1.0, 0.0])
        y_des = np.cross(z_des, x_c)
        norm_y = np.linalg.norm(y_des)
    y_des /= norm_y
    x_des  = np.cross(y_des, z_des)

    return np.column_stack([x_des, y_des, z_des])

def desired_euler_from_force(F_des, yaw_des=0.0):
    R_des = desired_R_from_force(F_des, yaw_des=0.0)
    return rot_to_euler_zyx(R_des)

def desired_quat_from_force(F_des, yaw_des=0.0):
    R_des = desired_R_from_force(F_des, yaw_des=0.0)
    return R.from_matrix(R_des).as_quat()


def position_controller(a, a_dot, a_star, a_star_dot, body_z):
    f = (gains['Kp_pos'] * (a_star - a) + gains['Kd_pos'] * (a_star_dot - a_dot)) + m_i * np.array([0.0, 0.0, g])
    F_total = max(0.0, np.dot(f, body_z))
    quat_des = desired_quat_from_force(f, yaw_des=0.0)

    return F_total, quat_des

def attitude_controller(desired_quat, current_quat, R_current):
    angular_vel = R_current.T @ data.qvel[3:6]

    current_q = R.from_quat(current_quat)
    des_q = R.from_quat(desired_quat)

    error_q_obj = current_q.inv() * des_q
    error_q = error_q_obj.as_quat() # Vector structure: [x, y, z, w]

    sign = 1.0 if error_q[3] >= 0 else -1.0

    I_vec   = np.array([I_xx, I_yy, I_zz])
    tau_des = -gains['Kd_att'] * angular_vel + gains['Kp_att'] * (sign * error_q[:3])
    return tau_des * I_vec
    

print("Launching simulation…")
b      = np.array([0.0, 0.0, 0.0])     # cable anchor
f_star = np.array([4.0, 2.0, 8.0])     # desired cable force vector

f_norm   = np.linalg.norm(f_star)
u_star   = f_star / f_norm              # Eq 3.11
a_star   = b + CABLE_LENGTH_L * u_star  # Eq 3.12
a_star_dot = np.zeros(3)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # Current state
        a     = data.xpos[drone_id].copy()
        a_dot = data.qvel[0:3].copy()
        
        R_curr_mat = data.xmat[drone_id].reshape(3, 3)
        body_z  = R_curr_mat[:, 2]
        current_quat = R.from_matrix(R_curr_mat).as_quat()


        F_total, quat_des = position_controller(a, a_dot, a_star, a_star_dot, body_z)
        tau_des = attitude_controller(quat_des, current_quat, R_curr_mat)

        # ── Motor mixing & saturation ────────────────────────────────────
        wrench   = np.array([F_total, tau_des[0], tau_des[1], tau_des[2]])
        omega_sq = np.clip(inv_mixer @ wrench, 0.0, omega_max_sq)

        # Reconstruct actual wrench after clipping
        w_act = mixer @ omega_sq
        F_act = w_act[0]
        tau_act = w_act[1:4]

        data.xfrc_applied[drone_id, 0:3] = F_act * body_z
        data.xfrc_applied[drone_id, 3:6] = R_curr_mat @ tau_act

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - step_start
        remaining = dt - elapsed
        if remaining > 0:
            time.sleep(remaining)