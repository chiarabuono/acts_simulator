import mujoco
import mujoco.viewer
import numpy as np
import time
from utils_optimization import *
from params_acts import *
import threading
from time import strftime, localtime

def set_cable_length(tendon_idx, max_len):
    if max_len < 0: 
        print(f"Error: Negative length {max_len}")
        return
        
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    model.tendon_range[cable_idx, 1] = max_len

def get_cable_length(tendon_idx):
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    return model.tendon_range[cable_idx, 1]


def run_tuning_gui():
    root = tk.Tk()
    root.title("Section 3.1.3 Position Target Mixer")
    root.geometry("360x700")

    def update_val(key, val):
        ctrl_params[key] = float(val)

    tk.Label(root, text="Desired Payload Coordinates (p*)", font=('Helvetica', 10, 'bold')).pack(pady=5)

    tk.Label(root, text="Target X Position").pack()
    s_px = tk.Scale(root, from_=-3.0, to=3.0, resolution=0.05, orient='horizontal',
                    command=lambda v: update_val('px', v))
    s_px.set(ctrl_params['px'])
    s_px.pack(fill='x', padx=10)

    tk.Label(root, text="Target Y Position").pack()
    s_py = tk.Scale(root, from_=-2.0, to=2.0, resolution=0.05, orient='horizontal',
                    command=lambda v: update_val('py', v))
    s_py.set(ctrl_params['py'])
    s_py.pack(fill='x', padx=10)

    tk.Label(root, text="Target Z Position (Height)").pack()
    s_pz = tk.Scale(root, from_=0.5, to=5.0, resolution=0.05, orient='horizontal',
                    command=lambda v: update_val('pz', v))
    s_pz.set(ctrl_params['pz'])
    s_pz.pack(fill='x', padx=10)

    tk.Label(root, text="Kp Payload").pack()
    s_kp = tk.Scale(root, from_=0.5, to=40.0, resolution=0.05, orient='horizontal',
                    command=lambda v: update_val('kp', v))
    s_kp.set(ctrl_params['Kp_pos'])
    s_kp.pack(fill='x', padx=10)

    tk.Label(root, text="Kd payload").pack()
    s_kd = tk.Scale(root, from_=0.5, to=40.0, resolution=0.05, orient='horizontal',
                    command=lambda v: update_val('kd', v))
    s_kd.set(ctrl_params['Kd_pos'])
    s_kd.pack(fill='x', padx=10)

    root.mainloop()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

def quaternon_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def compute_Wp_star(p_payload, p_star):
    Kp_pos, Kd_pos = ctrl_params['Kp_pos'], ctrl_params['Kd_pos']
    v_star = np.zeros(3)
    v_payload = data.cvel[payload_id][3:6].copy()                 
    w_payload = data.cvel[payload_id][0:3].copy() 
    F_p_star = PAYLOAD_MASS * (np.array([0.0, 0.0, G_ACCEL])) + Kp_pos * (p_star - p_payload) + Kd_pos * (v_star - v_payload)
    
    Kr, Kw = 15.0, 5.0

    q = data.body("payload").xquat
    q_star = np.array([1.0, 0.0, 0.0, 0.0])
    q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
    q_err = quaternon_multiply(q_inv, q_star)
    sign = np.sign(q_err[0]) if q_err[0] != 0 else 1.0

    omega_star = np.zeros(3)

    e_r = sign * q_err[1:4]
    e_w = omega_star - w_payload
    M_p_star = Kr * e_r +  Kw * e_w
    W_p_star = np.concatenate([F_p_star, M_p_star])

    return W_p_star

step_counter = 0
iteration = 1

p_star = np.array([ctrl_params['px'], ctrl_params['py'], ctrl_params['pz']])
data.mocap_pos[0] = p_star
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        viewer.sync()
        p_star = np.array([ctrl_params['px'], ctrl_params['py'], ctrl_params['pz']])
        data.mocap_pos[0] = p_star
        
        # 1. Capture current payload operational state 
        p_payload = data.body("payload").xpos                
        R_mat_payload = data.xmat[payload_id].reshape(3, 3).copy()   

        if step_counter % OPTIMIZATION_FREQUENCY == 0:
            print(f"{iteration} Computing {strftime('%Y-%m-%d %H:%M:%S', localtime(time.time()))}")
            W_p_star = compute_Wp_star(p_payload, p_star)

            p_drone_targets, optimal_tensions = optimize_drone_positions(
                p_star, R_star, P_GROUND_ANCHORS, DRONE_MASSES, 
                L_CABLES_DRONES, HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND, 
                W_p_star, w_min=W_MIN, d_safe=D_SAFE, g=G_ACCEL
            )
            iteration += 1
        step_counter += 1

        a1_star, a2_star, a3_star = p_drone_targets[0], p_drone_targets[1], p_drone_targets[2]

        for k in range(6):
            anchor_id = GROUND_ANCHOR_IDS[k]
            p_anchor_global = data.site_xpos[anchor_id] 
            
            b_k_global = p_star + R_star @ HOOK_OFFSETS_GROUND[k]
            rho_star = np.linalg.norm(p_anchor_global - b_k_global)
            
            current_len = get_cable_length(k + 4) 
            new_len = current_len + CABLE_FILTER_ALPHA * (rho_star - current_len)
            set_cable_length(k + 4, new_len)

        p_hook1 = p_star + R_mat_payload @ HOOK_OFFSETS_DRONE[0]
        p_hook2 = p_star + R_mat_payload @ HOOK_OFFSETS_DRONE[1]
        p_hook3 = p_star + R_mat_payload @ HOOK_OFFSETS_DRONE[2]

        drone1.set_cable_target(optimal_tensions[0], p_hook1)
        drone2.set_cable_target(optimal_tensions[1], p_hook2)
        drone3.set_cable_target(optimal_tensions[2], p_hook3)

        drone1.apply_wrench(a1_star)
        drone2.apply_wrench(a2_star)
        drone3.apply_wrench(a3_star) 

        mujoco.mj_step(model, data)
        drone1.update_data(data)
        drone2.update_data(data)
        drone3.update_data(data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)