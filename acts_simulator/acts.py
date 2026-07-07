import mujoco
import mujoco.viewer
import numpy as np
import time
from utils_optimization import *
from params_acts import *
from utils_visual import * 
from time import strftime, localtime
from utils_performance_indices import compute_rig_performance_indices, append_robot_data

def set_cable_length(tendon_idx, max_len):
    if max_len < 0: 
        print(f"Error: Negative length {max_len}")
        return
    winch_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"cable_{tendon_idx}_winch")
    data.ctrl[winch_id] = max_len

def get_cable_length(tendon_idx):
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    return data.ten_length[cable_idx]

def get_cable_tension(model, data, tendon_name):
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{tendon_name}_tension")
    return data.sensordata[model.sensor_adr[sensor_id]]

def quaternon_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def compute_Wp_star(p_payload, p_star, q_star):
    Kp_pos, Kd_pos = ctrl_params['Kp_pos'], ctrl_params['Kd_pos']
    v_star = np.zeros(3)
    v_payload = data.cvel[payload_id][3:6].copy()                 
    w_payload = data.cvel[payload_id][0:3].copy() 
    F_p_star = PAYLOAD_MASS * (np.array([0.0, 0.0, G_ACCEL])) + Kp_pos * (p_star - p_payload) + Kd_pos * (v_star - v_payload)
    
    Kr, Kw = ctrl_params['Kr'], ctrl_params['Kw']

    q = data.body("payload").xquat
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

p_star, q_star, R_star = read_desired_pose()
set_desired_pose(p_star, q_star)

is_paused = False

def key_callback(keycode):
    global is_paused
    # 32 is the keycode for the Spacebar
    if keycode == 32:
        is_paused = not is_paused
        print(f"Simulation {'PAUSED' if is_paused else 'RESUMED'}")

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()
        viewer.sync()

        if is_paused:
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            continue  # Skips optimization loops and physics updates completely

        p_star, q_star, R_star = read_desired_pose()
        set_desired_pose(p_star, q_star)
        
        # 1. Capture current payload operational state 
        p_payload = data.body("payload").xpos                
        R_mat_payload = data.xmat[payload_id].reshape(3, 3).copy()   

        if step_counter % OPTIMIZATION_FREQUENCY == 0:
            print(f"{iteration} Computing {strftime('%Y-%m-%d %H:%M:%S', localtime(time.time()))}")
            W_p_star = compute_Wp_star(p_payload, p_star, q_star)

            p_drone_targets, optimal_tensions = optimize_drone_positions(
                p_star, R_star, P_GROUND_ANCHORS, DRONE_MASSES,
                L_CABLES_DRONES, HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND,
                W_p_star, w_min=W_MIN, d_safe=D_SAFE, g=G_ACCEL
            )

            tau_drone_actual = np.array([get_cable_tension(model, data, f"cable_{i}") for i in (1, 2, 3)])
            tau_ground_actual = np.array([get_cable_tension(model, data, f"cable_{i}") for i in (4, 5, 6, 7, 8, 9)])

            indices = compute_rig_performance_indices(
                p_payload, R_mat_payload,
                p_drone_targets, P_GROUND_ANCHORS,
                HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND,
                tau_drone_actual, tau_ground_actual,
                W_p_star, PAYLOAD_MASS,
            )
            plot.update(data.time, indices)
            if iteration == ITERATION_COLLECTION: append_robot_data("indices.xlsx", FILENAME, p_star, q_star, indices)
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