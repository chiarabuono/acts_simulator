import mujoco
import mujoco.viewer
import numpy as np
import time
from utils_optimization import optimize_drone_positions, compute_payload_jacobian_transpose, check_ground_cable_rubbing
from params_acts import *
from utils_visual import QtWidgets, LiveIndexPlot, LiveErrorPlot, run_tuning_gui
from time import strftime, localtime
from utils_performance_indices import compute_rig_performance_indices, append_robot_data, pose_reached
from video_recorder import VideoRecorder
from acts_simulator import D_SAFE_DRONE, D_SAFE_CABLE, TAU_MIN, TAU_MAX, CHECK_RUB_FREQUENCY, OPTIMIZATION_FREQUENCY, MAX_WINCH_SPEED
from acts_simulator import THRUST_MAX, THRUST_MIN

import tkinter as tk
import threading

def set_cable_length(tendon_idx, max_len):
    if max_len < 0: 
        print(f"Error: Negative length {max_len}")
        return
    winch_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"cable_{tendon_idx}_winch")
    data.ctrl[winch_id] = max_len

def get_cable_length(tendon_idx):
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    return data.ten_length[cable_idx]

def get_cable_tension(model, data, cable_idx):
    if cable_idx in (1, 2, 3):
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"cable_{cable_idx}_tension")
        return data.sensordata[model.sensor_adr[sensor_id]]
    else:
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"cable_{cable_idx}_winch")
        assert actuator_id >= 0, f"actuator 'cable_{cable_idx}_winch' not found in model"
        return -data.actuator_force[actuator_id]

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

def vee(S: np.ndarray) -> np.ndarray:
    """Inverse of the skew-symmetric (hat) map. S must be skew-symmetric."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def compute_Wp_star_geometric(p_payload, R_mat_payload, p_star, R_star):
    """
    SO(3) geometric PD regulator (Lee et al., 2010), for a constant setpoint
    (R_star, p_star), i.e. Omega_star = 0, R_star_dot = 0.
    """
    Kp_pos, Kd_pos = ctrl_params['Kp_pos'], ctrl_params['Kd_pos']
    v_star = np.zeros(3)
    v_payload = data.cvel[payload_id][3:6].copy()
    w_payload_world = data.cvel[payload_id][0:3].copy()

    # --- translational part (unchanged) ---
    F_p_star = (PAYLOAD_MASS * np.array([0.0, 0.0, G_ACCEL])
                + Kp_pos * (p_star - p_payload)
                + Kd_pos * (v_star - v_payload))

    # --- rotational part, geometric on SO(3) ---
    Kr, Kw = ctrl_params['Kr'], ctrl_params['Kw']

    Omega_body = R_mat_payload.T @ w_payload_world       # world -> body frame
    Omega_star_body = np.zeros(3)                        # regulation: Omega_star = 0

    e_R_mat = 0.5 * (R_star.T @ R_mat_payload - R_mat_payload.T @ R_star)
    e_R = vee(e_R_mat)                                    # body-frame attitude error
    e_Omega = Omega_body - Omega_star_body                # body-frame rate error

    M_body = -Kr * e_R - Kw * e_Omega

    M_p_star = R_mat_payload @ M_body                     # body -> world frame

    W_p_star = np.concatenate([F_p_star, M_p_star])
    return W_p_star

# Create Qt Application context once
app = QtWidgets.QApplication.instance()
if app is None:
    app = QtWidgets.QApplication(sys.argv)

index_plot = LiveIndexPlot(max_points=500)
error_plot = LiveErrorPlot(max_points=500)
index_plot.show()
error_plot.show()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

FPS = 30
ERROR_PLOT_FREQUENCY = int(1/(model.opt.timestep * FPS))

step_counter = 0
iteration = 1

p_star, q_star, R_star = read_desired_pose()
set_desired_pose(p_star, q_star)
p_drone_targets_warm = None

is_paused = False

def key_callback(keycode):
    global is_paused
    if keycode == 32:  # Spacebar
        is_paused = not is_paused
        print(f"Simulation {'PAUSED' if is_paused else 'RESUMED'}")

recorder = VideoRecorder(model, fps=15, width=640, height=480)

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    while viewer.is_running():
        step_start = time.time()
        viewer.sync()

        # Keep Qt event loop alive without blocking physics
        app.processEvents()

        if is_paused:
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            continue

        p_star, q_star, R_star = read_desired_pose()
        set_desired_pose(p_star, q_star)
        
        # 1. Capture current payload operational state 
        p_payload = data.body("payload").xpos                
        R_mat_payload = data.xmat[payload_id].reshape(3, 3).copy()   

        # --- LOW FREQUENCY UPDATE: OPTIMIZATION & INDEX PLOTS ---
        if step_counter % OPTIMIZATION_FREQUENCY == 0:
            print(f"{iteration} Computing {strftime('%Y-%m-%d %H:%M:%S', localtime(time.time()))}")

            # W_p_star = compute_Wp_star(p_payload, p_star, q_star)
            W_p_star = compute_Wp_star_geometric(p_payload, R_mat_payload, p_star, R_star)

            app.processEvents()
            p_drone_targets, optimal_tensions = optimize_drone_positions(
                p_payload, R_mat_payload, P_GROUND_ANCHORS, DRONE_MASSES,
                L_CABLES_DRONES, HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND,
                W_p_star, tau_min=TAU_MIN, tau_max=TAU_MAX, d_safe=D_SAFE_DRONE, g=G_ACCEL,
                x0_warm=p_drone_targets_warm
            )
            app.processEvents()
            p_drone_targets_warm = p_drone_targets

            tau = np.array([get_cable_tension(model, data, i) for i in range(1, 10)])
            tau_drone_actual = np.array([get_cable_tension(model, data, i) for i in (1, 2, 3)])
            tau_ground_actual = np.array([get_cable_tension(model, data, i) for i in (4, 5, 6, 7, 8, 9)])

            anchors = list(p_drone_targets) + list(P_GROUND_ANCHORS)
            all_offsets = list(HOOK_OFFSETS_DRONE) + list(HOOK_OFFSETS_GROUND)
            Jp = compute_payload_jacobian_transpose(p_payload, R_mat_payload, anchors, all_offsets)
            M_desired = W_p_star[3:]
            M_actual = (Jp @ tau)[3:]
            
            indices = compute_rig_performance_indices(
                p_payload, R_mat_payload,
                p_drone_targets, P_GROUND_ANCHORS,
                HOOK_OFFSETS_DRONE, HOOK_OFFSETS_GROUND,
                tau_drone_actual, tau_ground_actual,
                W_p_star, PAYLOAD_MASS,
                tau_min_drone=THRUST_MIN,          
                tau_max_drone=THRUST_MAX,   
                w_min_ground=TAU_MIN,       
                w_max_ground=TAU_MAX,       
                g=G_ACCEL,
            )
            
            # UPDATED: Low-Frequency Index Plot
            index_plot.update(data.time, indices)

            if iteration == ITERATION_COLLECTION: 
                pose_params = pose_reached(p_payload, R_mat_payload, p_star, R_star)
                append_robot_data("collected_data/indices.xlsx", FILENAME, p_star, q_star, indices, pose_params)

                print("p_drone_targets:", p_drone_targets)
                print("optimal_tensions (UAV):", optimal_tensions)
                print("tau_ground (measured):", tau_ground_actual)

                print("W_p_generated:", Jp @ tau)   # tau = [*optimal_tensions, *tau_ground_actual]
                print("W_p_star:", W_p_star)

                print(f"{drone1.data.ctrl[drone1.thrust_id]=}")
                print(f"{drone2.data.ctrl[drone2.thrust_id]=}")
                print(f"{drone3.data.ctrl[drone3.thrust_id]=}")
            iteration += 1

        if step_counter % CHECK_RUB_FREQUENCY == 0:
            min_d, ok, pair_dists = check_ground_cable_rubbing(
                    p_payload, R_mat_payload, P_GROUND_ANCHORS, HOOK_OFFSETS_GROUND, d_safe=D_SAFE_CABLE
                    )
            if not ok:
                print(f"Iteration: {iteration}] Cable Rubbing! min_d = {min_d:.3f} < {D_SAFE_CABLE}")

        a1_star, a2_star, a3_star = p_drone_targets[0], p_drone_targets[1], p_drone_targets[2]

        for k in range(6):
            anchor_id = GROUND_ANCHOR_IDS[k]
            p_anchor_global = data.site_xpos[anchor_id] 
            
            b_k_global = p_star + R_star @ HOOK_OFFSETS_GROUND[k]
            rho_star = np.linalg.norm(p_anchor_global - b_k_global)
            
            current_len = get_cable_length(k + 4) 
            dt = model.opt.timestep

            delta = rho_star - current_len
            max_step = MAX_WINCH_SPEED * dt
            step = np.clip(delta, -max_step, max_step)
            new_len = current_len + step
            set_cable_length(k + 4, new_len)

        p_hook1 = p_payload + R_mat_payload @ HOOK_OFFSETS_DRONE[0]
        p_hook2 = p_payload + R_mat_payload @ HOOK_OFFSETS_DRONE[1]
        p_hook3 = p_payload + R_mat_payload @ HOOK_OFFSETS_DRONE[2]

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

        # --- MEDIUM FREQUENCY UPDATE: ERROR PLOTS (e.g. 30 Hz / 30 FPS) ---
        if step_counter % ERROR_PLOT_FREQUENCY == 0:
            error_plot.update(data.time, p_payload, R_mat_payload, p_star, q_star)

        step_counter += 1

        viewer.sync()
        recorder.capture_frame(data)

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        
    recorder.save(f"{VIDEONAME}")
    index_plot.export_image(f"{GRAPHNAME}_indices.png")
    error_plot.export_image(f"{GRAPHNAME}_errors.png")