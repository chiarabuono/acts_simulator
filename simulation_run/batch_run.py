import os
import sys
import glob
import time
import numpy as np
import pandas as pd
import mujoco
from time import strftime, localtime
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------
# Path Configurations
# -----------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))               
_SIMULATOR_PKG = os.path.dirname(_SCRIPT_DIR)                          
_SRC_DIR = os.path.dirname(_SIMULATOR_PKG)                             

for p in [_SIMULATOR_PKG, _SRC_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from acts_simulator import max_thrust
from acts_simulator.utils_control import ACTScontrolDrone
from acts_simulator.utils_optimization import optimize_drone_positions, check_ground_cable_rubbing
from acts_simulator.utils_performance_indices import compute_rig_performance_indices, append_robot_data, pose_reached
from acts_simulator import D_SAFE_DRONE, D_SAFE_CABLE, TAU_MIN, TAU_MAX, OPTIMIZATION_FREQUENCY, MAX_WINCH_SPEED, CHECK_RUB_FREQUENCY
from acts_simulator.utils_configuration_selection import select_and_load_folder

import os, sys

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)

# -----------------------------------------------------------------------
# Configuration Parameters
# -----------------------------------------------------------------------
FOLDER_NAME, FOLDER_PATH = select_and_load_folder()
MODELS_FOLDER = os.path.join(_PROJECT_ROOT, FOLDER_PATH)
POSES_EXCEL = os.path.join(_PROJECT_ROOT, "simulation_run/batch_run.xlsx")
OUTPUT_EXCEL = "simulation_run/results.xlsx"


MAX_ITERATIONS = 50               # Max limit changed from 30 to 50
POS_TOLERANCE = 0.1              # Target position error threshold (in meters)
ROT_TOLERANCE = np.deg2rad(3.0)              # Target orientation error threshold (in radians or norm)


# Global container to track rubbing events across all poses
rubbing_events = []

# -----------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def vee(S: np.ndarray) -> np.ndarray:
    return np.array([S[2, 1], S[0, 2], S[1, 0]])

def compute_Wp_star_geometric(model, data, payload_id, payload_mass, p_payload, R_mat_payload, p_star, R_star, ctrl_params, g_accel):
    Kp_pos, Kd_pos = ctrl_params['Kp_pos'], ctrl_params['Kd_pos']
    v_star = np.zeros(3)
    v_payload = data.cvel[payload_id][3:6].copy()
    w_payload_world = data.cvel[payload_id][0:3].copy()

    F_p_star = (payload_mass * np.array([0.0, 0.0, g_accel])
                + Kp_pos * (p_star - p_payload)
                + Kd_pos * (v_star - v_payload))

    Kr, Kw = ctrl_params['Kr'], ctrl_params['Kw']
    Omega_body = R_mat_payload.T @ w_payload_world
    Omega_star_body = np.zeros(3)

    e_R_mat = 0.5 * (R_star.T @ R_mat_payload - R_mat_payload.T @ R_star)
    e_R = vee(e_R_mat)
    e_Omega = Omega_body - Omega_star_body

    M_body = -Kr * e_R - Kw * e_Omega
    M_p_star = R_mat_payload @ M_body
    return np.concatenate([F_p_star, M_p_star])

def set_cable_length(model, data, tendon_idx, max_len):
    if max_len < 0: return
    winch_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"cable_{tendon_idx}_winch")
    data.ctrl[winch_id] = max_len

def get_cable_length(model, data, tendon_idx):
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    return data.ten_length[cable_idx]

def get_cable_tension(model, data, cable_idx):
    if cable_idx in (1, 2, 3):
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"cable_{cable_idx}_tension")
        return data.sensordata[model.sensor_adr[sensor_id]]
    else:
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"cable_{cable_idx}_winch")
        return -data.actuator_force[actuator_id]


def run_simulation_for_pose(xml_path, pose_row, pose_idx):
    filename = os.path.splitext(os.path.basename(xml_path))[0]
    
    # 1. Load Model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 2. Extract System Properties
    payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
    payload_mass = model.body("payload").mass[0]
    g_accel = np.linalg.norm(model.opt.gravity)

    drone1 = ACTScontrolDrone(model, drone_name="drone_1", payload_mass=payload_mass)
    drone2 = ACTScontrolDrone(model, drone_name="drone_2", payload_mass=payload_mass)
    drone3 = ACTScontrolDrone(model, drone_name="drone_3", payload_mass=payload_mass)

    drone_masses = [model.body(f"drone_{i}").mass[0] for i in range(1, 4)]
    l_cables_drones = [
        model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{i}")][1]
        for i in range(1, 4)
    ]

    hook_offsets_drone = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(1, 4)]
    hook_offsets_ground = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(4, 10)]
    p_ground_anchors = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}")] for i in range(4, 10)]
    ground_anchor_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}") for i in range(4, 10)]

    # 3. Target Pose Setups
    p_star = np.array([pose_row['pos_x'], pose_row['pos_y'], pose_row['pos_z']])
    q_star = np.array([pose_row['quat_w'], pose_row['quat_x'], pose_row['quat_y'], pose_row['quat_z']])
    q_scipy_format = [q_star[1], q_star[2], q_star[3], q_star[0]]
    R_star = R.from_quat(q_scipy_format).as_matrix()

    # Controller Gains
    i_xx, _, i_zz = model.body_inertia[payload_id]
    inertia_ratio = i_zz / i_xx
    kp = 28.0
    kr_xy = 8.0
    kr_z = kr_xy * inertia_ratio

    ctrl_params = {
        'Kp_pos': kp, 
        'Kd_pos': 2 * (kp)**0.5,
        'Kr': np.array([kr_xy, kr_xy, kr_z]),
        'Kw': np.array([2 * (kr_xy)**0.5, 2 * (kr_xy)**0.5, 2 * (kr_z)**0.5])
    }

    # Set Mocap Target
    marker_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    mocap_id = model.body_mocapid[marker_id]
    data.mocap_pos[0] = p_star
    data.mocap_quat[mocap_id, :] = q_star

    step_counter = 0
    iteration = 1
    p_drone_targets_warm = None
    dt = model.opt.timestep

    # 4. Simulation Loop with Early Convergence Exit
    converged = 0
    while iteration <= MAX_ITERATIONS:
        p_payload = data.body("payload").xpos.copy()             
        R_mat_payload = data.xmat[payload_id].reshape(3, 3).copy()

        # Low-Frequency Optimization & Error Check Step
        if step_counter % OPTIMIZATION_FREQUENCY == 0:
            print(f"[Iter {iteration}/{MAX_ITERATIONS}] Evaluating pose at {strftime('%H:%M:%S', localtime())}")
            
            W_p_star = compute_Wp_star_geometric(
                model, data, payload_id, payload_mass, 
                p_payload, R_mat_payload, p_star, R_star, 
                ctrl_params, g_accel
            )

            p_drone_targets, optimal_tensions = optimize_drone_positions(
                p_payload, R_mat_payload, p_ground_anchors, drone_masses,
                l_cables_drones, hook_offsets_drone, hook_offsets_ground,
                W_p_star, tau_min=TAU_MIN, tau_max=TAU_MAX, d_safe=D_SAFE_DRONE, g=g_accel,
                x0_warm=p_drone_targets_warm
            )
            p_drone_targets_warm = p_drone_targets

            # Evaluate Pose Error using pose_reached
            pose_params = pose_reached(p_payload, R_mat_payload, p_star, R_star)
            
            pos_error = pose_params.get('position_error')
            rot_error = pose_params.get('orientation_error')
            
            is_converged = (pos_error <= POS_TOLERANCE) and (rot_error <= ROT_TOLERANCE)
            is_max_reached = (iteration == MAX_ITERATIONS)
            
            if is_converged: converged += 1
            if converged > 5 or is_max_reached:
                reason = "CONVERGED" if is_converged else "MAX ITERATIONS REACHED"
                print(f" Exit condition met ({reason}) at iteration {iteration}. Pos error: {pos_error:.4f} m.")

                tau_drone_actual = np.array([get_cable_tension(model, data, i) for i in (1, 2, 3)])
                tau_ground_actual = np.array([get_cable_tension(model, data, i) for i in range(4, 10)])

                indices = compute_rig_performance_indices(
                    p_payload, R_mat_payload,
                    p_drone_targets, p_ground_anchors,
                    hook_offsets_drone, hook_offsets_ground,
                    tau_drone_actual, tau_ground_actual,
                    W_p_star, payload_mass,
                )
                
                # --- ADD ITERATION & REASON TO EXCEL OUTPUT ---
                pose_params['final_iteration'] = iteration
                pose_params['stop_reason'] = reason
                
                # Save to batch_run.xlsx
                append_robot_data(OUTPUT_EXCEL, filename, p_star, q_star, indices, pose_params)
                print(f" Saved metrics for XML: {filename} (Stopped @ Iteration {iteration}).")
                break  # Exit loop to proceed to next pose

            iteration += 1

        # Cable Rubbing Detection
        if step_counter % CHECK_RUB_FREQUENCY == 0:
            min_d, ok, pair_dists = check_ground_cable_rubbing(
                p_payload, R_mat_payload, p_ground_anchors, hook_offsets_ground, d_safe=D_SAFE_CABLE
            )
            if not ok:
                print(f"[Model: {filename} | Iteration: {iteration}] Cable Rubbing! min_d = {min_d:.3f} < {D_SAFE_CABLE}")
                rubbing_events.append({
                    "model_xml": filename,
                    "pose_index": pose_idx,
                    "target_pos_x": p_star[0],
                    "target_pos_y": p_star[1],
                    "target_pos_z": p_star[2],
                    "iteration": iteration,
                    "step_counter": step_counter,
                    "simulation_time_s": step_counter * dt,
                    "min_distance_m": min_d,
                    "d_safe_threshold_m": D_SAFE_CABLE,
                    "violation_margin_m": min_d - D_SAFE_CABLE,
                    "status": "RUBBING_DETECTED"
                })

        a1_star, a2_star, a3_star = p_drone_targets[0], p_drone_targets[1], p_drone_targets[2]

        # Winch Control Strategy
        for k in range(6):
            anchor_id = ground_anchor_ids[k]
            p_anchor_global = data.site_xpos[anchor_id]
            b_k_global = p_star + R_star @ hook_offsets_ground[k]
            rho_star = np.linalg.norm(p_anchor_global - b_k_global)

            current_len = get_cable_length(model, data, k + 4)
            delta = rho_star - current_len
            max_step = MAX_WINCH_SPEED * dt
            step = np.clip(delta, -max_step, max_step)
            set_cable_length(model, data, k + 4, current_len + step)

        p_hook1 = p_payload + R_mat_payload @ hook_offsets_drone[0]
        p_hook2 = p_payload + R_mat_payload @ hook_offsets_drone[1]
        p_hook3 = p_payload + R_mat_payload @ hook_offsets_drone[2]

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

        step_counter += 1


# -----------------------------------------------------------------------
# Execution Block
# -----------------------------------------------------------------------
if __name__ == "__main__":
    poses_df = pd.read_excel(POSES_EXCEL)
    poses_df.columns = poses_df.columns.str.strip()  # Clean whitespace from Excel headers
    
    xml_files = glob.glob(os.path.join(MODELS_FOLDER, "*.xml"))

    print(f"Found {len(xml_files)} XML configuration files.")
    print(f"Loaded {len(poses_df)} target poses from Excel.")

    for xml_path in xml_files:
        xml_name = os.path.basename(xml_path)
        print(f"\n==========================================")
        print(f"Processing Model File: {xml_name}")
        print(f"==========================================")

        for idx, pose_row in poses_df.iterrows():
            print(f"--> [Pose {idx+1}/{len(poses_df)}] Target: [{pose_row['pos_x']}, {pose_row['pos_y']}, {pose_row['pos_z']}]")
            run_simulation_for_pose(xml_path, pose_row, pose_idx=idx + 1)

    print("\n All batch jobs successfully processed!")

    # Save cable rubbing events
    if rubbing_events:
        df_rubbing = pd.DataFrame(rubbing_events)
        df_rubbing.to_csv("ground_cable_rubbing_log.csv", index=False)
        with pd.ExcelWriter("ground_cable_rubbing_log.xlsx", engine="openpyxl") as writer:
            df_rubbing.to_excel(writer, sheet_name="Cable_Rubbing_Events", index=False)
        print(f"\n Logged {len(df_rubbing)} cable rubbing events to 'ground_cable_rubbing_log.xlsx' and '.csv'")
    else:
        print("\n No ground cable rubbing detected across all batch runs.")