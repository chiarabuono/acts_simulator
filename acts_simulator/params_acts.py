import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R
from acts_simulator.utils_control import ACTScontrolDrone
from acts_simulator.utils_configuration_selection import select_and_load_xml

import os, sys
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)
from acts_simulator import kr_xy, kp


FILENAME, xml_model = select_and_load_xml()
if xml_model:
    print(f"--> Target Loaded Successfully! Active Key: {FILENAME}")
else:
    print("--> Configuration load aborted or canceled.")

print("Compiling multi-drone payload model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# ------ Payload ------------------------------------------------------------
PAYLOAD_MASS = model.body("payload").mass[0]
payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")

# ------ Drones  ------------------------------------------------------------
drone1 = ACTScontrolDrone(model, drone_name="drone_1", payload_mass=PAYLOAD_MASS)
drone2 = ACTScontrolDrone(model, drone_name="drone_2", payload_mass=PAYLOAD_MASS)
drone3 = ACTScontrolDrone(model, drone_name="drone_3", payload_mass=PAYLOAD_MASS)

DRONE_MASSES = [
    model.body("drone_1").mass[0],
    model.body("drone_2").mass[0],
    model.body("drone_3").mass[0] ]

L_CABLES_DRONES = [
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_1")][1],
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_2")][1],
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_3")][1] ]

# ------ Global variables ------------------------------------------------------------
G_ACCEL = np.linalg.norm(model.opt.gravity)

# ------ Cables  ------------------------------------------------------------
HOOK_OFFSETS_DRONE = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(1, 4)]
HOOK_OFFSETS_GROUND = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(4, 10)]
P_GROUND_ANCHORS = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}")] for i in range(4, 10)]
GROUND_ANCHOR_IDS = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}") for i in range(4, 10)]

# ------ Optimization parameters  ------------------------------------------------------------
RENDER_EVERY_N_STEPS = 50
ITERATION_COLLECTION = 50  # Iteration at which indices are collected

payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
i_xx, i_yy, i_zz = model.body_inertia[payload_id]

# 3. Calculate gains using extracted values
inertia_ratio = i_zz / i_xx

kr_z = kr_xy * inertia_ratio
ctrl_params = {
    'px': 0.5, 'py': -0.5, 'pz': 2.0,
    'Kp_pos': kp, 
    'Kd_pos': 2 * (kp)**0.5,
    'Kr': np.array([kr_xy, kr_xy, kr_z]),
    'Kw': np.array([2 * (kr_xy)**0.5, 2 * (kr_xy)**0.5, 2 * (kr_z)**0.5]),
    'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0
}

from acts_simulator import MODE
VIDEONAME = f"{FILENAME}_{MODE}_{ctrl_params['px']}-{ctrl_params['py']}-{ctrl_params['pz']}_{ctrl_params['quat_w']}-{ctrl_params['quat_x']}-{ctrl_params['quat_y']}-{ctrl_params['quat_z']}.mp4"
GRAPHNAME = f"{FILENAME}_{MODE}_{ctrl_params['px']}-{ctrl_params['py']}-{ctrl_params['pz']}_{ctrl_params['quat_w']}-{ctrl_params['quat_x']}-{ctrl_params['quat_y']}-{ctrl_params['quat_z']}"

# ------ Desired pose parameters  ------------------------------------------------------------
def read_desired_pose():
    p_star = np.array([ctrl_params['px'], ctrl_params['py'], ctrl_params['pz']])
    q_star = np.array([ctrl_params["quat_w"], ctrl_params["quat_x"], ctrl_params["quat_y"], ctrl_params["quat_z"]])

    q_scipy_format = [q_star[1], q_star[2], q_star[3], q_star[0]]
    R_star = R.from_quat(q_scipy_format).as_matrix()

    return p_star, q_star, R_star


# ------ Set desired pose  ------------------------------------------------------------
def set_desired_pose(p_star, q_star):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_marker")
    mocap_id = model.body_mocapid[body_id]

    data.mocap_pos[0] = p_star
    data.mocap_quat[mocap_id, :] = q_star