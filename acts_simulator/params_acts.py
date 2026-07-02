import mujoco
import numpy as np
import tkinter as tk
from utils_control import ACTScontrolDrone
from scipy.spatial.transform import Rotation as R

with open("mujoco/acts_stewart.xml", "r") as f:
    xml_model = f.read()

print("Compiling multi-drone payload model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data  = mujoco.MjData(model)

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
W_MIN = 5.0                                  
D_SAFE = 0.4

# ------ Cables  ------------------------------------------------------------
HOOK_OFFSETS_DRONE = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(1, 4) ]
HOOK_OFFSETS_GROUND = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(4, 10) ]
P_GROUND_ANCHORS = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}")] for i in range(4, 10)]
GROUND_ANCHOR_IDS = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}") for i in range(4, 10)]

# ------ Optimization parameters  ------------------------------------------------------------
CABLE_FILTER_ALPHA = 0.05
OPTIMIZATION_FREQUENCY = 1000

kp = 21.0
kr = 50.0
ctrl_params = {
    'px': 0.5,
    'py': 0.0,
    'pz': 2.0,
    'Kp_pos' : kp,
    'Kd_pos' : 2*(kp)**0.5,
    'Kr' : kr,
    'Kw' : 2*(kr)**0.5,
    'quat_w' : 1.0,
    'quat_x' : 0.0,
    'quat_y' : 0.0,
    'quat_z' : 0.0
}


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
