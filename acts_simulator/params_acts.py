import mujoco
import numpy as np
import tkinter as tk
from utils_control import ACTScontrolDrone

with open("mujoco/acts_stewart.xml", "r") as f:
    xml_model = f.read()

print("Compiling multi-drone payload model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data  = mujoco.MjData(model)

# ------ Payload ------------------------------------------------------------
PAYLOAD_MASS = model.body("payload").mass[0]
payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")

drone1 = ACTScontrolDrone(model, drone_name="drone_1", payload_mass=PAYLOAD_MASS)
drone2 = ACTScontrolDrone(model, drone_name="drone_2", payload_mass=PAYLOAD_MASS)
drone3 = ACTScontrolDrone(model, drone_name="drone_3", payload_mass=PAYLOAD_MASS)

G_ACCEL = np.linalg.norm(model.opt.gravity) 
W_MIN = 5.0                                  
D_SAFE = 0.4


DRONE_MASSES = [
    model.body("drone_1").mass[0],
    model.body("drone_2").mass[0],
    model.body("drone_3").mass[0] ]

L_CABLES_DRONES = [
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_1")][1],
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_2")][1],
    model.tendon_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_3")][1] ]

HOOK_OFFSETS_DRONE = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(1, 4) ]
HOOK_OFFSETS_GROUND = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"hook_{i}")] for i in range(4, 10) ]
P_GROUND_ANCHORS = [model.site_pos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}")] for i in range(4, 10)]

GROUND_ANCHOR_IDS = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"ground_anchor_{i}") for i in range(4, 10)]

CABLE_FILTER_ALPHA = 0.005
OPTIMIZATION_FREQUENCY = 1000

kp = 21.0
ctrl_params = {
    'px': 0.5,
    'py': 0.0,
    'pz': 2.0,
    'Kp_pos' : kp,
    'Kd_pos' : 2*(kp)**0.5
}

R_star = np.eye(3)
# R_star = R.from_quat([q_star[1], q_star[2], q_star[3], q_star[0]]).as_matrix()


