import os
import sys
import json
import tkinter as tk
from tkinter import messagebox, ttk


# --- PATH CONFIGURATIONS ---
IMAGE_PATH_UGV = "create_xml/images/6_ugv_config.jpeg"
IMAGE_PATH_UAV = "create_xml/images/3_uav_config.png"

UGV_DB_PATH = "create_xml/database/ugv_configuration_database.json"
UAV_DB_PATH = "create_xml/database/uav_configuration_database.json"

# --- UGV BOUNDING CROP BOXES ---
GRID_MAPPING_UGV = {
    "line": (19, 16, 305, 382), "diagonal": (380, 14, 665, 385), "triangle": (745, 15, 1028, 382),
    "rectangle-same": (19, 410, 327, 785), "rectangle-opposite": (382, 405, 705, 792), "rhombus-ext": (737, 408, 1058, 800),
    "rhombus-int": (1084, 410, 1405, 793), "parallelepiped-ext": (1435, 401, 1751, 780), "parallelepiped-int": (17, 828, 354, 1208),
    "trapezoid-ext": (385, 827, 708, 1203), "trapezoid-int": (747, 826, 1071, 1207), "pentagon": (6, 1238, 330, 1643),
    "arrow": (360, 1235, 668, 1638), "cross": (695, 1232, 1010, 1634), "central-trapezoid": (1029, 1231, 1362, 1643),
    "wave": (1379, 1232, 1711, 1643), "base-trapezoid": (11, 1670, 300, 2038), "central-parallelepiped": (348, 1649, 687, 2050),
    "2X3": (6, 2070, 284, 2457), "two-triangles": (340, 2068, 627, 2458), "vertical-parallelepiped": (650, 2064, 1022, 2473),
    "pyramid": (1040, 2065, 1328, 2455), "horizzontal-parallelepiped": (1351, 2065, 1756, 2443), "3X2": (2, 2492, 288, 2887),
    "hexagon": (320, 2498, 617, 2876),
}

# --- UAV BOUNDING CROP BOXES ---
GRID_MAPPING_UAV = {
            "point": (24, 14, 203, 177),
            "aligned": (230, 11, 426, 175),
            "disaligned": (444, 6, 669, 179),
            "line": (16, 215, 203, 364),
            "triangle": (240, 213, 432, 366),
            "diagonal": (478, 192, 699, 371),
        }

# --- MUJOCO CORE XML STRING TEMPLATE ---
MUJOCO_TEMPLATE = """<mujoco model="acts">
    <compiler angle="radian" autolimits="true"/>
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <visual>
        <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8"/>
        <rgba haze="0.15 0.25 0.35 1"/>
    </visual>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.4 0.6 0.8" rgb2="0.0 0.0 0.0" width="512" height="512"/>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid_mat" texture="grid" texrepeat="10 10"/>
        <material name="payload_mat" rgba="0.6 0.4 0.2 1"/>
    </asset>

    <worldbody>
        <light pos="0 0 10" dir="0 0 -1"/>
        <geom name="ground" type="plane" size="10 10 0.1" material="grid_mat" condim="3" friction="1 0.005 0.0001"/>
        <body name="target_marker" mocap="true" pos="0.0 0 1.0" euler="0 0 0">
            <site name="target_com" type="sphere" size="0.05" rgba="1 0 0 0.6" group="1"/>
            <geom type="box" size="1.0 1.0 0.1" rgba="1 0.5 0 0.25" contype="0" conaffinity="0" group="1"/>
        </body>
        <body name="payload" pos="0 0 0.11">
            <freejoint name="payload_joint"/>
            <inertial pos="0 0 0" mass="1.0" diaginertia="0.3467 0.3467 0.6667"/>
            <geom name="payload_geom" type="box" size="1.0 1.0 0.1" 
                material="payload_mat" condim="3" friction="1 0.005 0.0001"
                rgba="0.6 0.4 0.2 0.35" />
            <site name="payload_com" pos="0 0 0" size="0.06" rgba="1 0 0 1" type="sphere"/>
{payload_sites}        </body>
{uav_bodies_string}
{ground_sites}    </worldbody>
    <tendon>
{uav_tendons_string}
{tendon_elements}    </tendon>
    <actuator>
{uav_actuators_string}
{actuator_elements}    </actuator>
    <sensor>
{uav_sensors_string}
{sensor_elements}    </sensor>
</mujoco>"""

 
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)
 
from acts_simulator import TAU_MIN, TAU_MAX, D_SAFE, W_MIN, PAYLOAD_HALF_EXTENTS

def _load_json_db(path):
    if os.path.exists(path):
        with open(path, "r") as f:
            return json.load(f)
    messagebox.showerror("Missing File", f"Could not find '{path}'. Run annotations first!")
    return None