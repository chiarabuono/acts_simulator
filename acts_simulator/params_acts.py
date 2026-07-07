import mujoco
import numpy as np
import tkinter as tk
from utils_control import ACTScontrolDrone
from scipy.spatial.transform import Rotation as R
import os
import glob
from tkinter import ttk, messagebox

# ----- Select dynamically the xml file -------------------------------------------------
def select_and_load_xml():
    target_dir = "mujoco"
    
    if not os.path.exists(target_dir):
        messagebox.showerror("Error", f"Directory '{target_dir}' does not exist.")
        return None
        
    xml_files = [os.path.basename(f) for f in glob.glob(f"{target_dir}/*.xml")]
    
    if not xml_files:
        messagebox.showinfo("Empty Directory", f"No XML configuration files found in '{target_dir}'.")
        return None
    
    result = {"content": None, "filename": None}

    ui = tk.Tk()
    ui.title("Select MuJoCo Configuration Target")
    ui.geometry("500x350")
    
    lbl = ttk.Label(ui, text="Choose a layout configuration to initialize:", font=("Arial", 11, "bold"))
    lbl.pack(anchor=tk.W, padx=15, pady=(15, 5))

    list_frame = ttk.Frame(ui)
    list_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=5)
    
    scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL)
    listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, font=("Arial", 10))
    scrollbar.config(command=listbox.yview)
    
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Populate files sorted alphabetically
    for file in sorted(xml_files):
        listbox.insert(tk.END, file)

    def on_confirm():
        selection = listbox.curselection()
        if not selection:
            messagebox.showwarning("Selection Required", "Please highlight an XML file configuration first.")
            return
            
        chosen_file = listbox.get(selection[0])
        full_path = os.path.join(target_dir, chosen_file)
        
        try:
            with open(full_path, "r") as f:
                result["content"] = f.read()
                result["filename"] = chosen_file.replace(".xml", "")
            ui.destroy()
        except IOError as e:
            messagebox.showerror("File Read Error", f"Could not load system data:\n{e}")

    btn_load = ttk.Button(ui, text="Load Configuration Model", command=on_confirm)
    btn_load.pack(fill=tk.X, padx=15, pady=15)
    listbox.bind("<Double-1>", lambda event: on_confirm())
    ui.mainloop()
    
    return result["filename"], result["content"]

FILENAME, xml_model = select_and_load_xml()

if xml_model:
    print(f"--> Target Loaded Successfully! Active Key: {FILENAME}")
    # Proceed with your simulation engine using 'xml_model'
else:
    print("--> Configuration load aborted or canceled.")

with open(f"mujoco/{FILENAME}.xml", "r") as f:
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
ITERATION_COLLECTION = 20 # Iteration at which indices are collected

kp = 21.0
kr = 50.0
ctrl_params = {
    'px': 1.0,
    'py': -0.5,
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
