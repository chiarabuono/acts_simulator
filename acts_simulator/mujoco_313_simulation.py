import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk

with open("mujoco/313_model.xml", "r") as f:
    xml_model = f.read()

print("Compiling Section 3.1.3 MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# Tendon limit constraints
CABLE_1_MAX_L = model.tendon_range[2][1]
CABLE_2_MAX_L = model.tendon_range[0][1]  # Maximum length of ground cable 2
CABLE_3_MAX_L = model.tendon_range[1][1]  # Maximum length of ground cable 3

# Masses
m_payload = model.body("payload").mass[0]
m_drone   = model.body("drone").mass[0]

g  = 9.81
e3 = np.array([0.0, 0.0, 1.0])

# Anchor absolute world coordinate placements
a2 = model.site("ground_anchor_1").pos.copy()
a3 = model.site("ground_anchor_2").pos.copy()

# Dynamic Parameter
ctrl_params = {
    'px': 0.3,
    'py': 0.0,
    'pz': 2.2,
    'Kp_p': 1.0,
    'Kd_p': 2*(m_payload*1.0)**0.5,       
    'Kp_d': 6.0,
    'Kd_d': 2*(m_drone*6.0)**0.5,
    'K_virtual': 1.0   # Stiffness constant for out-of-reach tension generation
}

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

    tk.Label(root, text="Case 2 Tuning Parameters", font=('Helvetica', 10, 'bold')).pack(pady=10)

    tk.Label(root, text="Virtual Tension Gain (K)").pack()
    s_kv = tk.Scale(root, from_=0, to=100, resolution=0.05, orient='horizontal', command=lambda v: update_val('K_virtual', v))
    s_kv.set(ctrl_params['K_virtual'])
    s_kv.pack(fill='x', padx=10)

    tk.Label(root, text="Loop Controllers Tuning Panel", font=('Helvetica', 10, 'bold')).pack(pady=10)

    tk.Label(root, text="Payload Outer Loop Kp").pack()
    s1 = tk.Scale(root, from_=0, to=40, resolution=0.05, orient='horizontal', command=lambda v: update_val('Kp_p', v))
    s1.set(ctrl_params['Kp_p'])
    s1.pack(fill='x', padx=10)

    tk.Label(root, text="Payload Outer Loop Kd").pack()

    s3 = tk.Scale(root, from_=0, to=40, resolution=0.05, orient='horizontal', command=lambda v: update_val('Kd_p', v))
    s3.set(ctrl_params['Kd_p'])
    s3.pack(fill='x', padx=10)

    tk.Label(root, text="Drone Inner Position Loop Kp").pack()
    s2 = tk.Scale(root, from_=0, to=40, resolution=0.05, orient='horizontal', command=lambda v: update_val('Kp_d', v))
    s2.set(ctrl_params['Kp_d'])
    s2.pack(fill='x', padx=10)

    tk.Label(root, text="Drone Inner Position Loop Kd").pack()
    s4 = tk.Scale(root, from_=0, to=40, resolution=0.05, orient='horizontal', command=lambda v: update_val('Kd_d', v))
    s4.set(ctrl_params['Kd_d'])
    s4.pack(fill='x', padx=10)

    root.mainloop()

gui_thread = threading.Thread(target=run_tuning_gui, daemon=True)
gui_thread.start()

def get_cable_tensions(model, data):
    """
    Extract scalar tensions for ground cable tendons by name
    Returns (tau2, tau3) — non-negative tension scalars for cable 2 and 3.
    """
    tau2 = 0.0
    tau3 = 0.0

    tendon_idx_cable2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_2")
    tendon_idx_cable3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "cable_3")

    for i in range(data.nefc):
        # efc_type == 3  →  mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON
        if data.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON:
            tidx = data.efc_id[i]          # which tendon triggered this row
            force = -data.efc_force[i]     # sign convention: positive = tension
            if tidx == tendon_idx_cable2:
                tau2 = max(0.0, force)
            elif tidx == tendon_idx_cable3:
                tau3 = max(0.0, force)

    return tau2, tau3


print("Running Section 3.1.3 Multi-Case Tracking Controller.")

case = 0
def switch_case(case_to_switch):
    global case
    if case != case_to_switch:
        case = case_to_switch
        if case == 1: desc = "p* lies on (or very close to) the intersecting boundary"
        elif case == 2: desc = "p* is out of reach — ground cables block the payload."
        elif case == 3: desc = "p* is inside the workspace triangle — cables are slack."
        else:
            print("No accepted case")
            return
        print(f"SWITCH CASE {case}: {desc}")

prev_a1_star     = None
prev_step_time   = None

with mujoco.viewer.launch_passive(model, data) as viewer:

    payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
    drone_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone")


    while viewer.is_running():
        step_start = time.time()
        viewer.sync()

        # Desired target pose
        p_star = np.array([ctrl_params['px'], ctrl_params['py'], ctrl_params['pz']])
        data.mocap_pos[0] = p_star   # Synchronize translucent visualizer sphere

        p_star_dot   = np.zeros(3)
        p_star_ddot  = np.zeros(3)

        # Actual system state  
        p     = data.xpos[payload_id].copy() # Payload body global coordinate
        p_dot = data.qvel[0:3].copy()

        a1    = data.xpos[drone_id].copy()     # Drone body global coordinate
        a1_dot = data.qvel[6:9].copy()

        # Compute cable forces
        vec_c2  = p - a2
        dist_c2 = np.linalg.norm(vec_c2)
        u2 = vec_c2 / dist_c2 if dist_c2 > 0.001 else e3

        vec_c3  = p - a3
        dist_c3 = np.linalg.norm(vec_c3)
        u3 = vec_c3 / dist_c3 if dist_c3 > 0.001 else e3

        tau2, tau3 = get_cable_tensions(model, data)

        Fa_2 = tau2 * u2
        Fa_3 = tau3 * u3

        # Impedance control for desired payload wrench
        F_p_star = (m_payload * (p_star_ddot + g * e3)
                    + ctrl_params['Kp_p'] * (p_star - p)
                    + ctrl_params['Kd_p'] * (p_star_dot - p_dot))
        
        # E. MULTI-CASE GEOMETRIC SELECTION LAYER
        target_dist_c2 = np.linalg.norm(p_star - a2)
        target_dist_c3 = np.linalg.norm(p_star - a3)
        eps = 0.05   # Tolerance for boundary contact

        inside_c2 = target_dist_c2 <= CABLE_2_MAX_L
        inside_c3 = target_dist_c3 <= CABLE_3_MAX_L

        on_boundary_c2 = abs(target_dist_c2 - CABLE_2_MAX_L) <= eps
        on_boundary_c3 = abs(target_dist_c3 - CABLE_3_MAX_L) <= eps


        if inside_c2 or inside_c3:
            switch_case(3)
            # ----------------------------------------------------------------
            # CASE 3: p* is inside the workspace triangle — cables are slack.
            # ----------------------------------------------------------------
            
            F_a1_star = F_p_star

        elif on_boundary_c2 or on_boundary_c3:
            switch_case(1)
            # ----------------------------------------------------------------
            # CASE 1: p* lies on (or very close to) the intersecting boundary
            # arcs.  F*_{a,1} = F*_p − Σ F_{a,i}
            # ----------------------------------------------------------------
            F_a1_star = F_p_star - (Fa_2 + Fa_3)
            

        else:
            switch_case(2)
            # ----------------------------------------------------------------
            # CASE 2: p* is out of reach — ground cables block the payload.
            # ----------------------------------------------------------------
            u2_target = (p_star - a2) / target_dist_c2 if target_dist_c2 > 0.001 else e3
            u3_target = (p_star - a3) / target_dist_c3 if target_dist_c3 > 0.001 else e3

            p_geom_c2 = a2 + CABLE_2_MAX_L * u2_target
            p_geom_c3 = a3 + CABLE_3_MAX_L * u3_target

            # Select the binding cable (the one most over its limit)
            excess_c2 = target_dist_c2 - CABLE_2_MAX_L
            excess_c3 = target_dist_c3 - CABLE_3_MAX_L

            if excess_c2 >= excess_c3:
                p_geom = p_geom_c2
            else:
                p_geom = p_geom_c3

            F_virtual = ctrl_params['K_virtual'] * (p_star - p_geom)
            F_a1_star = F_p_star - (Fa_2 + Fa_3) + F_virtual

        # Decompose F*_{a,1} into scalar tension and unit direction
        F_a1_star_norm = np.linalg.norm(F_a1_star)
        tau1_star = F_a1_star_norm
        u1_star   = F_a1_star / F_a1_star_norm if F_a1_star_norm > 0.001 else e3
        a1_star = p + CABLE_1_MAX_L * u1_star

        now = time.time()
        if prev_a1_star is not None and prev_step_time is not None:
            dt_real = now - prev_step_time
            if dt_real > 1e-9:
                a1_star_dot = (a1_star - prev_a1_star) / dt_real
            else:
                a1_star_dot = np.zeros(3)
        else:
            a1_star_dot = np.zeros(3)

        prev_a1_star   = a1_star.copy()
        prev_step_time = now

        # Apply drone propulsion force (Eq. 3.13)
        vec_cable1  = a1 - p
        dist_cable1 = np.linalg.norm(vec_cable1)
        u1 = vec_cable1 / dist_cable1 if dist_cable1 > 0.001 else e3

        cable1_taut = dist_cable1 > 0.99 * CABLE_1_MAX_L

        F_prop = (m_drone * g * e3
                + ctrl_params['Kp_d'] * (a1_star - a1)
                + ctrl_params['Kd_d'] * (a1_star_dot - a1_dot)
                + (tau1_star * u1 if cable1_taut else np.zeros(3)))
        
        data.qfrc_applied[6:9] = F_prop

        mujoco.mj_step(model, data)
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)