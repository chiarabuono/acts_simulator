import mujoco
import mujoco.viewer
import numpy as np
import time

# ==========================================
# 1. SYSTEM DEFINITION (MJCF XML format)
# ==========================================
xml_model = """
<mujoco model="section_3_1_2_interactive_system">
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid_mat" texture="grid" texrepeat="10 10"/>
    </asset>

    <worldbody>
        <light pos="0 0 10" dir="0 0 -1"/>
        <geom type="plane" size="10 10 0.1" material="grid_mat"/>

        <body name="target_marker" mocap="true" pos="1 3 4">
            <geom type="sphere" size="0.06" rgba="1 0.5 0 0.4" contype="0" conaffinity="0"/>
        </body>

        <site name="ground_anchor" pos="0 0 0" size="0.05" rgba="1 0 0 1"/>

        <body name="payload" pos="0 0 2.5">
            <freejoint name="payload_joint"/>
            <geom type="sphere" size="0.08" rgba="0.9 0.1 0.1 1" mass="1.0"/>
            <site name="payload_top" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
            <site name="payload_bottom" pos="0 0 0" size="0.01" rgba="1 1 1 1"/>
        </body>

        <body name="drone" pos="0 0 5.5">
            <freejoint name="drone_joint"/>
            <geom type="sphere" size="0.15" rgba="0 0.7 0.9 1" mass="2.0"/>
            <site name="drone_attachment" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>
        </body>
    </worldbody>

    <tendon>
        <spatial name="cable_2" limited="true" range="0 5.0" width="0.015" rgba="0.7 0.7 0.7 1">
            <site site="ground_anchor"/>
            <site site="payload_bottom"/>
        </spatial>
        
        <spatial name="cable_1" limited="true" range="0 4.0" width="0.015" rgba="0 0.8 0 1">
            <site site="payload_top"/>
            <site site="drone_attachment"/>
        </spatial>
    </tendon>
</mujoco>
"""

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# Extract tendon limit constraint for Cable 1
CABLE_1_MAX_L = model.tendon_range[1][1]  # 4.0 meters

# ==========================================
# 2. CONTROL PARAMETERS (Section 3.1.2)
# ==========================================
m_drone = 2.0       # Drone mass (m1)
m_payload = 1.0     # Payload mass (mp)
g = 9.81            # Gravity constant
e3 = np.array([0.0, 0.0, 1.0])

# Impedance Controller Gains (Eq. 3.19)
Kp_p = 1.0         # Proportional Gain for payload position tracking
Kd_p = 1.0         # Derivative Gain for payload velocity tracking

# Drone Tracking Gains (Eq. 3.13)
Kp_d = 6.0        # Proportional Gain for drone position tracking
Kd_d = 2.0         # Derivative Gain for drone velocity tracking

# Ground anchor coordinates (fixed origin)
a2 = np.array([0.0, 0.0, 0.0])

print("Launching Section 3.1.2 MuJoCo Cascade Controller...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    while viewer.is_running():
        step_start = time.time()

        USE_TRAJECTORY = False  

        if not USE_TRAJECTORY:
            # Mode A: Static Final Goal Position
            # Define your constant 3D target coordinates here
            p_star = np.array([2.879738394, 0.3, 0.5])
            
            # Since the target is fixed, its derivatives must be identically zero
            p_star_dot = np.array([0.0, 0.0, 0.0])
            p_star_ddot = np.array([0.0, 0.0, 0.0])
            
        else:
            # Mode B: Time-Varying Trajectory
            t = data.time
            radius = 0.5
            omega = 1.2
            
            p_star = np.array([
                radius * np.cos(omega * t),
                radius * np.sin(omega * t),
                2.2
            ])
            p_star_dot = np.array([
                -radius * omega * np.sin(omega * t),
                 radius * omega * np.cos(omega * t),
                 0.0
            ])
            p_star_ddot = np.array([
                -radius * (omega**2) * np.cos(omega * t),
                -radius * (omega**2) * np.sin(omega * t),
                 0.0
            ])

        # A. Read current states from the physics engine
        p = data.xpos[1]         # Actual position of payload
        p_dot = data.qvel[0:3]   # Actual velocity of payload

        print(f"Error payload position: {p_star} - {p} = {p_star - p}")
        
        a1 = data.xpos[2]        # Actual position of drone
        a1_dot = data.qvel[6:9]  # Actual velocity of drone

        # B. Get current Ground Cable 2 unit vector and force (Fa,2)
        vec_cable2 = p - a2
        dist_cable2 = np.linalg.norm(vec_cable2)
        u2 = vec_cable2 / dist_cable2 if dist_cable2 > 0.001 else e3
        
        # Get scalar tension tau2 from solver constraints (index 0)
        tau2 = -data.efc_force[0] if data.efc_force.size > 0 else 0.0
        tau2 = max(0.0, tau2)    # Unilateral limit enforcement
        Fa_2 = tau2 * u2

        # C. Apply Eq. 3.19: Desired payload force via Impedance Control
        F_p_star = m_payload * (p_star_ddot + g * e3) + \
                   Kp_p * (p_star - p) + Kd_p * (p_star_dot - p_dot)

        # D. Apply Eq. 3.20: Remaining force to be generated by cable 1
        F_a1_star = F_p_star - Fa_2
        
        # E. Map the desired force vector to a geometric target position for the drone (Eq. 3.10, 3.11, 3.12)
        F_a1_star_norm = np.linalg.norm(F_a1_star)
        tau1_star = F_a1_star_norm
        u1_star = F_a1_star / F_a1_star_norm if F_a1_star_norm > 0.001 else e3
        
        # Desired drone position reference
        a1_star = p + CABLE_1_MAX_L * u1_star
        a1_star_dot = np.array([0.0, 0.0, 0.0]) # Assumed target velocity profile for step-cascade

        # print(f"Error drone position {a1_star} - {a1} = {a1_star - a1}")

        # F. Apply Eq. 3.13: Controller output for the drone propulsion force
        vec_cable1 = a1 - p
        dist_cable1 = np.linalg.norm(vec_cable1)
        u1 = vec_cable1 / dist_cable1 if dist_cable1 > 0.001 else e3

        F_prop = m_drone * g * e3 + Kp_d * (a1_star - a1) + Kd_d * (a1_star_dot - a1_dot) + tau1_star * u1

        # G. Inject calculated control force into drone Degrees of Freedom
        data.qfrc_applied[6:9] = F_prop

        # H. Step forward physics and sync viewer
        mujoco.mj_step(model, data)
        viewer.sync()

        # I. Frame pacing
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)