import mujoco
import mujoco.viewer
import numpy as np
import time

# ==========================================
# 1. SYSTEM DEFINITION (MJCF XML format)
# ==========================================
# We define a 1.5kg sphere (drone) and a ground anchor point.
# A native spatial tendon handles the unilateral inelastic cable constraint.
xml_model = """
<mujoco model="section_3_1_1_cable_system">
    <option gravity="0 0 -9.81" timestep="0.002"/>

    <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" rgb2=".2 .3 .4" width="300" height="300"/>
        <material name="grid_mat" texture="grid" texrepeat="10 10"/>
    </asset>

    <worldbody>
        <light pos="0 0 10" dir="0 0 -1"/>
        <geom type="plane" size="10 10 0.1" material="grid_mat"/>

        <site name="anchor_b" pos="0 0 0" size="0.05" rgba="1 0 0 1"/>

        <body name="drone" pos="0 0 0">
            <freejoint name="drone_joint"/>
            <geom type="sphere" size="0.15" rgba="0 0.7 0.9 1" mass="1.5"/>
            <site name="attachment_a" pos="0 0 0" size="0.02" rgba="1 1 0 1"/>
        </body>
    </worldbody>

    <tendon>
        <spatial name="cable_tether" limited="true" range="0 5.0" width="0.02" rgba="0 0.8 0 1">
            <site site="anchor_b"/>
            <site site="attachment_a"/>
        </spatial>
    </tendon>
</mujoco>
"""

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)

# Extract cable length straight from the tendon limit range specified in the XML
CABLE_LENGTH_L = model.tendon_range[0][1] # Evaluates to 5.0 meters

# ==========================================
# 2. CONTROL PARAMETERS (Section 3.1.1)
# ==========================================
m_i = 1.5           # Drone mass (mi)
g = 9.81            # Gravity acceleration
K_p = 25.0          # Controller Proportional Gain (Kp)
K_d = 6.0           # Controller Derivative Gain (Kd)

b = np.array([0.0, 0.0, 0.0])  # Anchor coordinates (b)

# Input: Target Ground Force vector f* requested by the operator
f_star = np.array([4.0, 2.0, 8.0])

# ==========================================
# 3. CONTROLLER & PHYSICS EXECUTION LOOP
# ==========================================
print("Launching Section 3.1.1 MuJoCo Controller...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    # Pre-calculate reference vector constants once (Equations 3.10 to 3.12)
    f_star_norm = np.linalg.norm(f_star)
    tau_star = f_star_norm                               # Eq 3.10
    u_star = f_star / f_star_norm                        # Eq 3.11
    a_star = b + CABLE_LENGTH_L * u_star                 # Eq 3.12
    a_star_dot = np.array([0.0, 0.0, 0.0])               # Static tracking target (\dot{a}* = 0)
    
    # Track metrics to observe the tension spike during a snap taut event
    max_tension_observed = 0.0

    while viewer.is_running():
        step_start = time.time()

        # A. Read Current State from the physics environment
        a = data.xpos[1]      # Spatial Position of the drone (a)
        a_dot = data.qvel[0:3] # Spatial Linear Velocity of the drone (\dot{a})
        
        # Calculate real-time cable unit vector (Equation 3.9)
        vec_cable = a - b
        dist_cable = np.linalg.norm(vec_cable)
        u = vec_cable / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0]) # Eq 3.9

        # B. Apply Propulsion Control Law (Equation 3.13)
        gravity_compensation = m_i * g * np.array([0.0, 0.0, 1.0])           # m_i * g * e3
        pd_feedback = K_p * (a_star - a) + K_d * (a_star_dot - a_dot)         # Kp(a*-a) + Kd(\dot{a}*-\dot{a})
        feedforward_tension = tau_star * u                                   # \tau* * u
        
        F_prop = gravity_compensation + pd_feedback + feedforward_tension     # Eq 3.13

        # C. Inject control force into MuJoCo's generalized forces array (qfrc_applied)
        data.qfrc_applied[0:3] = F_prop

        # D. Advance Physics Step
        mujoco.mj_step(model, data)
        
        # E. Read internal constraint forces to see the true cable tension (including the snap)
        # MuJoCo maps contact/constraint forces into the 'efc_force' array
        if data.efc_force.size > 0:
            cable_tension = -data.efc_force[0] # Constraint force acts negatively to pull back
            if cable_tension > max_tension_observed:
                max_tension_observed = cable_tension

        # F. Synchronize 3D UI display
        viewer.sync()

        # G. Pacing loop to stay close to real-time execution speeds
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\n--- Simulation Summary ---")
print(f"Target Drone Position (a*): {a_star}")
print(f"Final Drone Position (a):  {data.xpos[1]}")
print(f"Max Taut Constraint Force (Snap Spike): {max_tension_observed:.2f} Newtons")