import mujoco
import mujoco.viewer
import numpy as np
import time

with open("mujoco/simpler_cases/311_model.xml", "r") as f:
    xml_model = f.read()

print("Compiling MuJoCo model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data = mujoco.MjData(model)


CABLE_LENGTH_L = model.tendon_range[0][1] 
m_i = model.body("drone").mass[0] 
g = 9.81            
K_p = 6.0          
K_d = 2.0           

b = np.array([0.0, 0.0, 0.0])
f_star = np.array([4.0, 2.0, 8.0])

print("Launching Section 3.1.1 MuJoCo Controller...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    
    f_star_norm = np.linalg.norm(f_star)
    tau_star = f_star_norm                               # Eq 3.10
    u_star = f_star / f_star_norm                        # Eq 3.11
    a_star = b + CABLE_LENGTH_L * u_star                 # Eq 3.12
    a_star_dot = np.array([0.0, 0.0, 0.0])            
    
    max_tension_observed = 0.0

    while viewer.is_running():
        step_start = time.time()

        # Current State from the physics environment
        a = data.xpos[1]      
        a_dot = data.qvel[0:3]
        
        dist_cable = np.linalg.norm(a - b)
        u = (a - b) / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0]) # Eq 3.9

        gravity_compensation = m_i * g * np.array([0.0, 0.0, 1.0])        
        pd_feedback = K_p * (a_star - a) + K_d * (a_star_dot - a_dot)        
        feedforward_tension = tau_star * u                                   
        
        F_prop = gravity_compensation + pd_feedback + feedforward_tension     # Eq 3.13

        data.qfrc_applied[0:3] = F_prop

        mujoco.mj_step(model, data)
        
        # cable tension (including the snap)
        if data.efc_force.size > 0:
            cable_tension = -data.efc_force[0]
            if cable_tension > max_tension_observed:
                max_tension_observed = cable_tension

        viewer.sync()

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\n--- Simulation Summary ---")
print(f"Target Drone Position (a*): {a_star}")
print(f"Final Drone Position (a):  {data.xpos[1]}")
print(f"Max Taut Constraint Force (Snap Spike): {max_tension_observed:.2f} Newtons")