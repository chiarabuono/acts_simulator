import mujoco
import mujoco.viewer
import numpy as np
import time

with open("mujoco/acts_model.xml", "r") as f:
    xml_model = f.read()

print("Compiling multi-drone payload model...")
model = mujoco.MjModel.from_xml_string(xml_model)
data  = mujoco.MjData(model)

# ── Body IDs ──────────────────────────────────────────────────────────────────
payload_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "payload")
drone_1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_1")
drone_2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_2")
drone_3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "drone_3")

print(f"payload_id={payload_id}, drone_1_id={drone_1_id}, "
      f"drone_2_id={drone_2_id}, drone_3_id={drone_3_id}")

# ── DOF address for qvel ──────────────────────────────────────────────────────
# Each freejoint contributes 6 DOFs: 3 translational + 3 rotational
# qvel layout: [payload(0:6), drone_1(6:12), drone_2(12:18), drone_3(18:24)]
payload_dof = model.body(payload_id).dofadr[0]
drone_1_dof = model.body(drone_1_id).dofadr[0]
drone_2_dof = model.body(drone_2_id).dofadr[0]
drone_3_dof = model.body(drone_3_id).dofadr[0]

print(f"DOF addresses — payload:{payload_dof}, drone_1:{drone_1_dof}, "
      f"drone_2:{drone_2_dof}, drone_3:{drone_3_dof}")

# ── Constants ─────────────────────────────────────────────────────────────────
g        = 9.81
e3       = np.array([0.0, 0.0, 1.0])
F_HOVER  = 25.0

print(f"Each drone applying {F_HOVER} N upward.")

def set_ground_cable_length(tendon_idx, max_len):
    """
    tendon_idx: from 4 to 9 as 1 to 3 are for drones
    """
    if max_len < 0: 
        print(f"Negative len")
        return
    model.tendon_range[tendon_idx, 1]
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    model.tendon_range[cable_idx, 1] = max_len

def get_ground_cable_length(tendon_idx):
    cable_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"cable_{tendon_idx}")
    return model.tendon_range[cable_idx, 1]

with mujoco.viewer.launch_passive(model, data) as viewer:
    len = get_ground_cable_length(4)

    while viewer.is_running():
        step_start = time.time()

        R1 = data.xmat[drone_1_id].reshape(3, 3)
        R2 = data.xmat[drone_1_id].reshape(3, 3)
        R3 = data.xmat[drone_1_id].reshape(3, 3)
        

        # Apply thrust along body z only
        data.xfrc_applied[drone_1_id, 0:3] = F_HOVER * R1[:, 2]
        data.xfrc_applied[drone_2_id, 0:3] = F_HOVER * R2[:, 2]
        data.xfrc_applied[drone_3_id, 0:3] = F_HOVER * R3[:, 2]

        mujoco.mj_step(model, data)
        viewer.sync()

        # Print state every 100 steps
        if int(data.time / model.opt.timestep) % 1500 == 0:
           pass
           # set_ground_cable_length(4, 0.5)
           len -= 0.05
           set_ground_cable_length(4, len)
           if len < 0: break

        #     p  = data.xpos[payload_id]
        #     d1 = data.xpos[drone_1_id]
        #     d2 = data.xpos[drone_2_id]
        #     d3 = data.xpos[drone_3_id]
        #     print(f"t={data.time:.2f}s | "
        #           f"payload z={p[2]:.3f} | "
        #           f"d1 z={d1[2]:.3f} | d2 z={d2[2]:.3f} | d3 z={d3[2]:.3f}")

        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)