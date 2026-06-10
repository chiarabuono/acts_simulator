import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

class Drone:
    def __init__(self, model, data, drone_name, qvel_offset=0):
        self.drone_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, drone_name)
        self.data = data
        self.qvel_offset = qvel_offset
        
        # Physical parameters read from XML
        self.mass    = model.body(self.drone_id).mass[0]
        self.I_xx    = model.body(self.drone_id).inertia[0]
        self.I_yy    = model.body(self.drone_id).inertia[1]
        self.I_zz    = model.body(self.drone_id).inertia[2]

        self.g = np.abs(model.opt.gravity[2])
        
        # Motor parameters
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, drone_name)
        self.L_arm = model.geom_size[geom_id][0]
        omega_hover = 838.0
        self.kt = (self.mass * self.g / 4.0) / (omega_hover ** 2)
        self.kd = 0.016 * self.kt
        self.omega_max_sq = (2.0 * omega_hover) ** 2
        self._build_mixer()
        
        self.gains = {'Kp_pos': 0.4, 'Kd_pos': 1.2,
                      'Kp_att': 6.5, 'Kd_att': 5.0}

    def _build_mixer(self):
        kt, kd, L = self.kt, self.kd, self.L_arm
        self.mixer = np.array([
            [ kt,      kt,      kt,      kt    ],
            [ kt*L,   -kt*L,   -kt*L,    kt*L  ],
            [ kt*L,    kt*L,   -kt*L,   -kt*L  ],
            [ kd,     -kd,      kd,     -kd    ],
        ])
        self.inv_mixer = np.linalg.pinv(self.mixer)
    
    def set_desired(self, f_star, b, cable_len):
        # Cable
        self.cable_length = cable_len
        self.b = b

        f_norm   = np.linalg.norm(f_star)
        self.tau_star = f_norm                  # Eq 3.10
        self.u_star   = f_star / f_norm              # Eq 3.11
        self.a_star   = b + cable_len * self.u_star  # Eq 3.12
        self.a_star_dot = np.zeros(3)
        
    def get_state(self, data):
        self.a      = data.xpos[self.drone_id].copy()
        self.a_dot = data.qvel[self.qvel_offset:self.qvel_offset+3]
        self.R_mat  = data.xmat[self.drone_id].reshape(3, 3)
        self.body_z = self.R_mat[:, 2]
        self.quat   = R.from_matrix(self.R_mat).as_quat()

    def rot_to_euler_zyx(self, R):
        pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
        roll  = np.arctan2(R[2, 1], R[2, 2])
        yaw   = np.arctan2(R[1, 0], R[0, 0])
        return np.array([roll, pitch, yaw])

    def desired_R_from_force(self, F_des, yaw_des=0.0):
        F_norm = np.linalg.norm(F_des)
        if F_norm < 1e-6:
            return np.array([0.0, 0.0, yaw_des])
        z_des = F_des / F_norm
        x_c   = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
        y_des = np.cross(z_des, x_c)
        norm_y = np.linalg.norm(y_des)

        if norm_y < 1e-6:
            x_c = np.array([0.0, 1.0, 0.0])
            y_des = np.cross(z_des, x_c)
            norm_y = np.linalg.norm(y_des)
        y_des /= norm_y
        x_des  = np.cross(y_des, z_des)

        return np.column_stack([x_des, y_des, z_des])

    def desired_euler_from_force(self, F_des, yaw_des=0.0):
        R_des = self.desired_R_from_force(F_des, yaw_des=0.0)
        return self.rot_to_euler_zyx(R_des)

    def desired_quat_from_force(self, F_des, yaw_des=0.0):
        R_des = self.desired_R_from_force(F_des, yaw_des=0.0)
        return R.from_matrix(R_des).as_quat()

    def position_controller(self, a_star, a_star_dot):
        dist_cable = np.linalg.norm(self.a - self.b)
        u = (self.a - self.b) / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0]) # Eq 3.9
        
        f = (self.gains['Kp_pos'] * (a_star - self.a)
           + self.gains['Kd_pos'] * (a_star_dot - self.a_dot)
           + self.mass * np.array([0.0, 0.0, self.g])
           + self.tau_star * u)
        F_total   = max(0.0, np.dot(f, self.body_z))
        quat_des  = self.desired_quat_from_force(f)
        return F_total, quat_des

    def attitude_controller(self, quat_des):
        angular_vel = self.R_mat.T @ self.data.qvel[3:6]
        error_q     = (R.from_quat(self.quat).inv() * R.from_quat(quat_des)).as_quat()
        sign        = 1.0 if error_q[3] >= 0 else -1.0
        I_vec       = np.array([self.I_xx, self.I_yy, self.I_zz])
        tau_des     = (-self.gains['Kd_att'] * angular_vel
                      + self.gains['Kp_att'] * sign * error_q[:3])
        return tau_des * I_vec

    def apply_control(self, data, F_total, tau_des):
        wrench   = np.array([F_total, *tau_des])
        omega_sq = np.clip(self.inv_mixer @ wrench, 0.0, self.omega_max_sq)
        w_act    = self.mixer @ omega_sq
        data.xfrc_applied[self.drone_id, 0:3] = w_act[0] * self.body_z
        data.xfrc_applied[self.drone_id, 3:6] = self.R_mat @ w_act[1:4]

    def step(self, data, a_star, a_star_dot):
        self.get_state(data)
        F_total, quat_des = self.position_controller(a_star, a_star_dot)
        tau_des           = self.attitude_controller(quat_des)
        self.apply_control(data, F_total, tau_des)
