import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

class BaseDrone:
    def __init__(self, model, drone_name="drone", qvel_offset=0):
        self.GRAVITY = 9.81
        self.MAX_ROTOR_VELOCITY = 2000.0
        self.arm_length = 0.17
        self.kt = 5.5e-6
        self.kd = 3.299e-7
        self.drone_mass = model.body(drone_name).mass[0]
        self.qvel_offset = qvel_offset

        self.gains = {
            'Kp_pos': np.diag([5.0, 5.0, 10.0]), 
            'Kd_pos': np.diag([4.2, 4.2, 5.0]),
            'Kp_att': np.diag([5.6, 5.6, 5.6]), 
            'Kd_att': np.diag([1.0, 1.0, 1.0])
        } 
        self.inv_mixer = self.build_inverse_mixer_matrix(self.arm_length, self.kt, self.kd)

    def build_inverse_mixer_matrix(self, l, kt, kd):
        mixer = np.array([
            [  kt,       kt,       kt,       kt     ],
            [ -kt * l,   kt * l,   kt * l,  -kt * l ],
            [ -kt * l,  -kt * l,   kt * l,   kt * l ],
            [ -kd,       kd,      -kd,       kd     ]
        ])
        return np.linalg.inv(mixer)

    def build_forward_mixer(self, l, kt, kd):
        return np.array([
            [  kt,       kt,       kt,       kt     ],
            [ -kt * l,   kt * l,   kt * l,  -kt * l ],
            [ -kt * l,  -kt * l,   kt * l,   kt * l ],
            [ -kd,       kd,      -kd,       kd     ]
        ])

    def calculate_orientation_frame(self, f):
        norm_f = np.linalg.norm(f)
        z_b = f / norm_f if norm_f > 1e-6 else np.array([0.0, 0.0, 1.0])

        y_w = np.array([0.0, 1.0, 0.0])
        x_b = np.cross(y_w, z_b)
        if np.linalg.norm(x_b) < 1e-6:
            x_b = np.array([1.0, 0.0, 0.0])
        else:
            x_b /= np.linalg.norm(x_b)
            
        y_b = np.cross(z_b, x_b)
        R_mat = np.column_stack((x_b, y_b, z_b))
        desired_quat = R.from_matrix(R_mat).as_quat()
        return desired_quat, np.linalg.norm(f), R_mat

    def attitude_controller(self, desired_orientation_quat, current_quat, angular_vel):
        current_q = R.from_quat(current_quat)
        des_q = R.from_quat(desired_orientation_quat)
        error_q = (current_q.inv() * des_q).as_quat() 

        sign = 1.0 if error_q[3] >= 0 else -1.0
        return -self.gains["Kd_att"] @ angular_vel + self.gains["Kp_att"] @ (sign * error_q[:3])

    def compute_motor_wrenches(self, pd, p, vd, v, current_quat, angular_vel):
        att_desired, thrust, R_des = self.position_controller(pd, p, vd, v)
        torques = self.attitude_controller(att_desired, current_quat, angular_vel)

        w_vec = np.array([thrust, torques[0], torques[1], torques[2]])
        speeds = np.sqrt(np.maximum(self.inv_mixer @ w_vec, 0.0))
        speeds = np.minimum(speeds, self.MAX_ROTOR_VELOCITY)
        
        return self.build_forward_mixer(self.arm_length, self.kt, self.kd) @ (speeds**2), R_des
    
class FreeFlightDrone(BaseDrone):
    def position_controller(self, pd, p, vd, v):
        g_vec = np.array([0, 0, self.GRAVITY])
        f = self.gains["Kd_pos"] @ (vd - v) + self.gains["Kp_pos"] @ (pd - p) + (self.drone_mass * g_vec)
        
        # Pass the force to the shared frame generator
        return self.calculate_orientation_frame(f)

class CableTetheredDrone(BaseDrone):
    def __init__(self, model, drone_name="drone", anchor_b=None, cable_len=None, f_star=None):
        super().__init__(model, drone_name)

        # Cable
        self.cable_length = cable_len
        self.anchor_b = anchor_b
        self.tau_star = np.linalg.norm(f_star)                       # Eq 3.2
    
    def position_controller(self, pd, p, vd, v):
        g_vec = np.array([0, 0, self.GRAVITY])
        
        # Base translation force
        f_base = self.gains["Kd_pos"] @ (vd - v) + self.gains["Kp_pos"] @ (pd - p) + (self.drone_mass * g_vec) # Eq 3.5 partial
        
        # Explicit cable addition
        dist_cable = np.linalg.norm(p - self.anchor_b)
        u = (p - self.anchor_b) / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0]) # Eq 3.9
        f_total = f_base + (self.tau_star * u)                      # Eq 3.5
        
        return self.calculate_orientation_frame(f_total)

class PayloadControlDrone(BaseDrone):
    def __init__(self, model, drone_name="drone", qvel_offset=0, payload_mass=1.0, gains_platform=None):
        super().__init__(model, drone_name, qvel_offset)
        self.m_payload = payload_mass
        self.gains_platform = gains_platform if gains_platform is not None else {'Kp_pl': 0.2, 'Kd_pl': 0.1}
        self.e3 = np.array([0.0, 0.0, 1.0])
        
        self.p_star_ddot = np.zeros(3)
        self.Fa_2 = np.zeros(3)

    def set_payload_states(self, p_star_ddot, Fa_2, p_star, p_star_dot):
        self.p_star_ddot = p_star_ddot
        self.Fa_2 = Fa_2
        self.p_payload_des = p_star
        self.v_payload_des = p_star_dot

    def update_payload_pose(self, p_payload, v_payload):
        self.p_payload = p_payload
        self.v_payload = v_payload
    
    def compute_Fp_star(self):
        return self.m_payload * (self.p_star_ddot + self.GRAVITY * self.e3) + \
                   self.gains_platform['Kp_pl'] * (self.p_payload_des - self.p_payload) + \
                   self.gains_platform['Kd_pl'] * (self.v_payload_des - self.v_payload)

    def position_controller(self, pd, p, vd, v):
        # COMPLIANT TO EQ. 3.19: Fp* = mp*(p_ddot* + g*e3) + Kp*(p*-p) + Kd*(v*-v)
        F_p_star = self.compute_Fp_star()
        F_a1_star = F_p_star - self.Fa_2        # Eq 3.12

        g_vec = np.array([0, 0, self.GRAVITY])

        tau_star = np.linalg.norm(F_a1_star)                       # Eq 3.2
        f_base = self.gains["Kd_pos"] @ (vd - v) + self.gains["Kp_pos"] @ (pd - p) + (self.drone_mass * g_vec)

        dist_cable = np.linalg.norm(p - self.p_payload)
        u = (p - self.p_payload) / dist_cable if dist_cable > 0.001 else np.array([0.0, 0.0, 1.0]) # Eq 3.9

        f_total = f_base + (tau_star * u)                      # Eq 3.5

        return self.calculate_orientation_frame(f_total)