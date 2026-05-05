#!/usr/bin/env python3
"""
This files has been modified starting from:
 * File: controller.cpp
 * Project: Quadrotor Control Lab
 * File Created: Tuesday, 25th November 2025 11:57:05 AM
 * Author: nknab
 * Email: kojo.anyinam-boateng@ls2n.fr
 * Version: 1.0.0
 * Brief: Implementation of the quadrotor controller node.
 * -----
 * Last Modified: Tuesday, 25th November 2025 11:57:05 AM
 * Modified By: nknab
 * -----
 * Copyright ©2025 nknab
 */
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import math

class ForceControllerNode(Node):
    SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Starting Controller Node...")

        self.GRAVITY = 9.81
        self.MAX_ROTOR_VELOCITY = 2000.0
        self.MAX_THRUST = 40.0

        # --- Parameters ---
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("kt", 5.5e-6)
        self.declare_parameter("kd", 3.299e-7)
        self.declare_parameter("mass", 1.0)
        self.declare_parameter("anchor_pos", [0.0, 0.0, 0.0])
        self.declare_parameter("f_desired", [0.0, 0.0, -5.0])

        # --- Read parameters ---
        arm_length = self.get_parameter("arm_length").value
        kt = self.get_parameter("kt").value
        kd = self.get_parameter("kd").value
        self.drone_mass  = self.get_parameter("mass").value
        self.anchor_pos = np.array(self.get_parameter("anchor_pos").value, dtype=float)
        self.f_desired   = np.array(self.get_parameter("f_desired").value, dtype=float)
        self.GRAVITY_VEC = np.array([0.0, 0.0, - self.drone_mass * self.GRAVITY])

        self.Kp_cable = 0.0 * np.diag([1.0, 1.0, 1.0]) # 0.3
        self.Ki_cable = 0.0 * np.diag([1.0, 1.0, 1.0]) # 0.3
        self.Kd_cable = 0.0 * np.diag([1.0, 1.0, 1.0]) # 0.0

        # Attitude gains

        self.Kp_att = 2.0 * np.diag([1.0, 1.0, 1.0]) #  3.0
        self.Kd_att = 0.0 * np.diag([1.0, 1.0, 1.0]) #  8.0

        # --- State ---
        self.current_odometry  = Odometry()
        self.odom_received     = False
        self.imu_received      = False
        self.last_time         = None

        self.last_force_error  = np.zeros(3)
        self.force_error_integral = np.zeros(3)

        # Last commanded motor speeds (for thrust reconstruction)
        self.last_wrench       = np.zeros(4)

        self.inv_mixer = self.build_inverse_mixer_matrix(arm_length, kt, kd)

        # Publishers & Subscribers
        self.actuators_pub = self.create_publisher(Actuators, "command/motor_speed", 10)
        self.callback_group = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, "mocap/odom", self.odom_callback, 10,callback_group=self.callback_group)
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.pub_actuator_timer = self.create_timer(0.005, self.publish_actuator_speeds, callback_group=self.callback_group)

        # If anchor is below the drone, z must be negative
        if self.f_desired[2] < 0:
            self.get_logger().warn("Commanded cable force has negative z — is the cable in compression?")

        self.get_logger().info(f"Closed-loop cable controller ready | f_des={self.f_desired} N")

        self.error_pub = self.create_publisher(Float32MultiArray, "error", 10)
        self.derror_pub = self.create_publisher(Float32MultiArray, "d_error", 10)
        # self.info_pub = self.create_publisher(Float32MultiArray, "info", 10)

    def odom_callback(self, msg):
        self.current_odometry = msg
        self.get_logger().info(f"{msg}", once=True)
        self.odom_received    = True

    def imu_callback(self, msg):
        self.imu_msg = msg

        self.linear_accel = np.array([
            self.imu_msg.linear_acceleration.x,
            self.imu_msg.linear_acceleration.y,
            self.imu_msg.linear_acceleration.z
        ])

        self.imu_received = True

    def build_inverse_mixer_matrix(self, l, kt, kd):
        mixer = np.array([
            [kt,      kt,      kt,      kt    ],
            [-kt * l, kt * l,  0.0,     0.0   ],
            [0.0,     0.0,    -kt * l,  kt * l],
            [-kd,    -kd,      kd,      kd    ]
        ])
        return np.linalg.inv(mixer)
    

    def reconstruct_thrust_world(self, current_odom) -> np.ndarray:
        T_total = self.last_wrench[0]  # scalar thrust from previous cycle

        q = current_odom.pose.pose.orientation
        R_body = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        return R_body @ np.array([0.0, 0.0, T_total])


    def estimate_cable_force(self, current_odom) -> np.ndarray:
        q = current_odom.pose.pose.orientation
        R_body = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        # Rotate IMU acceleration to world frame
        accel_world = R_body @ self.linear_accel - np.array([0, 0, self.GRAVITY])
        
        f_prop = self.reconstruct_thrust_world(current_odom)
        
        f_cable_est = - (self.drone_mass * accel_world) + self.GRAVITY_VEC + f_prop
        return f_cable_est


    def cable_force_pid(self, f_desired, f_estimated, dt) -> np.ndarray:
        """
        3-axis PID on cable force error.
        Output is a correction to the thrust command (world frame, N).

        error = f_desired - f_estimated   (3D vector)

        The correction is added to the feedforward thrust:
            f_thrust_cmd = (m·g·ê_z - f_desired)  ← feedforward (open-loop base)
                         + pid_correction           ← closes the loop
        """
        error = f_desired - f_estimated

        # Publish error
        error_msg = Float32MultiArray()
        error_msg.data = error.tolist()
        self.error_pub.publish(error_msg)

        if np.linalg.norm(error) < 1e-2:
            self.get_logger().info(f"Force desired reached", once=True)
        

        if dt > 1e-6:
            d_error = (error - self.last_force_error) / dt
            self.last_force_error = error.copy()
            # Publish d error
            d_error_msg = Float32MultiArray()
            d_error_msg.data = d_error.tolist()
            self.derror_pub.publish(d_error_msg)
        else:
            d_error = np.zeros(3)
            self.last_force_error = error.copy()
            # Publish d error
            d_error_msg = Float32MultiArray()
            d_error_msg.data = d_error.tolist()
            self.derror_pub.publish(d_error_msg)

        

        self.force_error_integral += error * dt
        self.force_error_integral = np.clip(self.force_error_integral, -5.0, 5.0)

        correction = (self.Kp_cable @ error 
                    + self.Kd_cable @ d_error 
                    + self.Ki_cable @ self.force_error_integral)
        
        return correction
    
    
    def force_controller(self, f_thrust: np.ndarray):
        """
        Extract desired quaternion and scalar thrust from a world-frame
        thrust vector. Body z-axis is aligned with f_thrust.
        """
        scalar_force = np.linalg.norm(f_thrust)
        scalar_force = min(scalar_force, self.MAX_THRUST)          # safety cap

        norm_f = np.linalg.norm(f_thrust)
        z_b    = f_thrust / norm_f if norm_f >= 1e-6 else np.array([0.0, 0.0, 1.0])

        y_w = np.array([0.0, 1.0, 0.0])
        x_b_raw = np.cross(y_w, z_b)
        if np.linalg.norm(x_b_raw) < 1e-6:
            x_b_raw = np.cross(np.array([1.0, 0.0, 0.0]), z_b)          # fallback if thrust aligns with y_w

        x_b_n = np.linalg.norm(x_b_raw)
        x_b = x_b_raw / x_b_n if x_b_n >= 1e-6 else np.array([1.0, 0.0, 0.0])
        y_b = np.cross(z_b, x_b)

        R_mat = np.column_stack((x_b, y_b, z_b))
        desired_quat = R.from_matrix(R_mat).as_quat()   # [x, y, z, w]
        return desired_quat, scalar_force
    
    def attitude_controller(self, desired_orientation_quat, current_odom):

        q = current_odom.pose.pose.orientation
        current_q = R.from_quat([q.x, q.y, q.z, q.w])
        des_q = R.from_quat(desired_orientation_quat)

        error_q = (current_q.inv() * des_q).as_quat()  # [x, y, z, w]

        angular_vel = np.array([
            current_odom.twist.twist.angular.x,
            current_odom.twist.twist.angular.y,
            current_odom.twist.twist.angular.z
        ])

        sign = 1.0 if error_q[3] >= 0 else -1.0
        tau = -self.Kd_att @ angular_vel + self.Kp_att @ (sign * error_q[:3])

        return tau


    def _get_yaw(self, odom) -> float:
        """Extract current yaw from odometry quaternion."""
        q = odom.pose.pose.orientation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_euler('ZYX')[0]


    def compute_actuator_speeds(self, f_desired, current_odom, dt):

        f_estimated = self.estimate_cable_force(current_odom)
        pid_correction = self.cable_force_pid(f_desired, f_estimated, dt) # Introduces closed loop correction

        f_prop_desired  = f_desired - self.GRAVITY_VEC 
        f_thrust_cmd = f_prop_desired + pid_correction
        att_desired, thrust = self.force_controller(f_thrust_cmd)
        torques = self.attitude_controller(att_desired, current_odom)

        w_vec = np.array([thrust, torques[0], torques[1], torques[2]])
        self.last_wrench = w_vec
        w_squared = self.inv_mixer @ w_vec

        speeds = np.sqrt(np.maximum(w_squared, 0.0))
        speeds = np.minimum(speeds, self.MAX_ROTOR_VELOCITY)

        # # Publish d error
        # lista = np.array([self.last_wrench[0], 
        #         self.reconstruct_thrust_world(current_odom)[2],
        #         f_estimated[0],
        #         f_estimated[1],
        #         f_estimated[2],
        #         max(speeds)
        #         ])
        
        # d_error_msg = Float32MultiArray()
        # d_error_msg.data = lista.tolist()
        # self.info_pub.publish(d_error_msg)

        return speeds

    def publish_actuator_speeds(self):
        if not self.odom_received:
            self.get_logger().warn("Waiting for ODOM...")
            return
        if not self.imu_received:
            self.get_logger().warn("Waiting for Imu...")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        dt  = (now - self.last_time) if self.last_time is not None else 0.005
        self.last_time = now
        dt  = np.clip(dt, 1e-4, 0.1)                   # guard against bad dt
    
        speeds = self.compute_actuator_speeds(self.f_desired, self.current_odometry, dt)
        msg = Actuators()
        msg.velocity = speeds.tolist()
        self.actuators_pub.publish(msg)


    def __del__(self):
        print("Shutting down Controller Node...")