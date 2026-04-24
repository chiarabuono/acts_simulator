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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS Message Imports
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators

from rclpy.callback_groups import ReentrantCallbackGroup

class ForceControllerNode(Node):
    # Static QoS profile for sensor data (Matches C++ sensor_qos_)
    SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Starting Controller Node...")

        self.GRAVITY = 9.81
        self.e3 = np.array([0, 0, 1])
        self.MAX_ROTOR_VELOCITY = 2000.0

        self.declare_parameter("Kv", 0.2)
        self.declare_parameter("Kp", 3.2)
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("kt", 5.5e-6)
        self.declare_parameter("kd", 3.299e-7)
        self.declare_parameter("mass", 1.0)
        self.drone_mass = self.get_parameter("mass").value

        self.declare_parameter("desired_force", [0.0, 0.0, self.drone_mass * self.GRAVITY])
        
        # Get parameter values
        self.desired_force = np.array(self.get_parameter("desired_force").value)
        arm_length = self.get_parameter("arm_length").value
        kt = self.get_parameter("kt").value
        kd = self.get_parameter("kd").value
        self.Kv = self.get_parameter("Kv").value
        self.Kp = self.get_parameter("Kp").value
        self.last_F_prop = np.zeros(3)


        times = 1.0

        self.Kp_att = times * 4.0 * np.diag([1.0, 1.0, 1.0]) # Matches C++ 4, 4, 4
        self.Kd_att = times * 0.5 * np.diag([1.0, 1.0, 1.0]) # Matches C++ 0.5, 0.5, 0.5

        self.inv_mixer = self.build_inverse_mixer_matrix(arm_length, kt, kd)

        # Publishers
        self.actuators_pub = self.create_publisher(Actuators, "command/motor_speed", 10)

        # Subscribers
        self.current_odometry = Odometry()
        self.odom_received = False
        self.callback_group = ReentrantCallbackGroup()
        self.odom_sub = self.create_subscription(Odometry, "mocap/odom", self.odom_callback, 10, callback_group=self.callback_group)
        
        # Timers
        self.timer_period = 0.005  # seconds
        self.pub_actuator_timer = self.create_timer(self.timer_period, self.publish_actuator_speeds, callback_group=self.callback_group)

        self.get_logger().info("Force Controller Node started.")


    def odom_callback(self, msg):
        self.current_odometry = msg
        self.odom_received = True

    def build_inverse_mixer_matrix(self, l, kt, kd):
        mixer = np.array([
            [kt, kt, kt, kt],
            [-kt * l, kt * l, 0.0, 0.0],
            [0.0, 0.0, -kt * l, kt * l],
            [-kd, -kd, kd, kd]
        ])
        return np.linalg.inv(mixer)
    
    def estimate_tau(self, u):
        return float(np.dot(u, self.last_F_prop - self.drone_mass * self.GRAVITY * self.e3))
    
    def compute_force_propulsion(self, desired_force, current_odom):
        Kv = self.Kv
        Kp = self.Kp

        Kv = 1.0
        Kp = 0.6
        Ktau = 0.1

        tau_desired = np.linalg.norm(desired_force)
        u_desired = desired_force/tau_desired
        
    
        drone_pose = np.array([
            current_odom.pose.pose.position.x,
            current_odom.pose.pose.position.y,
            current_odom.pose.pose.position.z
        ])
        b = np.array([0, 0, 0])   # TODO: hardcoded as we suppose  cable attachment point at [0, 0, 0]
        u = drone_pose - b
        norm_u = np.linalg.norm(u)

        if norm_u > 1e-6: u = u / norm_u
        else: u = np.zeros(3)

        error_u = (np.eye(3) - np.outer(u_desired, u_desired)) @ (drone_pose - b)

        tau = self.estimate_tau(u)     # TODO: take it from somewhere
        error_tau = tau_desired - tau
        tau_input = tau_desired + Ktau * error_tau

        linear_vel_des = - Kp * error_u
        linear_vel = np.array([
            current_odom.twist.twist.linear.x,
            current_odom.twist.twist.linear.y,
            current_odom.twist.twist.linear.z
        ])
        error_v = linear_vel_des - linear_vel
        # print(f"lin vel: {linear_vel} | drone pose: {drone_pose}")
        # print(f"{tau_input} * {u} + {self.drone_mass * self.GRAVITY * self.e3} + {Kv * error_v} = {tau_input * u + self.drone_mass * self.GRAVITY * self.e3 + Kv * error_v}")
        print(f"{error_tau=} | tau={tau:.4f} | tau_desired={tau_desired:.4f}")
        print(f"{error_u=} | u={u} | u_desired={u_desired}")
        
        return tau_input * u + self.drone_mass * self.GRAVITY * self.e3 + Kv * error_v

    def force_controller(self, desired_force, current_odom):
        f = self.compute_force_propulsion(desired_force, current_odom)
        self.last_F_prop = f.copy()
        # f = np.array([0, 100, 100])
        # print(f"Force: {f}")

        scalar_force = np.linalg.norm(f)
        max_thrust = 20.0
        #scalar_force = min(scalar_force, max_thrust)

        norm_f = np.linalg.norm(f)
        if norm_f < 1e-6:
            z_b = np.array([0, 0, 1])
        else:
            z_b = f / norm_f

        y_w = np.array([0, 1, 0])
        x_b = np.cross(y_w, z_b)
        x_b /= np.linalg.norm(x_b)
        y_b = np.cross(z_b, x_b)

        R_mat = np.column_stack((x_b, y_b, z_b))
        desired_quat = R.from_matrix(R_mat).as_quat()

        return desired_quat, scalar_force
    
    def attitude_controller(self, desired_orientation_quat, current_odom):

        q = current_odom.pose.pose.orientation
        current_q = R.from_quat([q.x, q.y, q.z, q.w])
        des_q = R.from_quat(desired_orientation_quat)

        # error = current.inverse() * desired
        error_q_obj = current_q.inv() * des_q
        error_q = error_q_obj.as_quat() # [x, y, z, w]

        angular_vel = np.array([
            current_odom.twist.twist.angular.x,
            current_odom.twist.twist.angular.y,
            current_odom.twist.twist.angular.z
        ])

        # sign * error.vec()
        sign = 1.0 if error_q[3] >= 0 else -1.0
        tau = -self.Kd_att @ angular_vel + self.Kp_att @ (sign * error_q[:3])

        return tau
    

    def compute_actuator_speeds(self, current_odom):
        
        att_desired, thrust = self.force_controller(self.desired_force, self.current_odometry)
        torques = self.attitude_controller(att_desired, current_odom)

        w_vec = np.array([thrust, torques[0], torques[1], torques[2]])
        w_squared = self.inv_mixer @ w_vec

        speeds = np.sqrt(np.maximum(w_squared, 0.0))
        speeds = np.minimum(speeds, self.MAX_ROTOR_VELOCITY)
        
        return speeds

    def publish_actuator_speeds(self):
        if not self.odom_received:
            self.get_logger().warn("Waiting for ODOM...")
            return

        speeds = self.compute_actuator_speeds(self.current_odometry)
        msg = Actuators()
        msg.velocity = speeds.tolist()
        self.actuators_pub.publish(msg)


    def __del__(self):
        print("Shutting down Controller Node...")