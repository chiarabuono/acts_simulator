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

class ControllerNode(Node):
    # Static QoS profile for sensor data (Matches C++ sensor_qos_)
    SENSOR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        depth=10
    )

    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info("Starting Controller Node...")

        # Constants (Defined in hpp)
        self.GRAVITY = 9.81
        self.MAX_ROTOR_VELOCITY = 800.0

        # Parameters
        self.declare_parameter("arm_length", 0.17)
        self.declare_parameter("kt", 5.5e-6)
        self.declare_parameter("kd", 3.299e-7)
        self.declare_parameter("mass", 1.0)
        self.declare_parameter("desired_position", [0.0, 0.0, 0.0])
        self.declare_parameter("desired_velocity", [0.0, 0.0, 0.0])

        # Get parameter values
        arm_length = self.get_parameter("arm_length").value
        kt = self.get_parameter("kt").value
        kd = self.get_parameter("kd").value
        self.drone_mass = self.get_parameter("mass").value
        d_pos = self.get_parameter("desired_position").value
        d_vel = self.get_parameter("desired_velocity").value

        # Initialize Odometry Objects
        self.desired_odometry = Odometry()
        self.desired_odometry.pose.pose.position.x = d_pos[0] if len(d_pos) > 0 else 0.0
        self.desired_odometry.pose.pose.position.y = d_pos[1] if len(d_pos) > 1 else 0.0
        self.desired_odometry.pose.pose.position.z = d_pos[2] if len(d_pos) > 2 else 1.0

        self.desired_odometry.twist.twist.linear.x = d_vel[0] if len(d_vel) > 0 else 0.0
        self.desired_odometry.twist.twist.linear.y = d_vel[1] if len(d_vel) > 1 else 0.0
        self.desired_odometry.twist.twist.linear.z = d_vel[2] if len(d_vel) > 2 else 0.0

        self.current_odometry = Odometry()
        self.odom_received = False

        # Variables Initialization
        self.inv_mixer = self.build_inverse_mixer_matrix(arm_length, kt, kd)

        # Publishers
        actuator_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.actuators_pub = self.create_publisher(Actuators, "command/motor_speed", actuator_qos)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "mocap/odom", self.odom_callback, self.SENSOR_QOS)

        # Timers
        self.timer_period = 0.005  # seconds
        self.pub_actuator_timer = self.create_timer(self.timer_period, self.publish_actuator_speeds)

        self.get_logger().info("Controller Node started.")

    def sgn(self, val):
        return (0 < val) - (val < 0)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
    # If the drone is upright, w should be near 1.0, and x, y, z near 0.0

        self.get_logger().warn(f"ODOM says {q}")
        new_z = msg.pose.pose.position.z
        if hasattr(self, 'last_z') and new_z == self.last_z:
            # If the drone is moving in Gz but this log stays the same, 
            # the plugin fix above is mandatory.
            self.get_logger().debug(f"Odom Frozen at Z: {new_z}")
        
        self.last_z = new_z
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

    def position_controller(self, desired, current):
        # Gains from C++ implementation
        Kp = np.diag([0.0, 0.0, 0.0])
        Kd = np.diag([0.0, 0.0, 0.0])

        pd = np.array([desired.pose.pose.position.x, desired.pose.pose.position.y, desired.pose.pose.position.z])
        p  = np.array([current.pose.pose.position.x, current.pose.pose.position.y, current.pose.pose.position.z])
        vd = np.array([desired.twist.twist.linear.x, desired.twist.twist.linear.y, desired.twist.twist.linear.z])
        v  = np.array([current.twist.twist.linear.x, current.twist.twist.linear.y, current.twist.twist.linear.z])

        g = np.array([0, 0, self.GRAVITY])
        
        # Calculate force vector
        f = Kd @ (vd - v) + Kp @ (pd - p) + (self.drone_mass * g)
        scalar_force = np.linalg.norm(f)

        # Cap thrust
        max_thrust = 12.0
        scalar_force = min(scalar_force, max_thrust)

        # Orientation Matrix construction
        norm_f = np.linalg.norm(f)
        if norm_f < 1e-6:
            z_b = np.array([0, 0, 1]) # Default to pointing up
        else:
            z_b = f / norm_f

        y_w = np.array([0, 1, 0]) # y0 in C++
        x_b = np.cross(y_w, z_b)
        x_b /= np.linalg.norm(x_b)
        y_b = np.cross(z_b, x_b)

        R_mat = np.column_stack((x_b, y_b, z_b))

        # Scipy uses [x, y, z, w] for quats
        desired_quat = R.from_matrix(R_mat).as_quat()

        #desired_quat = np.array([0, 0, 0])
        return desired_quat, scalar_force

    def attitude_controller(self, desired_orientation_quat, current_odom):
        Kp = np.diag([0.0, 0.0, 0.0])
        Kd = np.diag([0.0, 0.0, 0.0])

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
        tau = -Kd @ angular_vel + Kp @ (sign * error_q[:3])

        return tau

    def compute_actuator_speeds(self, desired_odom, current_odom):
        thrust, torques = 0.0, np.zeros(3)
        
        # Position Control
        att_desired, thrust = self.position_controller(desired_odom, current_odom)
        
        # Attitude Control
        torques = self.attitude_controller(att_desired, current_odom)

        w_vec = np.array([thrust, torques[0], torques[1], torques[2]])
        w_squared = self.inv_mixer @ w_vec

        # Apply constraints and sqrt
        speeds = np.sqrt(np.maximum(w_squared, 0.0))
        speeds = np.minimum(speeds, self.MAX_ROTOR_VELOCITY)
        
        return speeds

    def publish_actuator_speeds(self):
        if not self.odom_received:
            return
        
        current_z = self.current_odometry.pose.pose.position.z
        platform_z = 0.5 
        
        # 1. Check the platform condition FIRST
        if current_z < (platform_z + 0.05):
            self.get_logger().info("Hovering low to stabilize...", once=True)
            lift_off_speed = 600.0 
            msg = Actuators()
            msg.velocity = [lift_off_speed] * 4
            self.actuators_pub.publish(msg)
            return # Exit here so the 'crazy' math below never runs

        # 2. Only if we are safely above the platform do we run the PID
        speeds = self.compute_actuator_speeds(self.desired_odometry, self.current_odometry)
        msg = Actuators()
        msg.velocity = speeds.tolist()
        self.actuators_pub.publish(msg)


    def __del__(self):
        # Matches C++ destructor
        print("Shutting down Controller Node...")