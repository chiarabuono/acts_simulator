#!/usr/bin/env python3
"""
Node: gz_pose_to_odom
Subscribes to a geometry_msgs/PoseArray published by Gazebo's PosePublisher
(which bundles all link poses), extracts the pose for a specific link by name,
differentiates it to compute linear and angular velocity, and republishes
as nav_msgs/Odometry.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry


def quat_to_rot(q):
    """Convert geometry_msgs quaternion to 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def quat_multiply(q1, q2_inv):
    """Compute q1 * conjugate(q2) to get relative rotation quaternion."""
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    # conjugate of q2
    x2, y2, z2, w2 = -q2_inv.x, -q2_inv.y, -q2_inv.z, q2_inv.w
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


class GzPoseToOdom(Node):

    def __init__(self):
        super().__init__('gz_pose_to_odom')

        self.declare_parameter('link_name', 'drone1_base_link')
        self.declare_parameter('odom_frame', 'world')
        self.declare_parameter('base_frame', 'drone1_base_link')

        self.link_name = self.get_parameter('link_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self._prev_time = None
        self._prev_pos = None
        self._prev_quat = None

        self.sub = self.create_subscription(
            PoseArray,
            'pose_array',
            self._cb,
            10
        )
        self.pub = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info(
            f"gz_pose_to_odom ready — tracking link: '{self.link_name}'"
        )

    def _cb(self, msg: PoseArray):
        # PoseArray from Gazebo PosePublisher does not carry names —
        # the topic itself is scoped to the link, so there is only ONE pose.
        if len(msg.poses) == 0:
            return

        pose = msg.poses[0]
        now = self.get_clock().now()

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose

        # Differentiate position for linear velocity
        if self._prev_time is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
            if dt > 0.0:
                p = pose.position
                pp = self._prev_pos

                # Linear velocity in world frame
                vx = (p.x - pp[0]) / dt
                vy = (p.y - pp[1]) / dt
                vz = (p.z - pp[2]) / dt


                # Angular velocity from quaternion difference
                dq = quat_multiply(pose.orientation, self._prev_quat)
                # dq ≈ [wx, wy, wz, 1] * dt/2 for small rotations
                norm = np.linalg.norm(dq[:3])
                if norm > 1e-10:
                    angle = 2.0 * np.arctan2(norm, abs(dq[3]))
                    axis = dq[:3] / norm
                    w_world = axis * (angle / dt)
                    w_body = w_world #R @ w_world
                else:
                    w_body = np.zeros(3)

                odom.twist.twist.linear.x = float(vx)
                odom.twist.twist.linear.y = float(vy)
                odom.twist.twist.linear.z = float(vz)
                odom.twist.twist.angular.x = float(w_body[0])
                odom.twist.twist.angular.y = float(w_body[1])
                odom.twist.twist.angular.z = float(w_body[2])

        self._prev_time = now
        self._prev_pos = (pose.position.x, pose.position.y, pose.position.z)
        self._prev_quat = pose.orientation

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GzPoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()