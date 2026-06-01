#!/usr/bin/env python3
"""
ugv_tip_controller.py
─────────────────────
Controls the tip joints of UGV cables to move the UGV base_link
to a desired (x, y) ground position using smooth joint trajectory.

Architecture:
    Target (x, y) ground position
        │
        ▼  inverse geometry
    (angle_x, angle_y) for top-mounted pendulum joints
        │
        ▼
    JointTrajectory → /ugv_tip_position_controller/joint_trajectory
        │
        ▼
    Gazebo: cable swings, UGV body moves to target cleanly
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RosDuration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

class UGVState:
    """Helper class to manage tracking state and inverse kinematics for each UGV."""
    def __init__(self, name: str, panel_x: float, panel_y: float, panel_z: float, L: float):
        self.name = name
        self.panel_x = panel_x
        self.panel_y = panel_y
        self.panel_z = panel_z
        self.L = L

        # Track the active targets (persist across loop iterations)
        self.target_x = panel_x
        self.target_y = panel_y
        self.target_z = 0.0

        # Define explicit matching joint names corresponding to the URDF configuration
        self.joint_x = f"{name}_joint_x_cable_to_ugv"
        self.joint_y = f"{name}_joint_y_cable_to_ugv"

    def set_target(self, x: float, y: float, z: float = 0.0):
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def compute_target_angles(self) -> tuple[float, float]:
        """
        Calculates required angles based on overhead pendulum geometric projection.
        - Rotation around X (Roll) displaces along the Y-axis.
        - Rotation around Y (Pitch) displaces along the X-axis.
        """
        dx = self.target_x - self.panel_x
        dy = self.target_y - self.panel_y

        # Prevent domain errors outside math.asin boundaries [-1, 1]
        ratio_x = max(-0.99, min(0.99, -dx / self.L))
        ratio_y = max(-0.99, min(0.99, dy / self.L))

        angle_x = math.asin(ratio_y)
        angle_y = math.asin(ratio_x)

        return angle_x, angle_y


class UGVTipPositionController(Node):
    def __init__(self):
        super().__init__('ugv_tip_position_controller')

        # ── Declare and read system parameters ──────────────────────────
        self.declare_parameter('cable_length', 1.5)
        self.declare_parameter('motion_duration', 3.0)
        
        L = self.get_parameter('cable_length').value

        # Instantiate tracking handles for our two systems
        self.ugvs: dict[str, UGVState] = {}
        for idx in [1, 2]:
            prefix = f"ugv{idx}"
            self.declare_parameter(f'{prefix}_panel_x', 0.0 if idx == 1 else 0.5)
            self.declare_parameter(f'{prefix}_panel_y', 0.0 if idx == 1 else 0.5)
            self.declare_parameter(f'{prefix}_panel_z', 1.5)
            self.declare_parameter(f'{prefix}_target_x', 0.0 if idx == 1 else 0.5)
            self.declare_parameter(f'{prefix}_target_y', 0.0 if idx == 1 else 0.5)

            px = self.get_parameter(f'{prefix}_panel_x').value
            py = self.get_parameter(f'{prefix}_panel_y').value
            pz = self.get_parameter(f'{prefix}_panel_z').value
            tx = self.get_parameter(f'{prefix}_target_x').value
            ty = self.get_parameter(f'{prefix}_target_y').value

            state = UGVState(prefix, px, py, pz, L)
            state.set_target(tx, ty)
            self.ugvs[prefix] = state

        # ── Subscribes & Publishers ────────────────────────────────────
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/ugv_tip_position_controller/joint_trajectory',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_states,
            10
        )

        self.target_sub = self.create_subscription(
            PointStamped,
            '/ugv/target',
            self._on_new_target,
            10
        )

        # Cyclic 10Hz background heartbeat loop keeping controller active
        self.control_timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info('UGV Tip Joint Trajectory Controller initialized successfully!')

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        """Passively tracks ongoing state configurations if needed."""
        pass

    def _on_new_target(self, msg: PointStamped):
        """Processes dynamic targets sent at runtime via /ugv/target."""
        frame_id = msg.header.frame_id  # Expected format: "ugv1" or "ugv2"
        if frame_id not in self.ugvs:
            self.get_logger().warn(f"Target received for unknown UGV frame: '{frame_id}'")
            return

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(f"Updating {frame_id} Target to: ({x:.2f}, {y:.2f})")
        self.ugvs[frame_id].set_target(x, y, z)
        
        # Fire off an immediate, prioritized trajectory command update
        self._send_trajectory(ugv_ids=[frame_id])

    def _control_loop(self):
        """Main loop executing at 10Hz to maintain persistent position hold."""
        self._send_trajectory(ugv_ids=list(self.ugvs.keys()))

    def _send_trajectory(self, ugv_ids: list[str]):
        """Builds and dispatches the dual-point trajectory hold sequence."""
        if not ugv_ids:
            return

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        joint_names = []
        positions = []
        velocities = []

        # Parse joints strictly belonging to requested IDs
        for ugv_id in ugv_ids:
            if ugv_id not in self.ugvs:
                continue
                
            ugv = self.ugvs[ugv_id]
            joint_names.append(ugv.joint_x)
            joint_names.append(ugv.joint_y)

            angle_x, angle_y = ugv.compute_target_angles()
            positions.append(angle_x)
            positions.append(angle_y)
            
            # Request zero velocity to help stabilize target overshoot dampening
            velocities.append(0.0)
            velocities.append(0.0)

        if not joint_names:
            return

        msg.joint_names = joint_names

        # Point 1: Command target position transition window (reached in 0.1s)
        p1 = JointTrajectoryPoint()
        p1.positions = positions
        p1.velocities = velocities
        p1.time_from_start = RosDuration(seconds=0, nanoseconds=100000000).to_msg()

        # Point 2: Active HOLD point (locks configuration stretching out to 0.5s)
        p2 = JointTrajectoryPoint()
        p2.positions = positions
        p2.velocities = velocities
        p2.time_from_start = RosDuration(seconds=0, nanoseconds=500000000).to_msg()
        
        msg.points = [p1, p2]
        self.trajectory_pub.publish(msg)

    # ── Public API ────────────────────────────────────────────────────

    def move_ugv(self, ugv_id: str, x: float, y: float, z: float = 0.0):
        if ugv_id not in self.ugvs:
            self.get_logger().error(f'Unknown UGV: {ugv_id}')
            return
        self.ugvs[ugv_id].set_target(x, y, z)
        self._send_trajectory(ugv_ids=[ugv_id])

    def move_all(self, targets: dict[str, tuple[float, float, float]]):
        for ugv_id, (x, y, z) in targets.items():
            if ugv_id in self.ugvs:
                self.ugvs[ugv_id].set_target(x, y, z)
        self._send_trajectory(ugv_ids=list(targets.keys()))


def main(args=None):
    rclpy.init(args=args)
    node = UGVTipPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()