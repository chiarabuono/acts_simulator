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
    (angle_x, angle_y) for tip joints
        │
        ▼
    JointTrajectory → /ugv_tip_position_controller/joint_trajectory
        │
        ▼
    Gazebo: cable swings, UGV body moves to target

ROS2 interfaces:
    Publishes : /ugv_tip_position_controller/joint_trajectory  [JointTrajectory]
    Subscribes: /joint_states                                   [JointState]
    Subscribes: /ugv/target                                     [geometry_msgs/PointStamped]
                  → send targets at runtime without restarting

Parameters:
    panel_x, panel_y, panel_z  : world position of the cable attachment point
    cable_length               : total length of the cable
    motion_duration            : seconds to complete one motion
    ugv1_target_x/y            : initial target for ugv1
    ugv2_target_x/y            : initial target for ugv2
    use_sim_time               : use Gazebo sim time

Usage (from launch or CLI):
    ros2 run acts_simulator ugv_tip_controller
    ros2 run acts_simulator ugv_tip_controller --ros-args \
        -p ugv1_target_x:=0.5 -p ugv1_target_y:=0.3

Send a new target at runtime:
    ros2 topic pub /ugv/target geometry_msgs/msg/PointStamped \
        "{header: {frame_id: 'ugv1'}, point: {x: 0.4, y: 0.2, z: 0.0}}"
        # frame_id = 'ugv1' | 'ugv2' | ... selects which UGV to move
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Duration as RosDuration


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def xyz_to_tip_angles(
    target_x: float, target_y: float, target_z: float,
    panel_x: float,  panel_y: float,  panel_z: float,
    cable_length: float,
) -> tuple[float, float]:
    """
    Compute the two tip joint angles (joint_x, joint_y) needed to place
    the cable end at (target_x, target_y, target_z) given the panel position
    and cable length.

    The cable hangs from (panel_x, panel_y, panel_z) with total length
    cable_length. The tip joints steer the last segment:

        joint_x_cable_to_ugv  → controls sway in the Y direction (rotation around X)
        joint_y_cable_to_ugv  → controls sway in the X direction (rotation around Y)

    Returns (angle_x, angle_y) in radians, clamped to [-π/2, π/2].
    """
    dx = target_x - panel_x
    dy = target_y - panel_y
    # dz = target_z - panel_z  # not used directly, cable_length sets the reach

    # Guard: target must be reachable (within cable length in XY plane)
    xy_dist = math.sqrt(dx**2 + dy**2)
    if xy_dist > cable_length:
        scale = (cable_length * 0.99) / xy_dist  # clamp to 99% of max reach
        dx *= scale
        dy *= scale
        xy_dist = math.sqrt(dx**2 + dy**2)

    # sin(angle) = opposite / hypotenuse = offset / cable_length
    angle_x = math.asin(max(-1.0, min(1.0,  dy / cable_length)))  # sway Y
    angle_y = math.asin(max(-1.0, min(1.0,  dx / cable_length)))  # sway X

    # Clamp to joint limits (±90°)
    limit = math.pi / 2
    angle_x = max(-limit, min(limit, angle_x))
    angle_y = max(-limit, min(limit, angle_y))

    return angle_x, angle_y


# ─────────────────────────────────────────────────────────────────────────────
# UGV descriptor
# ─────────────────────────────────────────────────────────────────────────────

class UGV:
    """Holds the state and target for a single UGV cable."""

    def __init__(self, ugv_id: int, panel_x: float, panel_y: float, panel_z: float,
                 cable_length: float, target_x: float, target_y: float):
        self.id = ugv_id
        self.prefix = f"ugv{ugv_id}_"
        self.joint_x = f"ugv{ugv_id}_joint_x_cable_to_ugv"
        self.joint_y = f"ugv{ugv_id}_joint_y_cable_to_ugv"

        self.panel_x = panel_x
        self.panel_y = panel_y
        self.panel_z = panel_z
        self.cable_length = cable_length

        self.target_x = target_x
        self.target_y = target_y
        self.target_z = 0.0  # ground level

        # Current joint positions from /joint_states
        self.current_angle_x = 0.0
        self.current_angle_y = 0.0

    def compute_target_angles(self) -> tuple[float, float]:
        return xyz_to_tip_angles(
            self.target_x, self.target_y, self.target_z,
            self.panel_x, self.panel_y, self.panel_z,
            self.cable_length,
        )

    def set_target(self, x: float, y: float, z: float = 0.0):
        self.target_x = x
        self.target_y = y
        self.target_z = z


# ─────────────────────────────────────────────────────────────────────────────
# Controller node
# ─────────────────────────────────────────────────────────────────────────────

class UGVTipController(Node):

    CONTROLLER_TOPIC = '/ugv_tip_position_controller/joint_trajectory'
    TARGET_TOPIC     = '/ugv/target'
    JOINT_STATE_TOPIC = '/joint_states'

    def __init__(self):
        super().__init__('ugv_tip_controller')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('cable_length',   1.5)
        self.declare_parameter('motion_duration', 3.0)

        self.declare_parameter('ugv1_panel_x',  0.0)
        self.declare_parameter('ugv1_panel_y',  0.0)
        self.declare_parameter('ugv1_panel_z',  1.5)
        self.declare_parameter('ugv1_target_x', 0.0)
        self.declare_parameter('ugv1_target_y', 0.0)

        self.declare_parameter('ugv2_panel_x',  0.5)
        self.declare_parameter('ugv2_panel_y',  0.5)
        self.declare_parameter('ugv2_panel_z',  1.5)
        self.declare_parameter('ugv2_target_x', 0.5)
        self.declare_parameter('ugv2_target_y', 0.5)

        cable_length  = self.get_parameter('cable_length').value
        self.duration = self.get_parameter('motion_duration').value


        self.ugvs: dict[str, UGV] = {
            'ugv1': UGV(
                ugv_id=1,
                panel_x=self.get_parameter('ugv1_panel_x').value,
                panel_y=self.get_parameter('ugv1_panel_y').value,
                panel_z=self.get_parameter('ugv1_panel_z').value,
                cable_length=cable_length,
                target_x=self.get_parameter('ugv1_target_x').value,
                target_y=self.get_parameter('ugv1_target_y').value,
            ),
            'ugv2': UGV(
                ugv_id=2,
                panel_x=self.get_parameter('ugv2_panel_x').value,
                panel_y=self.get_parameter('ugv2_panel_y').value,
                panel_z=self.get_parameter('ugv2_panel_z').value,
                cable_length=cable_length,
                target_x=self.get_parameter('ugv2_target_x').value,
                target_y=self.get_parameter('ugv2_target_y').value,
            ),
        }

        # ── Publisher ─────────────────────────────────────────────────
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            self.CONTROLLER_TOPIC,
            10
        )

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            JointState,
            self.JOINT_STATE_TOPIC,
            self._on_joint_states,
            10
        )

        self.create_subscription(
            PointStamped,
            self.TARGET_TOPIC,
            self._on_new_target,
            10
        )

        # ── State ─────────────────────────────────────────────────────
        self._joint_states_received = False

        # Wait for joint states then send initial targets
        self.get_logger().info('Waiting for /joint_states...')
        # self._init_timer = self.create_timer(0.5, self._try_initial_move)
        self.get_logger().info('Starting cyclic control loop at 10Hz...')
        self.control_timer = self.create_timer(0.1, self._control_loop)

    def _control_loop(self):
        """Cyclic loop running at 10Hz to ensure state is consistently maintained."""
        if not self._joint_states_received:
            return
            
        # Continuously publish the trajectory to ensure Gazebo catches it
        # This resolves the DDS discovery race condition
        self._send_trajectory(ugv_ids=list(self.ugvs.keys()))

    # ── Joint state callback ──────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        """Track current joint positions for all UGV tip joints."""
        state = dict(zip(msg.name, msg.position))
        for ugv in self.ugvs.values():
            if ugv.joint_x in state:
                ugv.current_angle_x = state[ugv.joint_x]
            if ugv.joint_y in state:
                ugv.current_angle_y = state[ugv.joint_y]
        self._joint_states_received = True

    # ── Runtime target callback ───────────────────────────────────────

    def _on_new_target(self, msg: PointStamped):
        """
        Receive a new target position for a specific UGV.
        The frame_id field selects which UGV to move (e.g. 'ugv1', 'ugv2').
        """
        ugv_id = msg.header.frame_id
        if ugv_id not in self.ugvs:
            self.get_logger().warn(
                f'Unknown UGV id: "{ugv_id}". '
                f'Available: {list(self.ugvs.keys())}'
            )
            return

        ugv = self.ugvs[ugv_id]
        ugv.set_target(msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().info(
            f'[{ugv_id}] New target: x={msg.point.x:.3f}, '
            f'y={msg.point.y:.3f}, z={msg.point.z:.3f}'
        )
        self._send_trajectory(ugv_ids=[ugv_id])

    # ── Initial move ──────────────────────────────────────────────────

    def _try_initial_move(self):
        """Send initial targets once joint states are available."""
        if not self._joint_states_received:
            return
        self._init_timer.cancel()
        self.get_logger().info('Joint states received. Sending initial targets.')
        self._send_trajectory(ugv_ids=list(self.ugvs.keys()))

    # ── Core: build and publish trajectory ───────────────────────────

    def _send_trajectory(self, ugv_ids: list[str]):
        """
        Build a JointTrajectory for the requested UGVs and publish it.
        All other UGVs hold their current position.
        """
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg() # Always timestamp your trajectories!
        
        # Explicitly initialize arrays cleanly for this point
        joint_names = []
        positions = []
        velocities = []

        for ugv_id, ugv in self.ugvs.items():
            joint_names.append(ugv.joint_x)
            joint_names.append(ugv.joint_y)

            if ugv_id in ugv_ids:
                # Move to new target
                angle_x, angle_y = ugv.compute_target_angles()
                self.get_logger().info(
                    f'[{ugv_id}] → angle_x={math.degrees(angle_x):.1f}°, '
                    f'angle_y={math.degrees(angle_y):.1f}°'
                )
            else:
                # Hold current position
                angle_x = ugv.current_angle_x
                angle_y = ugv.current_angle_y

            positions.append(angle_x)
            positions.append(angle_y)
            velocities.append(0.0)
            velocities.append(0.0)

        # Pack data into message
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.time_from_start = RosDuration(
            sec=0,
            nanosec=150000000  # 0.15 seconds
        )
        
        msg.points = [point]
        self.trajectory_pub.publish(msg)

    # ── Public API ────────────────────────────────────────────────────

    def move_ugv(self, ugv_id: str, x: float, y: float, z: float = 0.0):
        """
        Move a single UGV to (x, y, z) from external code.

        Example:
            node.move_ugv('ugv1', x=0.4, y=0.2)
        """
        if ugv_id not in self.ugvs:
            self.get_logger().error(f'Unknown UGV: {ugv_id}')
            return
        self.ugvs[ugv_id].set_target(x, y, z)
        self._send_trajectory(ugv_ids=[ugv_id])

    def move_all(self, targets: dict[str, tuple[float, float, float]]):
        """
        Move all UGVs simultaneously.

        Example:
            node.move_all({
                'ugv1': (0.0, 0.0, 0.0),
                'ugv2': (0.5, 0.5, 0.0),
            })
        """
        for ugv_id, (x, y, z) in targets.items():
            if ugv_id in self.ugvs:
                self.ugvs[ugv_id].set_target(x, y, z)
        self._send_trajectory(ugv_ids=list(targets.keys()))


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = UGVTipController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()