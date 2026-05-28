"""
This files has been modified starting from:
File: uav_simulation.launch.py
Project: Quadrotor Control Lab
File Created: Tuesday, 25th November 2025 3:05:38 PM
Author: nknab
Email: kojo.anyinam-boateng@ls2n.fr
Version: 1.0.0
Brief: Launch file to simulate a UAV with a controller in Gazebo.
-----
Last Modified: Tuesday, 25th November 2025 10:50:26 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os import popen

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from acts_simulator.utils_controller import parse_float_list, get_drone_nodes_position_control, get_drone_start


PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 8.0


def clean_function(_: LaunchContext) -> None:
    """
    Function to clean up the system by killing the PX4 process.

    Parameters
    ----------
    _ : LaunchContext
        The launch context.

    """
    popen("pkill -x gz")
    popen("pkill -x ruby")


def launch_setup(
    context: LaunchContext,
) -> list[
    SetParameter | ExecuteProcess | IncludeLaunchDescription | RegisterEventHandler
]:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context object to get the launch configuration

    Returns
    -------
    list[SetParameter | IncludeLaunchDescription | RegisterEventHandler]
        The list of actions to be executed in the launch file

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    fixed = LaunchConfiguration("fixed").perform(context)

    sim = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " launch ",
                PathJoinSubstitution([
                    pkg_share,
                    "launch",
                    "acts_sim.launch.py",
                ]),
                " headless:=false",
                " use_rviz:=false",
                f" fixed:={fixed}",
            ]
        ],
        name="sim",
        shell=True,
        output="screen",
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["UAV Simulation - Cleaning up after shutdown!"]),
            ]
        )
    )

    actions = [sim, shutdown_handler]
    mass = 2.031400 # Received from the actual mass of the entire system 1 drone + 1 cable + 1 payload with 0.001 mass

    # get_drone_start(sim, "drone1_", actions)
    # get_drone_start(sim, "drone2_", actions)
    # get_drone_start(sim, "drone3_", actions)

#     pulley_x = "1.0"
#     pulley_y = "1.0"
#     pulley_z = "0.0"

#     init_x = "0.0"
#     init_y = "0.0"
#     init_z = "0.0"

#     position_controller_node = Node(
#         package=PACKAGE_NAME,
#         executable="cable_controller_position",
#         name="controller_pulley_a",
#         output="screen",
#         parameters=[{
#             "current_x": float(init_x),
#             "current_y": float(init_y),
#             "current_z": float(init_z),
#             "total_cable_len": 1.5,
#             "cable_extreme": "link_first",
#             "vel": 0.01,
#             "unwinded_cable_len": 0.0,
#             "final_cable_len": 1.0,
#             "pulley_x": float(pulley_x),
#             "pulley_y": float(pulley_y),
#             "pulley_z": float(pulley_z),  
#             "num_segments": 5,         
#         }],

#         remappings=[
#             ("motor_commands", "/pulley_position_controller/commands")
#         ]
#     )

#     delayed_controller = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=sim,
#             on_start=TimerAction(period=WAIT_TIME, actions=[position_controller_node]),
#         )
#     )

#     joint_state_broadcaster_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
#     )

#     position_controller_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["pulley_position_controller", "--controller-manager", "/controller_manager"],
#     )


#     pulley_xy_effort_spawner = Node(
#     package="controller_manager",
#     executable="spawner",
#     arguments=["pulley_xy_effort_controller", "--controller-manager", "/controller_manager"],
# )

#     set_xy_effort_zero = ExecuteProcess(
#         cmd=['ros2', 'topic', 'pub', '-1', '/pulley_xy_effort_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0.0, 0.0]}'],
#         output='screen'
#     )

#     xy_zero_handler = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=pulley_xy_effort_spawner,
#             on_start=[set_xy_effort_zero]
#         )
#     )

#     bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=[
#             '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
#             '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
#             # '/world/empty/state@ros_gz_interfaces/msg/WorldState[gz.msgs.WorldState',
#             '/model/acts/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
#         ],
#         output='screen'
#         )

#     actions.append(delayed_controller)
#     actions.append(xy_zero_handler)
#     actions.append(joint_state_broadcaster_spawner)
#     actions.append(position_controller_spawner)
#     actions.append(bridge)

    return actions


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description

    Returns
    -------
    LaunchDescription
        The launch description object

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "desired_position",
            default_value="[1.0, 1.0, 5.0]",
            description="Desired position the drone should reach",
        ),
        DeclareLaunchArgument(
            "desired_velocity",
            default_value="[0.0, 0.0, 0.0]",
            description="Desired velocity the drone should maintain",
        ),
        DeclareLaunchArgument(
            "fixed",
            default_value="false",
            choices=["true", "false"],
            description="Whether to fix the drone in a fixed ball joint",
        ),
        OpaqueFunction(function=launch_setup),
    ])
