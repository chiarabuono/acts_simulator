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

def parse_float_list(value: str, expected_length: int, name: str) -> list[float]:
    """
    Parse a string representing a list of floats.

    Parameters
    ----------
    value : str
        The string to parse.
    expected_length : int
        How many floats to expect.
    name : str
        The name of the parameter (used for error messages).

    Raises
    ------
    ValueError
        If parsing fails or the wrong number of values is provided.

    Returns
    -------
    List[float]
        Parsed floats.

    """
    if value is None:
        raise ValueError(
            f"Missing value for '{name}'. Expected {expected_length} comma-separated numbers."
        )

    s = value.strip()

    if s.startswith("[") and s.endswith("]"):
        s = s[1:-1]

    parts = [p.strip() for p in s.split(",") if p.strip() != ""]

    if len(parts) != expected_length:
        raise ValueError(
            f"Invalid format for '{name}': expected {expected_length} comma-separated values but got {len(parts)}."
            f" Example: '[{', '.join(['0'] * expected_length)}]'"
        )

    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise ValueError(
            f"Invalid numeric value in '{name}': '{value}'. Ensure all entries are numeric (e.g., '[0, 0, 0.056]')."
        ) from e


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

def get_drone_nodes_control(sim, prefix, actions):
    drone_start = Node(
        package=PACKAGE_NAME,
        executable="drone_start",
        name=f"{prefix}start",
        output="screen",
        parameters=[{
            "action_time": WAIT_TIME
        }],
        remappings=[
            ("command/motor_speed", f"/{prefix}/command/motor_speed"),
            ("mocap/odom", f"/{prefix}mocap/odom"),
        ]

    )

    delayed_drone_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sim,
            on_start=TimerAction(period=WAIT_TIME, actions=[drone_start]),
        )
    )

    # 2. The ROS-GZ Bridge Node
    # This replaces the manual 'ros2 run ros_gz_bridge...' command
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Motor Speed: ROS -> GZ (@ is bidirectional, which works fine here)
            f"/{prefix}/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            # Odometry: GZ -> ROS ([ means Gazebo to ROS)
            f"/{prefix}mocap/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        ],
        output="screen"
    )

    actions.append(delayed_drone_start)
    actions.append(bridge_node)


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

    desired_position = LaunchConfiguration("desired_position").perform(context)
    desired_velocity = LaunchConfiguration("desired_velocity").perform(context)

    fixed = LaunchConfiguration("fixed").perform(context)

    d_position = parse_float_list(desired_position, 3, "xyz")
    d_velocity = parse_float_list(desired_velocity, 3, "xyz")

    sim = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " launch ",
                PathJoinSubstitution([
                    pkg_share,
                    "launch",
                    "multi_uav_simulation.launch.py",
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

    get_drone_nodes_control(sim, "drone1_", actions)
    get_drone_nodes_control(sim, "drone2_", actions)

    
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
