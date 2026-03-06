"""
File: spawn.launch.py
Project: Quadrotor Control Lab
File Created: Tuesday, 25th November 2025 11:17:22 AM
Author: nknab
Email: kojo.anyinam-boateng@ls2n.fr
Version: 1.0.0
Brief: Launch file to spawn the crazy2fly drone in Gazebo.
-----
Last Modified: Tuesday, 25th November 2025 3:42:08 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import tempfile

# CONSTANTS
PACKAGE_NAME = "acts_simulator"

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

def replace_placeholder(config_file: str, placeholder: str, replacement: str) -> str:
    """
    Replace a placeholder in a config file with a given replacement string

    Parameters
    ----------
    config_file : str
        Path to the config file

    placeholder : str
        The placeholder string to replace

    replacement : str
        The string to replace the placeholder with

    Returns
    -------
    str
        Path to the temporary file with the placeholder replaced

    """
    with open(config_file, "r") as f:
        content = f.read()

    rep_value = f"{replacement}/" if replacement else ""
    content_with_ns = content.replace(f"<{placeholder}>/", rep_value)

    with tempfile.NamedTemporaryFile(
        delete=False, mode="w", suffix=".yaml"
    ) as temp_file:
        temp_file.write(content_with_ns)
        return temp_file.name

def launch_setup(
    context: LaunchContext,
) -> list[Node | IncludeLaunchDescription | RegisterEventHandler]:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context object to get the launch configuration

    Returns
    -------
    list[Node | IncludeLaunchDescription | RegisterEventHandler]
        The list of launch nodes to execute and the launch description
        to include in the launch file.

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    namespace = LaunchConfiguration("namespace").perform(context)

    # Spawn robot at the given position
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)

    sim_mode = LaunchConfiguration("sim_mode").perform(context)

    # Robot State Publisher node
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "rsp.launch.py")]),
        launch_arguments={
            "sim_mode": sim_mode,
            "use_jsp_gui": "false",
            "namespace": namespace,
        }.items(),
    )

    x, y, z = parse_float_list(xyz, 3, "xyz")
    roll, pitch, yaw = parse_float_list(rpy, 3, "rpy")

    use_sim_time = sim_mode.lower() == "true"

    robot_description_topic = (
        "robot_description" if namespace == "" else f"/{namespace}/robot_description"
    )
    robot_name = "crazy2fly" if namespace == "" else f"{namespace}"

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-topic",
            robot_description_topic,
            "-name",
            robot_name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            str(roll),
            "-P",
            str(pitch),
            "-Y",
            str(yaw),
        ],
        output="screen",
    )

    # ROS-Gazebo bridge node
    bridge_params_file = replace_placeholder(
        join(pkg_share, "config", "gz_bridge", "gz_bridge.yaml"), "namespace", namespace
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params_file}",
        ],
    )

    return [rsp, spawn_entity, ros_gz_bridge]


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
            "namespace",
            default_value="",
            description="Namespace for the robot",
        ),
        DeclareLaunchArgument(
            "sim_mode",
            default_value="true",
            choices=["true", "false"],
            description="Determine if the robot is in simulation mode",
        ),
        DeclareLaunchArgument(
            "xyz",
            default_value="[0, 0, 0.0955]",
            description="X Y Z position of the robot (list format) in meters",
        ),
        DeclareLaunchArgument(
            "rpy",
            default_value="[0, 0, 0]",
            description="Roll Pitch Yaw of the robot (list format) in radians",
        ),
        OpaqueFunction(function=launch_setup),
    ])
