"""
File: rsp.launch.py
Project: Quadrotor Control Lab
File Created: Tuesday, 25th November 2025 9:38:28 AM
Author: nknab
Email: kojo.anyinam-boateng@ls2n.fr
Version: 1.0.0
Brief: Launch file to publish the drone's state.
-----
Last Modified: Tuesday, 25th November 2025 10:45:29 PM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join
from pathlib import Path

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

# CONSTANTS
PACKAGE_NAME = "acts_simulator"
PKG_SHARE = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)


def launch_setup(context: LaunchContext) -> list[Node]:
    """
    Function to setup the robot_state_publisher node

    Parameters
    ----------
    context : LaunchContext
        The launch context

    Returns
    -------
    list[Node]
        The robot_state_publisher node

    """

    # Get the package share directory
    namespace = LaunchConfiguration("namespace").perform(context)
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_jsp_gui = LaunchConfiguration("use_jsp_gui").perform(context)
    fixed = LaunchConfiguration("fixed").perform(context)

    use_sim_time = sim_mode.lower() == "true"

    xacro_file = join(PKG_SHARE, "urdf", "crazy2fly.urdf.xacro")
    xacro_args = [f"namespace:={namespace}", f"fixed:={fixed}"]

    # Generate robot description using Command for launch-time processing
    robot_description = Command([
        f"xacro {xacro_file} ",
        " ".join(xacro_args),
    ])

    # Create robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # RVIZ Node
    rviz_config_file = join(PKG_SHARE, "rviz", f"{rviz_config}.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        namespace=namespace,
        condition=IfCondition(use_rviz),
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        namespace=namespace,
        condition=IfCondition(use_jsp_gui),
    )

    return [robot_state_publisher, rviz, joint_state_publisher_gui]


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to start the robot_state_publisher node

    Returns
    -------
    LaunchDescription
        The launch description

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace to apply to the robot",
        ),
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false",
            choices=["true", "false"],
            description="Determine if the robot is in simulation mode",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Whether to start RVIZ",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="view_crazy2fly",
            choices=[
                path.stem
                for path in Path(join(PKG_SHARE, "rviz")).glob("*.rviz")
                if path.is_file()
            ],
            description="RVIZ config file name (without .rviz extension)",
        ),
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            choices=["true", "false"],
            description="Launch the joint state publisher GUI",
        ),
        DeclareLaunchArgument(
            "fixed",
            default_value="false",
            choices=["true", "false"],
            description="Whether to fix the drone in a fixed ball joint",
        ),
        OpaqueFunction(function=launch_setup),
    ])
