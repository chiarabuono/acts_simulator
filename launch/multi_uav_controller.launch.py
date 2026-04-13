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

from acts_simulator.utils_controller import *

PACKAGE_NAME = "acts_simulator"

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

    desired_position = LaunchConfiguration("desired_position").perform(context)
    desired_velocity = LaunchConfiguration("desired_velocity").perform(context)

    d_position = parse_float_list(desired_position, 3, "xyz")
    d_velocity = parse_float_list(desired_velocity, 3, "xyz")

    d1_position = [2.0, 2.0, 10.0]
    d2_position = [-2.0, -1.0, 2.0]
    # d_velocity = [0.0, 0.0, 0.0]
    d1_force = [0.0, 0.0, 2.0 * 9.81]

    get_drone_nodes_force_control(sim, "drone1_", d1_force, d_velocity, actions)
    get_drone_nodes_position_control(sim, "drone2_", d2_position, d_velocity, actions)

    
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
