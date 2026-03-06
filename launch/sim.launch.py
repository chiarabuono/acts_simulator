"""
File: sim.launch.py
Project: Quadrotor Control Lab
File Created: Tuesday, 25th November 2025 11:15:07 AM
Author: nknab
Email: kojo.anyinam-boateng@ls2n.fr
Version: 1.0.0
Brief: Launch file to start the simulation.
       FOR TESTING PURPOSES ONLY
       This launch file is not intended for production use.
-----
Last Modified: Tuesday, 25th November 2025 11:17:00 AM
Modified By: nknab
-----
Copyright ©2025 nknab
"""

from os.path import join
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# CONSTANTS
PACKAGE_NAME = "acts_simulator"
GAZEBO_VERBOSE_LEVEL = 4
WORLD = "simple"


def launch_setup(context: LaunchContext) -> list[IncludeLaunchDescription]:
    """
    Setup the launch configuration

    Parameters
    ----------
    context : LaunchContext
        The launch context

    Returns
    -------
    list[IncludeLaunchDescription]
        The list of launch nodes to execute

    """

    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Get the launch configuration variables
    headless = LaunchConfiguration("headless").perform(context) == "true"

    # Gazebo launch file
    world_filepath = join(pkg_share, "worlds", f"{WORLD}.world")

    gz_args = f"-r -v{GAZEBO_VERBOSE_LEVEL} {world_filepath} "

    if headless:
        gz_args += "--headless-rendering -s"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ]),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(pkg_share, "launch", "spawn.launch.py")]),
        launch_arguments={
            "sim_mode": "true",
        }.items(),
    )

    return [gazebo, spawn]


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description

    Returns
    -------
    LaunchDescription
        The launch description object

    """
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    model_path = os.path.dirname(pkg_share)

    return LaunchDescription([
        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["true", "false"],
            description="Run Gazebo in headless mode (no GUI)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
