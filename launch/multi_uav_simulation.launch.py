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
import os
import xacro

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
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 8.0

pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

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


# def spawn_drone(fixed, prefix, x, y, z):
#     pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

#     return ExecuteProcess(
#         cmd=[
#             [
#                 FindExecutable(name="ros2"),
#                 " launch ",
#                 PathJoinSubstitution([pkg_share, "launch", "uav_spawner.launch.py",]),
#                 f" prefix:={prefix} x:={str(x)} y:={str(y)}", #z:={str(z)}
#                 " headless:=false",
#                 " use_rviz:=false",
#                 f" fixed:={fixed}",
#             ]
#         ],
#         name="sim",
#         shell=True,
#         output="screen",
#     )

def spawn_drone(prefix, x, y, z):
    xacro_file = os.path.join(pkg_share, 'urdf', 'uav.urdf.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={
            'prefix': prefix, 
            'x': f"{x}", 
            'y': f"{y}", 
            #'z': f"{z},
        }
    )
    robot_desc = robot_description_config.toxml()

    spawn_uav = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', prefix,
        '-string', robot_desc,
        '-x', f"{x}",
        '-y', f"{y}",
        #'-z', f"{z}"
    ],
    output='screen'
    )

    return spawn_uav

def state_publisher_node(robot_desc):
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    return node_robot_state_publisher

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

    fixed = LaunchConfiguration("fixed").perform(context)
    
    
    # START GAZEBO
    model_path = os.path.dirname(pkg_share)
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    drone1 = spawn_drone("drone1_", 1.0, 1.0, 1.0)
    drone2 = spawn_drone("drone2_", 0.0, 0.0, 0.0)
    


    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["Cable Simulation - Cleaning up after shutdown!"]),
            ]
        )
    )


    return [
        set_gz_resource_path, gazebo,
        drone1, drone2,
        shutdown_handler
    ]


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
            "fixed",
            default_value="false",
            choices=["true", "false"],
            description="Whether to fix the drone in a fixed ball joint",
        ),
        OpaqueFunction(function=launch_setup),
    ])