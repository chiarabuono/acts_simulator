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
from os import popen

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = "acts_simulator"

def clean_function(_: LaunchContext) -> None:
    """Cleans up background processes on shutdown."""
    popen("pkill -x gz")
    popen("pkill -x ruby")

def get_drone_nodes(xacro_path, prefix, x, y, z):
    """
    Generates the nodes required for one drone. 
    Includes the Gazebo spawner and the Robot State Publisher.
    """
    
    # Process Xacro once per drone instance
    robot_description_config = xacro.process_file(
        xacro_path,
        mappings={
            'id':        prefix.split('e')[1].strip("_"),
            'prefix':    prefix,
            #'namespace': prefix,                
            # 'x': str(x),
            # 'y': str(y),
            # 'z': str(z),
        }
    )
    robot_desc = robot_description_config.toxml()

    # 1. Spawn the drone entity into Gazebo
    spawn_uav = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=prefix,
        arguments=[
            '-name', prefix,
            '-id', id,
            '-string', robot_desc,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z)
        ],
        output='screen'
    )

    # 2. Robot State Publisher (Required for TF / RViz)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=prefix,
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # This is wrong
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        namespace=prefix.strip('/'),
        arguments=[
            # Motor Commands
            f"/{prefix.strip('/')}/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators",
            # Odometry
            f"/{prefix.strip('/')}mocap/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # ADD THIS: Joint States Bridge
            # Note: We use the EXACT name seen in 'gz topic -l'
            f"/{prefix.strip('/')}joint_states@sensor_msgs/msg/JointState[gz.msgs.Model" 
        ],
        output="screen"
    )

    return [spawn_uav, rsp]

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
    
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    xacro_path = os.path.join(pkg_share, 'urdf', 'uav_cable_usable.urdf.xacro')
    
    # Add URDF models to Gazebo path
    model_path = os.path.dirname(pkg_share)
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )

    # Start Gazebo Sim (Ogre2 engine is usually more efficient)
    # Using '--verbose 1' to keep logs clean; use '4' if debugging physics
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf --verbose 1'}.items(),
    )

    # Collect all actions
    actions = [
        set_gz_resource_path,
        gazebo,
    ]

    # Drone 1
    actions.extend(get_drone_nodes(xacro_path, "drone1_", 0.0, 0.0, 0.1))
    
    # Drone 2
    actions.extend(get_drone_nodes(xacro_path, "drone2_", 1.0, 1.0, 0.1))

    # Cleanup logic
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg="Simulation - Cleaning up processes!"),
            ]
        )
    )
    actions.append(shutdown_handler)

    return actions

def generate_launch_description() -> LaunchDescription:
    """Entry point for the ROS 2 launch system."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "fixed",
            default_value="false",
            choices=["true", "false"],
            description="Whether to fix the drone (handled in Xacro mappings if implemented)",
        ),
        OpaqueFunction(function=launch_setup),
    ])