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
)
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 5.0


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

    pulley_a_extreme = "link_first"
    pulley_b_extreme = "link_last"

    # Pulley A (Position) setup
    orientation_a = "3.1415" if pulley_a_extreme == "link_first" else "0.0"

    init_x = "0.0"
    init_y = "0.0"
    init_z = "7.0"              # Make it consistent with pulley_z, cable_len and unwinded_cable_len

    # PULLEY A
    pulley_x = "0.0"
    pulley_y = "0.0"
    pulley_z = "5.0"

    cable_len = "4.0"
    unwinded_len = "2.0"
    final_unwinded_cable_len = "3.0"

    # PULLEY B
    # init_x = "0.0"
    # init_y = "0.0"
    # init_z = "2.0"


    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    fixed = LaunchConfiguration("fixed").perform(context)


    sim = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " launch ",
                PathJoinSubstitution([pkg_share, "launch", "cable_simulation.launch.py",]),
                f" x:={init_x} y:={init_y} z:={init_z} length:={cable_len} orientation:={orientation_a}",
                " headless:=false",
                " use_rviz:=false",
                f" fixed:={fixed}",
            ]
        ],
        name="sim",
        shell=True,
        output="screen",
    )

    position_controller_node = Node(
        package=PACKAGE_NAME,
        executable="cable_controller_position",
        name="controller_pulley_a",
        output="screen",
        parameters=[{
            "current_x": float(init_x),
            "current_y": float(init_y),
            "current_z": float(init_z),
            "total_cable_len": float(cable_len),
            "cable_extreme": pulley_a_extreme,
            "vel": 0.01,
            "unwinded_cable_len": float(unwinded_len),
            "final_cable_len": float(final_unwinded_cable_len),
            "pulley_x": float(pulley_x),
            "pulley_y": float(pulley_y),
            "pulley_z": float(pulley_z),            
        }],

        remappings=[
            ("motor_commands", "/forward_position_controller/commands")
        ]
    )

    force_controller_node = Node(
        package=PACKAGE_NAME,
        executable="cable_controller_force",
        name="controller_pulley_b",
        output="screen",
        parameters=[{    
            "target_tension": 20.0,
            "max_effort": 100.0,
            "Kp": 5.0,
            "Ki": 2.0, 
            "Kd": 0.5, 
            "joint_name": 'joint_pulley_B_z',
            "cable_extreme": pulley_b_extreme,
            "winding_type" : "under"
        }],

        remappings=[
            ("motor_commands", "effort_controller/commands")
        ]
    )

    visibility_node = Node(
        package=PACKAGE_NAME,
        executable='cable_visibility',
        name='cable_visibility_manager',
        output='screen'
    ) 

    delayed_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sim,
            on_start=TimerAction(period=WAIT_TIME, actions=[force_controller_node]),
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "--controller-manager", "/controller_manager"],
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["Cable Simulation - Cleaning up after shutdown!"]),
            ]
        )
    )

    return [sim, # visibility_node, 
            delayed_controller, 
            joint_state_broadcaster_spawner, 
            position_controller_spawner, effort_controller_spawner, 
            shutdown_handler]


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
