import os
from os import popen
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from acts_simulator.utils_controller import parse_float_list, get_drone_nodes_position_control, get_drone_start

PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 10.0  # wait for Gazebo + controllers to be ready

def clean_function(_: LaunchContext) -> None:
    popen("pkill -x gz")
    popen("pkill -x ruby")

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # ── Launch simulation ─────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'acts_sim.launch.py')
        )
    )

    # ── UGV tip controller node (your Python controller) ──────────────
    ugv_controller_node = Node(
        package=PACKAGE_NAME,
        executable='ugv_tip_controller',
        name='ugv_tip_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cable_length':   1.5,
            'motion_duration': 3.0,
            # UGV1 — hangs from panel at (0.0, 0.0, 1.5)
            'ugv1_panel_x':  0.0,
            'ugv1_panel_y':  0.0,
            'ugv1_panel_z':  1.5,
            'ugv1_target_x': 1.0,
            'ugv1_target_y': 1.0,
            # UGV2 — hangs from panel at (0.5, 0.5, 1.5)
            'ugv2_panel_x':  0.5,
            'ugv2_panel_y':  0.5,
            'ugv2_panel_z':  1.5,
            'ugv2_target_x': -1.0,
            'ugv2_target_y': -1.0,
        }]
    )

    delayed_controller = TimerAction(
        period=WAIT_TIME,
        actions=[ugv_controller_node]
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(function=clean_function),
            LogInfo(msg=["ACTS - Cleaning up after shutdown!"]),
        ])
    )

    return LaunchDescription([
        sim,
        delayed_controller,
        shutdown_handler,
    ])