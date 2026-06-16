import os
from os import popen
import xacro
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    AppendEnvironmentVariable
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from simple_launch import SimpleLauncher, GazeboBridge
import subprocess as sp
from acts_simulator.generate_sdf import compile_xacro_to_loop_sdf


PACKAGE_NAME = "acts_simulator"
WORLD = "simple"
WAIT_TIME = 10.0 

sl = SimpleLauncher(use_sim_time = True)
sl.declare_arg('sliders', default_value=True)
sl.declare_arg('robot', default_value='acts')

def clean_function(_: LaunchContext) -> None:
    popen("pkill -x gz")
    popen("pkill -x ruby")

def launch_setup():
    robot = sl.arg('robot')

    # 1. Force the world name convention for the bridges
    GazeboBridge.set_world_name('empty')

    # 2. AUTOMATICALLY LAUNCH GAZEBO IN THE BACKGROUND
    # This fires up 'gz sim' with the '-r' (run) flag automatically
    sl.gz_launch('empty.sdf', '-r')

    # 3. Run your clean compilation pipeline
    try:
        final_model_path = compile_xacro_to_loop_sdf("acts_simulator", "urdf/acts_model.xacro")
    except Exception as e:
        print(f"\n[FATAL ERROR] Compilation pipeline failed: {e}\n")
        return []

    # 4. Spawn using 'model_file'
    sl.spawn_gz_model(
        name=robot,
        model_file=final_model_path,
        spawn_args=['-z', '0.0']
    )

    # ROS-Gz bridges
    bridges = [GazeboBridge.clock()]
    bridges.append((GazeboBridge.model_prefix(robot) + '/joint_state',
                    'joint_states',
                    'sensor_msgs/JointState',
                    GazeboBridge.gz2ros))

    sl.create_gz_bridge(bridges)

    if sl.arg('sliders'):
        sl.node('slider_publisher', arguments=[sl.find('gz_attach_links', 'effort_manual.yaml')])
    
    return sl.launch_description()
generate_launch_description = sl.launch_description(launch_setup)