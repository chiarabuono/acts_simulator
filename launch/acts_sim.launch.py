import os
import xacro
from os import popen
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    OpaqueFunction, 
    RegisterEventHandler,
    LogInfo,
    AppendEnvironmentVariable
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from acts_simulator.utils_simulation import get_drone_spawn_data, send_actsInfo_toxacro, create_actsXacro_file


# CONSTANTS
PACKAGE_NAME = "acts_simulator"
GAZEBO_VERBOSE_LEVEL = 4
WORLD = "simple"

def clean_function(_: LaunchContext) -> None:
    """Kills lingering processes on exit to prevent the Ruby ESRCH error."""
    popen("pkill -x gz")
    popen("pkill -x ruby")

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # START GAZEBO
    model_path = os.path.dirname(pkg_share)
    
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )

    world_file = os.path.join(get_package_share_directory('acts_simulator'), 'worlds', f'{WORLD}.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # config_file_path = os.path.join(pkg_share, 'config', 'acts_config.json')
    # drones, p_xyz, p_rpy, cable_rpys = get_drone_spawn_data(config_file_path)

    # USE acts.urdf.xacro file - send info
    xacro_file = os.path.join(pkg_share, 'urdf', 'acts_model.xacro')
    # robot_desc = send_actsInfo_toxacro(xacro_file, p_xyz, p_rpy, drones, cable_rpys)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # SPAWN THE ENTIRE SYSTEM
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acts',
            '-string', robot_desc,
            '-z', '0.5' 
        ],
        output='screen'
    )



    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["Simulation - Cleaning up Gazebo processes!"]),
            ]
        )
    )

    return LaunchDescription([
        set_gz_resource_path, 
        gazebo,
        spawn_system,
        node_robot_state_publisher,
        node_joint_state_publisher, 
        shutdown_handler,
    ])