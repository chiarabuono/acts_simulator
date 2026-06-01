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
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'adaptable_cable.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

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

    # SPAWN THE ENTIRE SYSTEM
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cable',
            '-string', robot_desc,
            '-z', '0.0'
        ],
        output='screen'
    )


    """
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    """

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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
        shutdown_handler,
        clock_bridge
    ])