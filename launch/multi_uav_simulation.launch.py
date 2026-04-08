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
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'multi_uavs.urdf.xacro')
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

    pkg_path = get_package_share_directory(PACKAGE_NAME)
    urdf_file = os.path.join(pkg_path, 'urdf', 'multi_uavs.urdf.xacro')

    # Define starting positions for the 4 elements
    positions = [
        {'name': 'drone1_', 'x': '1.0', 'y': '1.0'},
        {'name': 'drone2_', 'x': '-1.0', 'y': '1.0'},
        {'name': 'drone3_', 'x': '1.0', 'y': '-1.0'},
        {'name': 'drone4_', 'x': '-1.0', 'y': '-1.0'},
    ]

    nodes = []
    for pos in positions:
        robot_description_config = xacro.process_file(
            urdf_file, 
            mappings={
                'namespace': pos['name'], # 'drone1'
                'prefix': ''              # Leave prefix empty to avoid 'drone1_'
            }
        )
        robot_desc = robot_description_config.toxml()

        # 3. Spawn the entity
        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', pos['name'], 
                    '-string', robot_desc,
                    '-x', pos['x'],
                    '-y', pos['y'],
                    '-z', '0.0'
                ],
                output='screen'
            )
        )




    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["Simulation - Cleaning up Gazebo processes!"]),
            ]
        )
    )

    nodes.append(shutdown_handler)
    nodes.append(set_gz_resource_path)
    nodes.append(gazebo)

    return LaunchDescription(nodes)