import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    AppendEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from os import popen

PACKAGE_NAME = "acts_simulator"
WORLD = "simple"
WAIT_TIME = 5.0 

def clean_function(_: LaunchContext) -> None:
    popen("pkill -x gz")
    popen("pkill -x ruby")

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share)
    )

    world_file = os.path.join(pkg_share, 'worlds', f'{WORLD}.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    controller_config = os.path.join(pkg_share, 'config', 'acts_controller.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'acts_model.xacro')
    
    # Process Xacro normally without namespace forcing
    robot_desc = xacro.process_file(
        xacro_file,
        mappings={'controller_config': controller_config}
    ).toxml()

    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'acts', '-string', robot_desc, '-z', '0.0'],
        output='screen'
    )

    # 1. Global Robot State Publisher (No namespace sandbox)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )
    
    # 2. Global Spawners targeting the root controller manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    delayed_controllers = TimerAction(
        period=WAIT_TIME,
        actions=[
            joint_state_broadcaster_spawner,
            position_controller_spawner,
        ]
    )

    # 3. Clean global bridge mapping
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Fixed right-bracket token parsing error
            '/robot_description@std_msgs/msg/String[gz.msgs.String@/robot_description'
        ],
        output='screen'
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(function=clean_function),
            LogInfo(msg=["Simulation - Cleaning up Gazebo processes!"]),
        ])
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        clock_bridge,
        spawn_system,
        node_robot_state_publisher, 
        delayed_controllers, 
        shutdown_handler,
    ])