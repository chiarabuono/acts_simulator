import os
import tempfile
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

    xacro_hooks = os.path.join(pkg_share, 'urdf', 'ground_hooks.xacro')
    robot_hooks_config = xacro.process_file(xacro_hooks)
    robot_desc_hooks = robot_hooks_config.toxml()
    spawn_system_hooks = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hooks',
            '-string', robot_desc_hooks,
            '-z', '0.0' 
        ],
        output='screen'
    )

    # ── Resolve yaml path and pass it into xacro ──────────────────────
    # controller_config = os.path.join(pkg_share, 'config', 'cable_controller.yaml')

    xacro_file = os.path.join(pkg_share, 'urdf', 'acts_model.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 1. Create a temporary file and write the xacro/urdf string to it
    with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf') as tmp_file:
        tmp_file.write(robot_desc)
        tmp_file_path = tmp_file.name

    # 2. Pass the file path instead of the giant string
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', tmp_file_path, '-name', 'acts']
    )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    # )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'joint_state_broadcaster',
    #         '--controller-manager', '/controller_manager',
    #     ],
    # )


    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(function=clean_function),
            LogInfo(msg=["Simulation - Cleaning up Gazebo processes!"]),
        ])
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        spawn_system,
        # spawn_system_hooks,
        # robot_state_publisher,
        clock_bridge,
        # joint_state_broadcaster_spawner,
        shutdown_handler,
    ])