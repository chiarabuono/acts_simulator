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
from acts_simulator.utils import get_drone_spawn_data


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
    
    # PROCESS THE ACTS SYSTEM (UAV + Panel + Cable)
    xacro_file = os.path.join(pkg_share, 'urdf', 'acts.urdf.xacro')

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

    config_file_path = os.path.join(pkg_share, 'config', 'acts_config.json')
    drones, p_xyz, p_rpy, cable_rpys = get_drone_spawn_data(config_file_path)

    xacro_mappings = {
        'panel_x': str(p_xyz[0]),
        'panel_y': str(p_xyz[1]),
        'panel_z': str(p_xyz[2]),
        'panel_R': str(p_rpy[0]),
        'panel_P': str(p_rpy[1]),
        'panel_Y': str(p_rpy[2]),
    }

    for i, (drone, (cr, cp, cy)) in enumerate(zip(drones, cable_rpys)):
        prefix = f"drone{drone['id']}_"
        xacro_mappings.update({
            f'{prefix}id' : str(drone['id']), 
            f'{prefix}x': str(drone['drone_xyz_world'][0]),
            f'{prefix}y': str(drone['drone_xyz_world'][1]),
            f'{prefix}z': str(drone['drone_xyz_world'][2]),
            f'{prefix}attach_x': str(drone['attach_xyz_panel'][0]),
            f'{prefix}attach_y': str(drone['attach_xyz_panel'][1]),
            f'{prefix}attach_z': str(drone['attach_xyz_panel'][2]),
            f'{prefix}panel_link': str(drone['panel_link']),
            f'{prefix}len': str(drone['length']),
            f'{prefix}roll': str(cr),
            f'{prefix}pitch': str(cp),
            f'{prefix}yaw': str(cy),
        })

    robot_desc = xacro.process_file(xacro_file, mappings=xacro_mappings).toxml()

    # SPAWN THE ENTIRE SYSTEM
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acts_system',
            '-string', robot_desc,
            '-z', '0.0' 
        ],
        output='screen'
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # xacro_file2 = os.path.join(pkg_share, 'urdf', 'track.xacro')
    # robot_description_config2 = xacro.process_file(xacro_file2)
    # robot_desc2 = robot_description_config2.toxml()
    # spawn_reference = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'ref',
    #         '-string', robot_desc2,
    #         '-z', '0.0'  # Spawn it higher so the cables have room to hang
    #     ],
    #     output='screen'
    # )


    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # TURTLEBOT SETUP
    # tb3_pkg_path = get_package_share_directory('turtlebot3_description')
    # tb3_urdf_path = os.path.join(tb3_pkg_path, 'urdf', 'turtlebot3_burger.urdf')

    # spawn_turtlebot = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'my_turtlebot',
    #         '-file', tb3_urdf_path,
    #         '-x', '2.0', '-y', '0.0', '-z', '0.1'
    #     ],
    #     output='screen',
    # )

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
        # spawn_reference,
        shutdown_handler,
    ])