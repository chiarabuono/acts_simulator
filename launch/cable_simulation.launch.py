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
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = "acts_simulator"
GAZEBO_VERBOSE_LEVEL = 4
WORLD = "simple"

def clean_function(_: LaunchContext) -> None:
    """Kills lingering processes on exit to prevent the Ruby ESRCH error."""
    popen("pkill -x gz")
    popen("pkill -x ruby")

def launch_setup(context: LaunchContext, *args, **kwargs):
    """Function to process Xacro and setup nodes after context is loaded."""
    
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # 1. Resolve LaunchConfigurations to actual strings
    x_str = LaunchConfiguration('x').perform(context)
    y_str = LaunchConfiguration('y').perform(context)
    z_str = LaunchConfiguration('z').perform(context)
    length_str = LaunchConfiguration('length').perform(context)
    orientation_str = LaunchConfiguration('orientation').perform(context)
    segments_str = LaunchConfiguration('segments').perform(context)

    # 2. Process the primary Xacro (Cable System) with mappings
    xacro_file = os.path.join(pkg_share, 'urdf', 'adaptable_cable.xacro')
    robot_description_config = xacro.process_file(
        xacro_file, 
        mappings={
            'x': x_str, 
            'y': y_str, 
            'z': z_str, 
            'length': length_str,
            'orientation': orientation_str,
            'segments': segments_str
        }
    )
    robot_desc = robot_description_config.toxml()

    # 3. Define Nodes that require the processed robot_desc
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cable_system',
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

    # 4. Optional: Process and Spawn Pulley/Guide
    # (Note: If this also needs x,y,z, repeat the mappings logic here)
    pulley_xacro = os.path.join(pkg_share, 'urdf', 'cable_guide.urdf.xacro')
    pulley_desc = xacro.process_file(pulley_xacro).toxml()

    spawn_pulley = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pulley',
            '-string', pulley_desc,
            '-z', '0.0'
        ],
        output='screen'
    )

    # 5. Bridge for Clock and Commands
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/cable_system/joint/joint_pulley/0/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        output='screen'
    )

    return [
        spawn_system,
        spawn_pulley,
        node_robot_state_publisher,
        bridge
    ]

def generate_launch_description():
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # Environment Setup
    model_path = os.path.dirname(pkg_share)
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path
    )

    # Gazebo Server/Client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # Added -v 4 for verbose output to catch physics crashes
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # Shutdown Handler
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=clean_function),
                LogInfo(msg=["Simulation - Cleaning up Gazebo processes!"]),
            ]
        )
    )

    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('x', default_value='1.0', description='Initial X offset'),
        DeclareLaunchArgument('y', default_value='1.0', description='Initial Y offset'),
        DeclareLaunchArgument('z', default_value='1.0', description='Initial Z offset'),
        DeclareLaunchArgument('length', default_value='5.0', description='Total cable length'),
        DeclareLaunchArgument('orientation', default_value='0.0', description='Cable orientation'),
        DeclareLaunchArgument('segments', default_value='0.0', description='Cable number segments'),
        
        set_gz_resource_path, 
        gazebo,
        shutdown_handler,
        
        # This triggers the launch_setup function above
        OpaqueFunction(function=launch_setup),
    ])