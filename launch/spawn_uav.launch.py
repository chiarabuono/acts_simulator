import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'acts_simulator'
    pkg_share = get_package_share_directory(package_name)
    
    # 1. POINT TO THE MASTER ASSEMBLY FILE
    # This is the file that includes panel, uav, and cable
    xacro_file = os.path.join(pkg_share, 'urdf', 'acts.urdf.xacro')
    
    # 2. PROCESS THE XACRO
    # This automatically finds all the 'included' files
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. START GAZEBO
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. SPAWN THE ENTIRE SYSTEM
    spawn_system = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'acts_towed_system',
            '-string', robot_desc,
            '-z', '2.0'  # Spawn it higher so the cables have room to hang
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_system,
    ])