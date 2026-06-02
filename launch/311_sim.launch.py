import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('acts_simulator')
    
    # 1. Load and parse Xacro properly (with mappings)
    xacro_file = os.path.join(pkg_share, 'urdf', 'crazy2fly.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file, mappings={'namespace': '', 'fixed': 'false'}).toxml()
    
    # 2. Path to your validated MuJoCo physics XML file
    mujoco_model_file = os.path.join(pkg_share, 'mujoco', 'crazy2fly.xml')
    
    # 3. Path to your controller settings
    controller_config = os.path.join(pkg_share, 'config', 'acts_controller.yaml')

    # 4. Launch the MuJoCo Simulator Server Node
    mujoco_sim_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",  # <--- Core MuJoCo Simulator Binary Execution target
        parameters=[
            {"robot_description": robot_description_raw},
            {"mujoco_model_path": mujoco_model_file}, # Check if parameter name is robot_model_path or mujoco_model_path
            {"show_viewer": True},
            controller_config
        ],
        output="screen"
    )

    # 5. Broadcast coordinate states so RViz can locate the links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}]
    )

    # 6. Launch RViz
    rviz_file = os.path.join(pkg_share, 'rviz', 'view_crazy2fly.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_file],
        output="screen"
    )

    return LaunchDescription([
        mujoco_sim_node,
        robot_state_publisher,
        rviz_node
    ])