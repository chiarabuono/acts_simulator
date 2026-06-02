import os
import xacro
from os import popen
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
    TimerAction,
)

from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from acts_simulator.utils_controller import parse_float_list, get_drone_nodes_position_control, get_drone_start

PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 10.0  # wait for Gazebo + controllers to be ready

def clean_function(_: LaunchContext) -> None:
    popen("pkill -x gz")
    popen("pkill -x ruby")

def launch_setup(
    context: LaunchContext,
):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    # ── Launch simulation ─────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'acts_sim.launch.py')
        )
    )

    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
    fixed = LaunchConfiguration("fixed").perform(context)
    sim = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="ros2"),
                " launch ",
                PathJoinSubstitution([
                    pkg_share,
                    "launch",
                    "acts_sim.launch.py",
                ]),
                " headless:=false",
                " use_rviz:=false",
                f" fixed:={fixed}",
            ]
        ],
        name="sim",
        shell=True,
        output="screen",
    )

    controller_node = Node(
        package=PACKAGE_NAME,
        executable="ugv_controller",
        name="ugv_controller",
        output="screen",
        parameters=[{
            "mass": 0.001,
        }],

        remappings=[
        ]
    )

    delayed_controller = TimerAction(
        period=WAIT_TIME,
        actions=[controller_node]
    )

    shutdown_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            OpaqueFunction(function=clean_function),
            LogInfo(msg=["ACTS - Cleaning up after shutdown!"]),
        ])
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    actions = [
        sim,
        shutdown_handler,
        bridge,
        delayed_controller
    ]

    # get_drone_start(sim, "drone1_", actions)

    return actions

def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description

    Returns
    -------
    LaunchDescription
        The launch description object

    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "fixed",
            default_value="false",
            choices=["true", "false"],
            description="",
        ),
        OpaqueFunction(function=launch_setup),
    ])
