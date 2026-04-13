from launch_ros.actions import Node
from launch.actions import (
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart

PACKAGE_NAME = "acts_simulator"
WAIT_TIME = 8.0

def parse_float_list(value: str, expected_length: int, name: str) -> list[float]:
    """
    Parse a string representing a list of floats.

    Parameters
    ----------
    value : str
        The string to parse.
    expected_length : int
        How many floats to expect.
    name : str
        The name of the parameter (used for error messages).

    Raises
    ------
    ValueError
        If parsing fails or the wrong number of values is provided.

    Returns
    -------
    List[float]
        Parsed floats.

    """
    if value is None:
        raise ValueError(
            f"Missing value for '{name}'. Expected {expected_length} comma-separated numbers."
        )

    s = value.strip()

    if s.startswith("[") and s.endswith("]"):
        s = s[1:-1]

    parts = [p.strip() for p in s.split(",") if p.strip() != ""]

    if len(parts) != expected_length:
        raise ValueError(
            f"Invalid format for '{name}': expected {expected_length} comma-separated values but got {len(parts)}."
            f" Example: '[{', '.join(['0'] * expected_length)}]'"
        )

    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise ValueError(
            f"Invalid numeric value in '{name}': '{value}'. Ensure all entries are numeric (e.g., '[0, 0, 0.056]')."
        ) from e



def get_drone_start(sim, prefix, actions):
    drone_start = Node(
        package=PACKAGE_NAME,
        executable="drone_start",
        name=f"{prefix}start",
        output="screen",
        parameters=[{
            "action_time": WAIT_TIME
        }],
        remappings=[
            ("command/motor_speed", f"/{prefix}/command/motor_speed"),
            ("mocap/odom", f"/{prefix}mocap/odom"),
        ]
    )

    delayed_drone_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sim,
            on_start=TimerAction(period=WAIT_TIME, actions=[drone_start]),
        )
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/{prefix}/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            f"/{prefix}mocap/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/{prefix}tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ],
        output="screen"
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    actions.append(delayed_drone_start)
    actions.append(bridge_node)
    actions.append(clock_bridge)



def get_drone_nodes_position_control(sim, prefix, d_position, d_velocity, actions):

    control = Node(
        package=PACKAGE_NAME,
        executable="controller_node",
        name="controller",
        namespace=prefix.strip('/'),
        output="screen",
        parameters=[{
            "control_mode": "position",
            "desired_position": d_position,
            "desired_velocity": d_velocity,
            "mass": 2.0 + 0.1,
            "use_sim_time": True
        }],
        remappings=[
            ("command/motor_speed", f"/{prefix}/command/motor_speed"),
            ("mocap/odom", f"/{prefix}mocap/odom"),
        ]
    )

    delayed_drone_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sim,
            on_start=TimerAction(period=WAIT_TIME, actions=[control]),
        )
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/{prefix}/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            f"/{prefix}mocap/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/{prefix}tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ],
        output="screen"
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    actions.append(delayed_drone_control)
    actions.append(bridge_node)
    actions.append(clock_bridge)


def get_drone_nodes_force_control(sim, prefix, d_force, d_velocity, actions):
    control = Node(
        package=PACKAGE_NAME,
        executable="controller_node",
        name="controller",
        namespace=prefix.strip('/'),
        output="screen",
        parameters=[{
            "control_mode": "force",      # <-- add this
            "desired_force": d_force,
            "desired_velocity": d_velocity,
            "mass": 2.0,
            "use_sim_time": True
        }],
        remappings=[
            ("command/motor_speed", f"/{prefix}/command/motor_speed"),
            ("mocap/odom", f"/{prefix}mocap/odom"),
        ]
    )

    delayed_drone_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=sim,
            on_start=TimerAction(period=WAIT_TIME, actions=[control]),
        )
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/{prefix}/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            f"/{prefix}mocap/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            f"/{prefix}tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        ],
        output="screen"
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    actions.append(delayed_drone_control)
    actions.append(bridge_node)
    actions.append(clock_bridge)
