import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller_force')
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.pub_effort = self.create_publisher(Float64MultiArray, 'motor_commands', qos_profile)
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('joint_name', "joint_pulley"),
                ('target_tension', 0.0),
                ('Kp', 0.0),
                ('Ki', 0.0),
                ('Kd', 0.0),
                ('num_segments', 20),
                ('cable_extreme', 'link_first'),
                ('winding_type', 'under')
            ]
        )
 
        self.cable_extreme = self.get_parameter('cable_extreme').get_parameter_value().string_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value

        if self.cable_extreme == 'link_first':
            self.direction_factor = 1.0
        else:
            self.direction_factor = -1.0

        winding_type = self.get_parameter('winding_type').value
        if winding_type == 'over':
            self.direction_factor *= -1.0
           
        # Controller params
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value

        self.target_tension = self.get_parameter('target_tension').get_parameter_value().double_value

        self.integral_error = 0.0
        self.last_error = 0.0
        self.max_effort = 1000.0
        
        self.current_effort = None 
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def joint_state_callback(self, msg):
        try:
            if self.joint_name in msg.name:
                idx = msg.name.index(self.joint_name)
                val = msg.effort[idx]
                
                if np.isfinite(val):
                    self.current_effort = val
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        if self.current_effort is None:
            # self.get_logger().info(f"No effort coming")
            return 
        
        # P Controller
        # effort_command = (error * self.Kp)

        position_error = self.target_tension - self.current_effort
        self.integral_error += position_error * self.timer_period
        
        # PI Controller
        # effort_command = (error * self.Kp) + (self.integral_error * self.Ki)

        derivative = (position_error - self.last_error) / self.timer_period
        self.last_error = position_error

        # PID Controller
        effort_command = (position_error * self.Kp) + (self.integral_error * self.Ki) + (derivative * self.Kd)
        

        effort_command = np.clip(effort_command, -self.max_effort, self.max_effort)

        self.get_logger().info(f"Sending Effort: {effort_command:.2f}")

        if self.target_tension < 0:
            effort_command = effort_command * self.direction_factor * -1.0
        else:
            effort_command = effort_command * self.direction_factor

        msg = Float64MultiArray()
        msg.data = [float(effort_command)]
        self.pub_effort.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CableController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()