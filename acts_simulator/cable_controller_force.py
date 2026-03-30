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

        self.target_tension = 20.0
        self.Kp = 5.0
        self.Ki = 2.0
        self.integral_error = 0.0

        self.joint_name = "joint_pulley_B_z" 
        
        self.current_effort = None 
        self.timer = self.create_timer(0.1, self.control_loop)

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

        error = self.target_tension - self.current_effort
        self.integral_error += error * 0.1 # 0.1 is your timer period
        effort_command = (error * self.Kp) + (self.integral_error * self.Ki)

        self.get_logger().info(f"Sending Effort: {effort_command:.2f}")

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