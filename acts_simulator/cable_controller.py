import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PulleyTester(Node):
    def __init__(self):
        # Initialize the node with your name
        super().__init__('pulley_tester_xyz')
        
        # Explicitly declare and set use_sim_time parameter
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)
        ])
        
        # Create the publisher
        self.pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # Prepare the message
        self.msg = Float64MultiArray()
        self.msg.data = [1.0, 1.0, 0.0] # [X, Y, Z]
        
        # Create a ROS 2 timer instead of using time.sleep()
        # This timer natively respects Gazebo's simulation clock
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.get_logger().info('Pulley tester node started with sim_time=True.')

    def timer_callback(self):
        self.pub.publish(self.msg)
        self.get_logger().info(f"Publishing target XYZ: {self.msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    # Use Object-Oriented style for cleaner ROS 2 execution
    pulley_tester = PulleyTester()
    
    try:
        # rclpy.spin processes the timer callbacks efficiently
        rclpy.spin(pulley_tester)
    except KeyboardInterrupt:
        pass
    finally:
        pulley_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()