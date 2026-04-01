import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PulleyTorquePublisher(Node):
    def __init__(self):
        super().__init__('pulley_torque_publisher')
        
        # Topic name must match your controller configuration
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.pub = self.create_publisher(Float64MultiArray, 'motor_commands', qos_profile)
        
        # Timer to publish at 10Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.declare_parameter('torque_value', 0.5)

        # Physical Constants (Estimate these based on your model)
        self.pulley_radius = 0.01  # meters
        self.cable_mass = 0.1    # kg
        self.gravity = 9.81
        
        # Calculate holding torque: T = m * g * r
        #self.holding_torque = self.cable_mass * self.gravity * self.pulley_radius # ~0.049 Nm
        self.declare_parameter('mode', 'hold') # Modes: 'wind', 'release', 'hold'
        self.get_logger().info('Torque Publisher Node started. Use rqt_reconfigure or parameters to change torque.')

    def control_loop(self):
    # Get parameters
        mode = self.get_parameter('mode').get_parameter_value().string_value
        #mode = "wind"
        
        # Use Force (Newtons) for a Prismatic Joint
        # If cable_mass is 1kg and pulley is 0.1kg, total weight is ~10.8N
        total_mass = 1.1 
        gravity = 9.81
        holding_force = total_mass * gravity # Approx 10.8 N
        
        holding_force = 9.8 

        if mode == 'wind':
            cmd_force = 20.0  # Moves it up
        elif mode == 'release':
            cmd_force = 5.0   # Lets it fall slowly
        else: 
            cmd_force = holding_force # Stays perfectly still
        self.get_logger().info(f'Mode: {mode} | Force: {cmd_force:.2f} N')

        msg = Float64MultiArray()
        msg.data = [float(cmd_force)]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PulleyTorquePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()