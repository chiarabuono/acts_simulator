import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter

class PulleyTester(Node):
    def __init__(self):
        super().__init__('pulley_tester')
        
        # Fondamentale: dice al nodo di aspettare il clock di Gazebo
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands', 
            10
        )
        
        # Timer per pubblicare costantemente (Gazebo preferisce un flusso)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo di test avviato. Inviando posizione 2.0...')

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [2.0] # Spostamento di 2 metri
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = PulleyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()