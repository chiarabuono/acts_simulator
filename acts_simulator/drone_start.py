import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators

class DroneJumpstart(Node):
    def __init__(self):
        super().__init__('drone_jumpstart')
        self.publisher_ = self.create_publisher(Actuators, '/drone1_/command/motor_speed', 10)
        self.declare_parameter("action_time", 0.0)
        
        self.timer = self.create_timer(0.1, self.takeoff_kick)
        self.start_time = self.get_clock().now()
        self.time = self.get_parameter("action_time").value * 1e9
        self.velocity = 2000.0

    def takeoff_kick(self):
        now = self.get_clock().now()
        elapsed = now - self.start_time
        
        if elapsed.nanoseconds < self.time:  
            msg = Actuators()
            msg.velocity = [self.velocity, self.velocity, self.velocity, self.velocity]
            self.publisher_.publish(msg)
            self.get_logger().info('Applying RAW MOTOR thrust for takeoff...', once=True)
        else:
            self.get_logger().info('Takeoff kick finished.')
            self.timer.cancel()
            raise SystemExit 

def main():
    rclpy.init()
    node = DroneJumpstart()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()