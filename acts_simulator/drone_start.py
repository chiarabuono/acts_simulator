import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators

class DroneJumpstart(Node):
    def __init__(self):
        super().__init__('drone_jumpstart')
        # Use the topic that you confirmed works manually
        self.publisher_ = self.create_publisher(
            Actuators, 
            '/drone1_/command/motor_speed', 
            10)
        
        self.timer = self.create_timer(0.1, self.takeoff_kick)
        self.start_time = self.get_clock().now()

    def takeoff_kick(self):
        now = self.get_clock().now()
        elapsed = now - self.start_time
        
        # Apply full thrust for 3 seconds to clear the platform
        if elapsed.nanoseconds < 3e9:  
            msg = Actuators()
            # Sending 3000.0 to all 4 motors (adjust if 3000 isn't enough to lift)
            msg.velocity = [3500.0, 3500.0, 3500.0, 3500.0]
            self.publisher_.publish(msg)
            self.get_logger().info('Applying RAW MOTOR thrust for takeoff...')
        else:
            # Stop the kick
            self.get_logger().info('Takeoff kick finished.')
            self.timer.cancel()

def main():
    rclpy.init()
    node = DroneJumpstart()
    rclpy.spin(node)
    rclpy.shutdown()