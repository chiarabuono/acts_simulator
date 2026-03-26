import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import numpy as np

class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller')
        self.pub_motors = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_hide = self.create_publisher(String, '/hide_link', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pulley_x', 0.0),
                ('pulley_y', 0.0),
                ('pulley_z', 0.0),
                ('available_cable_len', 0.0),
                ('current_x', 0.0),
                ('current_y', 0.0),
                ('current_z', 0.0),
            ]
        )

        # --- pulley position
        self.target_x = self.get_parameter('pulley_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('pulley_y').get_parameter_value().double_value
        self.target_z_world = self.get_parameter('pulley_z').get_parameter_value().double_value
        
        self.cable_max_l = self.get_parameter('available_cable_len').get_parameter_value().double_value
        
        # --- Initial state
        self.curr_x = self.get_parameter('current_x').get_parameter_value().double_value
        self.curr_y = self.get_parameter('current_y').get_parameter_value().double_value
        self.base_z_offset = self.get_parameter('current_z').get_parameter_value().double_value
        self.curr_visible_l = self.cable_max_l # all the cable is available outside the pulley


        self.move_speed = 0.05  # meters per steps (50cm/s at 10Hz)
        self.winch_speed = 0.03
        self.phase = "APPROACH" # phases: "APPROACH" or "WINCH"

        self.timer = self.create_timer(0.1, self.control_loop)

        self.hidden_segments = set()

    def control_loop(self):
        msg = Float64MultiArray()

        if self.phase == "APPROACH":
            dx = self.target_x - self.curr_x
            dy = self.target_y - self.curr_y
            dist = np.sqrt(dx**2 + dy**2)

            if dist < 0.01:
                self.get_logger().info("Cable at the pulley")
                self.phase = "WINCH"
            else:
                self.curr_x += (dx / dist) * min(self.move_speed, dist)
                self.curr_y += (dy / dist) * min(self.move_speed, dist)

        elif self.phase == "WINCH":
            if self.curr_visible_l > 1.5:
                self.curr_visible_l -= self.winch_speed
            
            num_segments = 20
            seg_len = 10.0 / num_segments
            
            for i in range(1, num_segments + 1):
                if (i * seg_len) > self.curr_visible_l:
                    link_name = f"link_{i}"
                    virt_name = f"virtual_{i}"
                    
                    if link_name not in self.hidden_segments:
                        msg_hide = String()
                        msg_hide.data = link_name
                        #self.pub_hide.publish(msg_hide)
                        
                        msg_hide.data = virt_name
                        #self.pub_hide.publish(msg_hide)
                        
                        self.hidden_segments.add(link_name)

        z_winch_cmd = (self.target_z_world - self.base_z_offset) + (self.cable_max_l - self.curr_visible_l)

        msg.data = [float(self.curr_x), float(self.curr_y), float(z_winch_cmd)]
        self.pub_motors.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CableController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()