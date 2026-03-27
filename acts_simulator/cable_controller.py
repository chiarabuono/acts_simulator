import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import numpy as np

class CableController(Node):
    def __init__(self):
        super().__init__('cable_controller')
        self.pub_motors = self.create_publisher(Float64MultiArray, 'motor_commands', 10)
        self.pub_hide = self.create_publisher(String, '/hide_link', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('mode', 'WIND'),
                ('vel', 0.01),
                ('pulley_x', 0.0),
                ('pulley_y', 0.0),
                ('pulley_z', 0.0),
                ('total_cable_len', 5.0),
                ('unwinded_cable_len', 5.0),
                ('current_x', 0.0),
                ('current_y', 0.0),
                ('current_z', 0.0),
                ('num_segments', 20),
                ('cable_extreme', 'link_first')
            ]
        )

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.cable_extreme = self.get_parameter('cable_extreme').get_parameter_value().string_value
        self.winch_speed = self.get_parameter('vel').get_parameter_value().double_value

        if self.mode not in ["WIND", "UNWIND"]:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Use 'WIND' or 'UNWIND'.")
            raise RuntimeError("Shutting down due to invalid configuration parameter.")
        
        
        # Pulley and Cable Params
        self.target_x = self.get_parameter('pulley_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('pulley_y').get_parameter_value().double_value
        self.target_z = self.get_parameter('pulley_z').get_parameter_value().double_value
        self.cable_max_l = self.get_parameter('total_cable_len').get_parameter_value().double_value
        self.curr_visible_l = self.get_parameter('unwinded_cable_len').get_parameter_value().double_value
        self.num_segments = self.get_parameter('num_segments').get_parameter_value().integer_value
        
        # Initial State
        self.curr_x = self.get_parameter('current_x').get_parameter_value().double_value
        self.curr_y = self.get_parameter('current_y').get_parameter_value().double_value
        self.curr_z = self.get_parameter('current_z').get_parameter_value().double_value
        
        self.move_speed = 0.05 # Approach speed
        self.phase = "APPROACH" 
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Set to track what is currently hidden
        self.hidden_segments = set()

    def control_loop(self):
        msg = Float64MultiArray()

        if self.phase == "APPROACH":
            # To fully implement if want to be sure that the cable extreme is centered corrected
            self.phase = "WINCH"

        elif self.phase == "WINCH":
            if self.mode == "WIND":
                if self.curr_visible_l > 0.05:
                    self.curr_visible_l -= self.winch_speed
                else:
                    self.phase = "FINISHED"
            
            elif self.mode == "UNWIND":
                # Stop if nearly fully extended
                if self.curr_visible_l < self.cable_max_l:
                    self.curr_visible_l += self.winch_speed
                else:
                    self.phase = "FINISHED"


            seg_len = self.cable_max_l / self.num_segments
            for i in range(1, self.num_segments + 1):
                link_name = f"link_{i}"
                is_hidden = (i * seg_len) > self.curr_visible_l
                if is_hidden and link_name not in self.hidden_segments:
                    self.publish_visibility(link_name, hide=True)
                    self.hidden_segments.add(link_name)
                elif not is_hidden and link_name in self.hidden_segments:
                    self.publish_visibility(link_name, hide=False)
                    self.hidden_segments.remove(link_name)

        # amount_retracted: 0 when fully extended, total_cable_len when fully reeled in
        retraction_amount = self.cable_max_l - self.curr_visible_l
        
        if self.cable_extreme == "link_first":
            z_winch_cmd = (self.target_z - self.curr_z) + retraction_amount
        else:
            z_winch_cmd = (self.target_z - self.curr_z) - retraction_amount

        # if self.phase != "FINISHED":
        #     self.get_logger().info(f"{self.mode} | Visible: {self.curr_visible_l:.2f} | Cmd: {z_winch_cmd:.2f} | Amound rectracted: {retraction_amount}")
        # else:
        #     self.get_logger().info(f"{self.mode} | Cable max len {self.cable_max_l}", once=True)


        msg = Float64MultiArray()
        msg.data = [float(self.curr_x), float(self.curr_y), float(z_winch_cmd)]
        self.pub_motors.publish(msg)

    def publish_visibility(self, name, hide=True):
        # Placeholder for visibility plugin logic
        msg = String()
        msg.data = f"{name}:{'hide' if hide else 'show'}"
        self.pub_hide.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CableController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()