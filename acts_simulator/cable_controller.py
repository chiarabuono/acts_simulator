import rclpy
from std_msgs.msg import Float64MultiArray
import time

def main():
    rclpy.init()
    node = rclpy.create_node('pulley_tester_xyz')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
    
    pub = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
    
    msg = Float64MultiArray()
    msg.data = [2.0, 3.0, 4.0]                  # [X, Y, Z]
    #print(f"Sending target XYZ: {msg.data}")
    
    
    try:
        while rclpy.ok():
            pub.publish(msg)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()