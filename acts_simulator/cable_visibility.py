import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Useremo un semplice topic per i test rapidi

class CableVisibilityServer(Node):
    def __init__(self):
        super().__init__('cable_visibility_server')
        # Ascolta su un topic per nascondere i link (comodo per test da terminale)
        self.subscription = self.create_subscription(String, '/hide_link', self.listener_callback, 10)
        self.get_logger().info("Visibility Server ONLINE. Topic: /hide_link")

    def listener_callback(self, msg):
        link_name = msg.data
        self.get_logger().info(f"Ricevuto comando: nascondere {link_name}")
        self.apply_transparency(link_name)

    # def apply_transparency(self, link_name):
    #     # 1. TENTATIVO MARKER (Forza la trasparenza sull'entità)
    #     marker_cmd = [
    #         'gz', 'marker', '-m', 
    #         f'name: "{link_name}", parent: "cable", action: ADD, type: NONE, material: {{ diffuse: {{ a: 0.0 }}, ambient: {{ a: 0.0 }} }}'
    #     ]
        
    #     # 2. TENTATIVO SERVICE (Metodo standard)
    #     service_cmd = [
    #         'gz', 'service', '-s', '/world/empty/visual_config',
    #         '--reqtype', 'gz.msgs.Visual', '--reptype', 'gz.msgs.Boolean', '--timeout', '500',
    #         '--req', f'name: "visual", parent_name: "{link_name}", transparency: 1.0'
    #     ]

    #     try:
    #         subprocess.run(marker_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #         subprocess.run(service_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    #     except Exception as e:
    #         self.get_logger().error(f"Errore: {e}")

    
    def apply_transparency(self, link_name):
        model_name = "cable"
        # Il nome 'lumped' scoperto dall'Entity Tree
        target_visual = f"{link_name}_fixed_joint_lump__visual_visual"
        parent_path = f"{model_name}::{link_name}"
        
        self.get_logger().info(f"Tentativo di nascondere: {target_visual} sotto {parent_path}")

        cmd = [
            'gz', 'service', '-s', '/world/empty/visual_config',
            '--reqtype', 'gz.msgs.Visual',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', f'name: "{target_visual}", parent_name: "{parent_path}", transparency: 1.0'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            # if result.returncode == 0:
            #     self.get_logger().info(f"SUCCESSO: {link_name} rimosso dalla vista.")
            # else:
            #     self.get_logger().error(f"FALLITO: Gazebo ha risposto con errore.")
        except Exception as e:
            self.get_logger().error(f"Errore esecuzione comando: {e}")

            

def main():
    rclpy.init()
    node = CableVisibilityServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()