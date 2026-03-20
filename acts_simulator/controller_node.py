#!/usr/bin/env python3

"""
This files has been modified starting from:
 * File: controller_node.cpp
 * Project: Quadrotor Control Lab
 * File Created: Tuesday, 25th November 2025 11:57:25 AM
 * Author: nknab
 * Email: kojo.anyinam-boateng@ls2n.fr
 * Version: 1.0.0
 * Brief: Main entry point for the quadrotor controller node.
 * -----
 * Last Modified: Tuesday, 25th November 2025 11:57:25 AM
 * Modified By: nknab
 * -----
 * Copyright ©2025 nknab
"""

import rclpy
from rclpy.node import Node

from acts_simulator.controller import ControllerNode

def main(args=None):
    rclpy.init(args=args)

    try:
        controller_node = ControllerNode()
        rclpy.spin(controller_node)

    except KeyboardInterrupt:
        pass
    finally:
        if 'controller_node' in locals():
            controller_node.destroy_node()
        
        # Shutdown ROS2
        rclpy.shutdown()

if __name__ == '__main__':
    main()