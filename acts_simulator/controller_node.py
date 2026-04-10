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

from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()