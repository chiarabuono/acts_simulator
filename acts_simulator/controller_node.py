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

from acts_simulator.position_controller import PositionControllerNode
from acts_simulator.force_controller import ForceControllerNode

from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    param_node = rclpy.create_node('param_reader')
    param_node.declare_parameter('control_mode', 'position')  # default
    control_mode = param_node.get_parameter('control_mode').value
    param_node.destroy_node()

    if control_mode == 'force':
        node = ForceControllerNode()
    else:
        node = PositionControllerNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()