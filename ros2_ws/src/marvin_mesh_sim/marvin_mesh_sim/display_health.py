"""Check whether a reused backend still has its robot-description publisher."""
import time

import rclpy
from rclpy.node import Node


def main():
    rclpy.init();node=Node('marvin_display_health')
    try:
        deadline=time.monotonic()+4.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if node.count_publishers('/robot_description'):
                print('present');return
        print('missing')
    finally:node.destroy_node();rclpy.shutdown()
