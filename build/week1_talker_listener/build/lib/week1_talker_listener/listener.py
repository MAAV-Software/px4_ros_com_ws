#!/usr/bin/env python3
"""Minimal ROS2 subscriber — week 1 onboarding exercise."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, 'chatter', self.on_message, 10)

    def on_message(self, msg):
        self.get_logger().info(f'heard: "{msg.data}"')


def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
