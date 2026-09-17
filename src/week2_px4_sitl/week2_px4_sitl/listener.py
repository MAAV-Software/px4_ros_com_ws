#!/usr/bin/env python3
"""Minimal PX4 topic subscriber — week 2 onboarding exercise.

Subscribes to /fmu/out/vehicle_local_position (published by PX4 over the
uXRCE-DDS bridge) and logs position data as it arrives.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleLocalPosition


class LocalPositionListener(Node):
    def __init__(self):
        super().__init__('local_position_listener')

        # PX4 publishes over uXRCE-DDS with best-effort, transient-local QoS —
        # a subscriber using ROS2's default QoS will never see any messages.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.on_local_position,
            qos_profile)

    def on_local_position(self, msg):
        self.get_logger().info(
            f'x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} '
            f'(xy_valid={msg.xy_valid}, z_valid={msg.z_valid})')


def main():
    rclpy.init()
    node = LocalPositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
