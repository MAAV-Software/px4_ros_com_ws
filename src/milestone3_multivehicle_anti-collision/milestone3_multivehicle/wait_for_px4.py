#!/usr/bin/env python3
"""Block until PX4 is ready to be flown, then exit 0 (exit 1 on timeout).

Used by the launch file to start the offboard node only once PX4 is publishing
a valid local position estimate, instead of guessing with a fixed delay. (If
PX4 still refuses to arm at that point, the offboard node keeps retrying.)

  ros2 run milestone3_multivehicle m3_wait_ready [--timeout 180]
"""
import math
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleLocalPosition

from milestone3_multivehicle.vehicle import declare_vehicle


class WaitForPx4(Node):
    def __init__(self):
        super().__init__('m3_wait_ready')
        self.vehicle = declare_vehicle(self)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.position = None
        self.create_subscription(
            VehicleLocalPosition, self.vehicle.topic('fmu/out/vehicle_local_position_v1'),
            lambda msg: setattr(self, 'position', msg), qos)

    def missing(self):
        """What PX4 is still waiting on, or '' when it is ready."""
        if self.position is None:
            return 'no vehicle_local_position yet (is PX4 up and the XRCE agent connected?)'
        if not (self.position.xy_valid and self.position.z_valid):
            return 'no valid local position estimate yet'
        if not all(math.isfinite(v) for v in (
                self.position.x, self.position.y, self.position.z, self.position.heading)):
            return 'non-finite local position or heading'
        return ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=180.0, help='seconds to wait')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WaitForPx4()
    deadline = time.monotonic() + args.timeout
    last_report = 0.0
    reason = 'waiting'
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        reason = node.missing()
        if not reason:
            node.get_logger().info('PX4 is ready')
            node.destroy_node()
            rclpy.shutdown()
            return 0
        if time.monotonic() - last_report > 10.0:
            node.get_logger().info(f'waiting for PX4: {reason}')
            last_report = time.monotonic()
    node.get_logger().error(f'timed out after {args.timeout:.0f} s waiting for PX4: {reason}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
