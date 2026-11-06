#!/usr/bin/env python3
"""Block until PX4 is ready to be flown, then exit 0 (exit 1 on timeout).

Used by the launch file to start the offboard node only once PX4 is publishing
a valid local position estimate, instead of guessing with a fixed delay. (If
PX4 still refuses to arm at that point, the offboard node keeps retrying.)

  ros2 run milestone1_singlevehicle m1_wait_ready [--timeout 180] [--topic-prefix /px4_1]
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleLocalPosition

from milestone1_singlevehicle.preflight import position_problem


class WaitForPx4(Node):
    def __init__(self, prefix=''):
        super().__init__('m1_wait_ready')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.position = None
        self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position_v1',
            lambda msg: setattr(self, 'position', msg), qos)

    def missing(self):
        """What PX4 is still waiting on, or '' when it is ready."""
        return position_problem(self.position)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=180.0, help='seconds to wait')
    parser.add_argument('--topic-prefix', default='',
                        help="PX4 topic prefix, e.g. /px4_1 for /px4_1/fmu/... (default: none)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = WaitForPx4(args.topic_prefix.rstrip('/'))
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
