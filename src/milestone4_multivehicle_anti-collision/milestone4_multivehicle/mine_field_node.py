"""Simulated mines for the field scan: hides them, reports what the drones' cameras find,
keeps planning the best path through the scanned ground and sends the last drone to check it.

One of these runs per flight (scan_launch.py starts it with the last drone). It reads
every drone's PX4 position, and all the deciding is in mines.MineSurvey.

Publishes:
- /milestone4/mines: [column, row, drone, found so far] for each mine as it is found.
- /milestone4/best_path: the checker's next route (mines.MineSurvey.message), twice a second.
The path itself, in the competition's path.txt form, is logged and written to path_file.
"""
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float64MultiArray

from milestone4_multivehicle import scan_routes
from milestone4_multivehicle.corridor import CorridorSearch
from milestone4_multivehicle.mines import MineSurvey, place_mines

MINES_TOPIC = '/milestone4/mines'
PATH_TOPIC = '/milestone4/best_path'
ROUTE_TOPIC = '/milestone4/route_{}'  # corridor search: one route topic per drone


class MineField(Node):
    def __init__(self):
        super().__init__('mine_field')
        self.declare_parameter('mine_count', 150)
        self.declare_parameter('mine_seed', 10)
        self.declare_parameter('detect_probability', 0.9)
        self.declare_parameter('vehicle_count', 4)
        self.declare_parameter('path_file', '/tmp/milestone4_path.txt')
        self.declare_parameter('search', 'mow')
        self.declare_parameter('formation', 'auto')
        count = self.get_parameter('vehicle_count').value
        self.origins = []
        for i in range(count):
            east, north = scan_routes.SPAWNS[i]
            self.declare_parameter(f'spawn_x_{i + 1}', east)
            self.declare_parameter(f'spawn_y_{i + 1}', north)
            self.origins.append((self.get_parameter(f'spawn_x_{i + 1}').value,
                                 self.get_parameter(f'spawn_y_{i + 1}').value))
        seed = self.get_parameter('mine_seed').value
        mines = place_mines(self.get_parameter('mine_count').value, seed)
        probability, log = self.get_parameter('detect_probability').value, self.get_logger().info
        self.corridor = self.get_parameter('search').value == 'corridor'
        if self.corridor:
            self.survey = CorridorSearch(mines, probability, seed, count,
                                         self.get_parameter('formation').value, log=log)
        else:
            self.survey = MineSurvey(mines, probability, seed, count, log=log)
        self.written = None
        self.positions = {}
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.subscriptions_ = [
            self.create_subscription(
                VehicleLocalPosition, f"{'' if i == 0 else f'/px4_{i}'}/fmu/out/vehicle_local_position_v1",
                lambda msg, i=i: self.positions.__setitem__(i, msg), px4_qos)
            for i in range(count)]
        self.mines_pub = self.create_publisher(Float64MultiArray, MINES_TOPIC, 50)
        self.path_pub = self.create_publisher(Float64MultiArray, PATH_TOPIC, 10)
        self.route_pubs = [self.create_publisher(Float64MultiArray, ROUTE_TOPIC.format(i), 10)
                           for i in range(count)] if self.corridor else []
        self.get_logger().info(
            f'{len(mines)} mines hidden (seed {seed}); each look finds one with probability '
            f"{self.get_parameter('detect_probability').value:g}")
        self.create_timer(0.1, self.look)
        self.create_timer(0.5, self.publish_path)

    def look(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        for drone, msg in sorted(self.positions.items()):
            if not (msg.xy_valid and msg.z_valid and all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.heading))):
                continue
            east, north = self.origins[drone][0] + msg.y, self.origins[drone][1] + msg.x
            for col, row in self.survey.observe(drone, east, north, -msg.z, msg.heading, now_s):
                found = len(self.survey.detector.found)
                self.get_logger().info(f'Mine found at column {col}, row {row} by drone {drone + 1} ({found} so far)')
                self.mines_pub.publish(Float64MultiArray(data=[float(col), float(row), float(drone), float(found)]))
        self.write_path()

    def write_path(self):
        plan, path_file = self.survey.plan, self.get_parameter('path_file').value
        if plan is None or not plan.complete or not path_file or self.written == plan.text():
            return
        self.written = plan.text()
        try:
            with open(os.path.expanduser(path_file), 'w') as stream:
                stream.write(self.written)
        except OSError as error:
            self.get_logger().warn(f'could not write {path_file}: {error}')

    def publish_path(self):
        if self.corridor:
            for drone, pub in enumerate(self.route_pubs):
                pub.publish(Float64MultiArray(data=self.survey.message(drone)))
        else:
            self.path_pub.publish(Float64MultiArray(data=self.survey.message()))


def main():
    rclpy.init()
    node = MineField()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
