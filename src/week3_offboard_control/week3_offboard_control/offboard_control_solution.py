#!/usr/bin/env python3
"""Reference solution — week 3 onboarding exercise.

Minimal PX4 offboard control: arms, takes off, flies a short hardcoded
waypoint path, then lands. Read this after attempting offboard_control_stub.py
yourself, to compare against your own implementation.

This is deliberately simplified for a first offboard-control lesson (single
vehicle, no namespacing, no failure recovery) — not a production controller.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# Hardcoded waypoint path in the local NED frame (x=north, y=east, z=down —
# so negative z is up), in metres. First waypoint doubles as the takeoff target.
WAYPOINTS = [
    (0.0, 0.0, -5.0),
    (5.0, 0.0, -5.0),
    (5.0, 5.0, -5.0),
    (0.0, 5.0, -5.0),
]

# --- Extension solution: load WAYPOINTS from resource/waypoints.yaml instead ---
# of hardcoding them above. Uncomment this block (and the imports below) to
# use it; it replaces the WAYPOINTS list assigned above.
#
# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory
#
# def load_waypoints_from_yaml(package_name='week3_offboard_control', filename='waypoints.yaml'):
#     share_dir = get_package_share_directory(package_name)
#     yaml_path = os.path.join(share_dir, 'resource', filename)
#     with open(yaml_path) as f:
#         data = yaml.safe_load(f)
#     return [tuple(waypoint) for waypoint in data['waypoints']]
#
# WAYPOINTS = load_waypoints_from_yaml()

ACCEPTANCE_RADIUS = 0.5  # metres — how close counts as "reached" a waypoint
ARM_AFTER_TICKS = 10     # send setpoints this many ticks before arming/switching to offboard
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_solution')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # This PX4 build republishes VehicleStatus/VehicleLocalPosition under
        # versioned topic names via its translation_node rather than the bare
        # ones (check `ros2 topic list | grep vehicle_local_position` if this
        # ever stops matching after a PX4 update).
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_local_position, qos_profile)

        self.tick = 0
        self.waypoint_index = 0
        self.vehicle_status = VehicleStatus()
        self.current_position = None
        self.landed = False

        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz — PX4 expects at least 2 Hz

    def on_status(self, msg):
        self.vehicle_status = msg

    def on_local_position(self, msg):
        self.current_position = msg

    def on_timer(self):
        if self.landed:
            return

        # PX4 requires a steady offboard_control_mode + trajectory_setpoint
        # stream before it will accept (and while it will stay in) offboard
        # mode — this is why the logic lives in a timer callback, not a
        # one-shot function.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.current_target())

        if self.tick == ARM_AFTER_TICKS:
            self.arm()
            self.engage_offboard_mode()

        self.tick += 1

        if self.tick > ARM_AFTER_TICKS and self.current_position is not None:
            self.advance_waypoint_if_reached()

    def current_target(self):
        return WAYPOINTS[self.waypoint_index]

    def advance_waypoint_if_reached(self):
        tx, ty, tz = self.current_target()
        dx = self.current_position.x - tx
        dy = self.current_position.y - ty
        dz = self.current_position.z - tz
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

        if distance >= ACCEPTANCE_RADIUS:
            return

        if self.waypoint_index < len(WAYPOINTS) - 1:
            self.waypoint_index += 1
            self.get_logger().info(
                f'reached waypoint {self.waypoint_index - 1}, advancing to {self.waypoint_index}')
        else:
            self.get_logger().info('reached final waypoint, landing')
            self.land()
            self.landed = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, target):
        msg = TrajectorySetpoint()
        msg.position = [float(target[0]), float(target[1]), float(target[2])]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        self.get_logger().info('offboard mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main():
    rclpy.init()
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
