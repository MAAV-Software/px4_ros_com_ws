#!/usr/bin/env python3
"""Milestone 1 single-vehicle offboard control, based on the week 3 onboarding
reference solution.

Minimal PX4 offboard control: arms, takes off, flies the waypoint path from
the YAML given by the waypoints_file parameter, then lands.

Usage:
  ros2 run milestone1_singlevehicle offboard_solution --ros-args \
    -p waypoints_file:=/absolute/path/to/waypoints.yaml
or via the provided launch file:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py

Still the simplified onboarding controller (single vehicle, no namespacing,
no failure recovery) — not a production controller.
"""
import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

ACCEPTANCE_RADIUS = 0.5  # metres — how close counts as "reached" a waypoint
ARM_AFTER_TICKS = 10     # send setpoints this many ticks before arming/switching to offboard
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
CONFIRM_RETRY_TICKS = 10  # how many times to retry a command if not confirmed
CONFIRM_TIMEOUT_TICKS = 100  # how many ticks to wait for a command to be confirmed


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_solution')
        self.declare_parameter("waypoints_file", "")
        waypoints_file = self.get_parameter("waypoints_file").value

        if not waypoints_file:
            raise ValueError("Provide the waypoints_file ROS parameter")

        with open(waypoints_file, "r") as file:
            data = yaml.safe_load(file)

        # YAML yaw is already in radians, matching PX4.
        self.waypoints = [
            {
                'pos': tuple(float(v) for v in waypoint['pos']),
                'yaw': float(waypoint['yaw']),
                'hold_s': float(waypoint['hold_s']),
            }
            for waypoint in data['waypoints']
        ]

        if not self.waypoints:
            raise ValueError('The path must contain at least one waypoint')
        for waypoint in self.waypoints:
            if (
                len(waypoint['pos']) != 3
                or not all(math.isfinite(v) for v in waypoint['pos'])
                or not math.isfinite(waypoint['yaw'])
                or not math.isfinite(waypoint['hold_s'])
                or waypoint['hold_s'] < 0
            ):
                raise ValueError('Invalid waypoint position, yaw, or hold_s')

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

        # This PX4 build republishes VehicleLocalPosition under a versioned
        # topic name via its translation_node rather than the bare one (check
        # `ros2 topic list | grep vehicle_local_position` if this ever stops
        # matching after a PX4 update).
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_local_position, qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.on_status, qos_profile)

        self.get_logger().info(f'loaded {len(self.waypoints)} waypoints from {waypoints_file}')

        self.tick = 0
        self.waypoint_index = 0
        self.current_position = None
        self.landed = False
        self.status = None
        self.confirmed_takeoff = False
        # No hold begins until both position and heading are within tolerance.
        self.hold_started_ns = None

        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz — PX4 expects at least 2 Hz
        

    def on_local_position(self, msg):
        self.current_position = msg

    def on_status(self, msg):
        self.status = msg
        # if not self.confirmed_takeoff and msg.nav_state == VehicleStatus.NAV_STATE_AUTO_TAKEOFF:
        #     self.confirmed_takeoff = True

    def armed_in_offboard(self):
        status = self.status
        return (
            status is not None
            and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and status.nav_state == VehicleStatus.NAV_STATE_OFFBOARD
        )

    def on_timer(self):
        if self.landed:
            return

        # PX4 requires a steady offboard_control_mode + trajectory_setpoint
        # stream before it will accept (and while it will stay in) offboard
        # mode — this is why the logic lives in a timer callback, not a
        # one-shot function.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.current_target())

        if self.tick >= ARM_AFTER_TICKS and not self.confirmed_takeoff:
            waited = self.tick - ARM_AFTER_TICKS
            if self.armed_in_offboard():
                self.confirmed_takeoff = True
                self.get_logger().info('armed and in offboard mode confirmed')
            elif waited >= CONFIRM_TIMEOUT_TICKS:
                self.get_logger().error('failed to confirm takeoff')
                self.landed = True
                return
            elif waited % CONFIRM_RETRY_TICKS == 0:
                self.arm()
                self.engage_offboard_mode()

        self.tick += 1
        if self.confirmed_takeoff and self.current_position is not None:
            self.advance_waypoint_if_reached()


    def current_target(self):
        return self.waypoints[self.waypoint_index]

    def advance_waypoint_if_reached(self):
        target = self.current_target()
        position = self.current_position
        if position is None or not position.xy_valid or not position.z_valid:
            self.hold_started_ns = None
            return

        tx, ty, tz = target['pos']
        distance = math.sqrt(
            (position.x - tx) ** 2
            + (position.y - ty) ** 2
            + (position.z - tz) ** 2
        )
        # Wrap across +/-180 degrees: 179 to -179 is a 2-degree error.
        yaw_difference = target['yaw'] - position.heading
        heading_error = abs(math.atan2(
            math.sin(yaw_difference), math.cos(yaw_difference)))
        reached = (
            math.isfinite(distance)
            and math.isfinite(heading_error)
            and distance <= ACCEPTANCE_RADIUS
            and heading_error <= math.radians(5.0)
        )
        if not reached:
            # The hold must be continuous; leaving either tolerance resets it.
            self.hold_started_ns = None
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.hold_started_ns is None or now_ns < self.hold_started_ns:
            self.hold_started_ns = now_ns
            self.get_logger().info(
                f'Waypoint {self.waypoint_index} reached; '
                f'holding for {target["hold_s"]} seconds')

        # on_timer continues sending this position and yaw during the hold.
        elapsed_s = (now_ns - self.hold_started_ns) / 1e9
        if elapsed_s < target['hold_s']:
            return

        self.hold_started_ns = None
        if self.waypoint_index < len(self.waypoints) - 1:
            self.waypoint_index += 1
            self.get_logger().info(
                f'advancing to waypoint {self.waypoint_index}')
        else:
            # Finish the last hold before requesting landing.
            self.get_logger().info('Final hold complete; landing')
            self.land()
            self.landed = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, target):
        msg = TrajectorySetpoint()
        msg.position = list(target['pos'])
        # Send the yaw in radians directly from the YAML.
        msg.yaw = target['yaw']
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
