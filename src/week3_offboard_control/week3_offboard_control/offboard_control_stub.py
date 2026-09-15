#!/usr/bin/env python3
"""Starter scaffold — week 3 onboarding exercise.

Fill in the TODOs below to complete tasks 1-3 from
onboarding/week-3-offboard-control.md, in order:
  TODO(task 1): arm the vehicle and command takeoff via offboard setpoints
  TODO(task 2): fly to a single target waypoint and hold
  TODO(task 3): extend to a short sequence of waypoints, then land

All the publishers/subscribers and message plumbing are already wired up —
you're filling in the control logic in on_timer() and its helpers. Once this
works, compare it against offboard_control_solution.py.

(Task 4 needs no code change: once this is flying, just Ctrl-C the node
mid-flight — that's "breaking the setpoint stream" — and watch PX4's
failsafe behavior in the PX4 console / QGroundControl.)
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

# TODO(task 2): start with a single target here, e.g. WAYPOINTS = [(0.0, 0.0, -5.0)]
# TODO(task 3): extend this into a short multi-waypoint path
WAYPOINTS = []

ACCEPTANCE_RADIUS = 0.5  # metres — how close counts as "reached" a waypoint
ARM_AFTER_TICKS = 10     # send setpoints this many ticks before arming/switching to offboard
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_stub')

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

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.on_status, qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_local_position, qos_profile)

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
        # TODO(task 1): every tick, publish OffboardControlMode and a
        # TrajectorySetpoint (self.publish_offboard_control_mode() and
        # self.publish_trajectory_setpoint(...) are already written for you
        # below — you just need to call them, and pick what target to send).
        #
        # TODO(task 1): once you've been streaming setpoints for a while
        # (see ARM_AFTER_TICKS), call self.arm() and self.engage_offboard_mode().
        # PX4 will reject the mode switch if you request it before it's seen
        # a steady stream of setpoints.
        #
        # TODO(task 2): make the target you publish WAYPOINTS[0], and just
        # hold there once reached.
        #
        # TODO(task 3): once self.current_position is available, check the
        # distance to the current target (see ACCEPTANCE_RADIUS) and advance
        # self.waypoint_index when reached. Call self.land() after the last one.
        pass

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
