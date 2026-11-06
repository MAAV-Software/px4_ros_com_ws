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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from milestone1_singlevehicle.preflight import DEFAULT_MAX_START_OFFSET_M, arm_blocker, target_position
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
)

ACCEPTANCE_RADIUS = 0.5  # metres — how close counts as "reached" a waypoint
ARM_AFTER_TICKS = 10     # send setpoints this many ticks before arming/switching to offboard
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0
CONFIRM_RETRY_TICKS = 10     # resend arm + offboard this often until PX4 reports both
CONFIRM_TIMEOUT_TICKS = 300  # give up if arm + offboard still aren't accepted after this many ticks
# Matches the MPC_YAWRAUTO_MAX agreed for these tests. Ramping the commanded
# yaw at this rate, instead of stepping straight to the target, keeps PX4's
# attitude controller from overshooting on a large yaw change.
YAW_RATE_LIMIT_RAD_S = math.radians(45.0)


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_solution')
        self.declare_parameter("waypoints_file", "")
        waypoints_file = self.get_parameter("waypoints_file").value
        # PX4 instance N != 0 (e.g. a fleet drone) publishes under /px4_N/fmu/...;
        # leave empty for the bare /fmu/... topics used by instance 0 / SITL.
        self.declare_parameter("topic_prefix", "")
        prefix = self.get_parameter("topic_prefix").value.rstrip('/')
        # Real-drone safety: don't arm unless the drone is within this many
        # metres (x/y) of the first waypoint; negative disables the check.
        self.declare_parameter("max_start_offset_m", DEFAULT_MAX_START_OFFSET_M)
        self.max_start_offset_m = float(self.get_parameter("max_start_offset_m").value)
        # Fly the path relative to where the drone is when it arms, instead of
        # to absolute EKF-origin coordinates. Use when the estimator's origin or
        # height offset isn't trustworthy (the "-5 m" climb is then 5 m above
        # the start). Waypoint hold/landing logic is unchanged.
        self.declare_parameter("relative_start", False)
        self.relative_start = bool(self.get_parameter("relative_start").value)
        self.start_offset = None
        # Extra wait (s) before the first arm attempt, so people can stand clear.
        self.declare_parameter("arm_delay_s", 0.0)
        arm_delay_s = max(0.0, float(self.get_parameter("arm_delay_s").value))
        self.arm_after_ticks = ARM_AFTER_TICKS + int(round(arm_delay_s * 10))

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
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', qos_profile)

        # This PX4 build republishes VehicleLocalPosition under a versioned
        # topic name via its translation_node rather than the bare one (check
        # `ros2 topic list | grep vehicle_local_position` if this ever stops
        # matching after a PX4 update).
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, f'{prefix}/fmu/out/vehicle_local_position_v1', self.on_local_position, qos_profile)

        # PX4 answers every command with an ack, which is all this node needs.
        self.ack_sub = self.create_subscription(
            VehicleCommandAck, f'{prefix}/fmu/out/vehicle_command_ack_v1', self.on_ack, qos_profile)

        self.get_logger().info(f'loaded {len(self.waypoints)} waypoints from {waypoints_file}')

        self.tick = 0
        self.waypoint_index = 0
        self.current_position = None
        self.arm_accepted = False
        self.offboard_accepted = False
        self.confirmed_offboard = False
        self.landed = False
        self.blocker = ''
        # No hold begins until both position and heading are within tolerance.
        self.hold_started_ns = None
        # Commanded (ramped) yaw sent to PX4; None until the first tick sets it.
        self.commanded_yaw = None
        self.last_yaw_tick_ns = None

        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz — PX4 expects at least 2 Hz

    def on_local_position(self, msg):
        self.current_position = msg

    def on_ack(self, msg):
        accepted = msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        if msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.arm_accepted = accepted
        elif msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.offboard_accepted = accepted
        if not accepted:
            self.get_logger().warn(f'PX4 did not accept command {msg.command} (result {msg.result}); will retry')

    def on_timer(self):
        if self.landed:
            return

        # PX4 requires a steady offboard_control_mode + trajectory_setpoint
        # stream before it will accept (and while it will stay in) offboard
        # mode — this is why the logic lives in a timer callback, not a
        # one-shot function.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.current_target())

        # Keep asking until PX4 accepts both arm and offboard: a single command
        # is lost if PX4 isn't ready yet or refuses to arm.
        if self.tick >= self.arm_after_ticks and not self.confirmed_offboard:
            waited = self.tick - self.arm_after_ticks
            if self.arm_accepted and self.offboard_accepted:
                self.confirmed_offboard = True
                self.get_logger().info('arm and offboard mode accepted by PX4')
            elif waited >= CONFIRM_TIMEOUT_TICKS:
                self.get_logger().error(
                    f'not flying: no arm + offboard within {CONFIRM_TIMEOUT_TICKS / 10:.0f} s; giving up '
                    f'({self.blocker or "check PX4 preflight checks / GCS connection"})')
                if self.arm_accepted:
                    # Armed but never reached offboard: don't leave it spinning on the ground.
                    self.disarm()
                self.landed = True
                return
            elif waited % CONFIRM_RETRY_TICKS == 0:
                # In relative mode the path starts wherever the drone is, so the
                # distance-from-origin check doesn't apply (validity still does).
                max_offset = -1.0 if self.relative_start else self.max_start_offset_m
                self.blocker = arm_blocker(
                    self.current_position, self.waypoints[0]['pos'], max_offset)
                if self.blocker:
                    if waited % (5 * CONFIRM_RETRY_TICKS) == 0:
                        self.get_logger().warn(f'not arming: {self.blocker}')
                else:
                    if self.relative_start and not (self.arm_accepted or self.offboard_accepted):
                        # Re-captured on each retry until PX4 accepts, then frozen.
                        p = self.current_position
                        self.start_offset = (float(p.x), float(p.y), float(p.z))
                        self.get_logger().info(
                            f'relative start: path origin = x {p.x:.2f} y {p.y:.2f} z {p.z:.2f} (EKF frame)')
                    self.arm()
                    self.engage_offboard_mode()

        self.tick += 1

        if self.confirmed_offboard and self.current_position is not None:
            self.advance_waypoint_if_reached()

    def target_position(self, target):
        return target_position(target['pos'], self.relative_start, self.start_offset, self.current_position)

    def current_target(self):
        return self.waypoints[self.waypoint_index]

    def advance_waypoint_if_reached(self):
        target = self.current_target()
        position = self.current_position
        if position is None or not position.xy_valid or not position.z_valid:
            self.hold_started_ns = None
            return
        if self.relative_start and self.start_offset is None:
            return

        tx, ty, tz = self.target_position(target)
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
        msg.position = list(self.target_position(target))
        # Ramp toward the YAML yaw (radians) rather than stepping straight to
        # it, so PX4 doesn't overshoot on a large heading change.
        msg.yaw = self.ramped_yaw(target['yaw'])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def ramped_yaw(self, target_yaw):
        now_ns = self.get_clock().now().nanoseconds
        if self.commanded_yaw is None:
            self.commanded_yaw = target_yaw
        else:
            dt = (now_ns - self.last_yaw_tick_ns) / 1e9
            error = math.atan2(math.sin(target_yaw - self.commanded_yaw),
                                math.cos(target_yaw - self.commanded_yaw))
            step = max(-YAW_RATE_LIMIT_RAD_S * dt, min(YAW_RATE_LIMIT_RAD_S * dt, error))
            self.commanded_yaw = math.atan2(
                math.sin(self.commanded_yaw + step), math.cos(self.commanded_yaw + step))
        self.last_yaw_tick_ns = now_ns
        return self.commanded_yaw

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

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('disarm command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main():
    rclpy.init()
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: stop quietly
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
