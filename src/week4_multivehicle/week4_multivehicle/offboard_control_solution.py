#!/usr/bin/env python3
"""Reference solution — week 4 onboarding exercise.

Same offboard control logic as week 3's solution (arm, takeoff, fly a short
waypoint path, land), but able to target *either* vehicle instance in a
2-instance SITL setup by topic namespace, selected via a ROS2 parameter.

PX4's SITL startup namespaces every /fmu/... topic with `px4_<instance>` for
any instance other than 0 (see ROMFS/px4fmu_common/init.d-posix/rcS in
PX4-Autopilot — instance 0 gets no prefix, instance N gets "px4_N"). That's
the exact rule `topic()` below implements.

Usage:
  ros2 run week4_multivehicle offboard_solution --ros-args -p vehicle_instance:=0 \
    -p waypoints_file:=/absolute/path/to/waypoints.yaml
or via the provided launch file:
  ros2 launch week4_multivehicle offboard_solution.launch.py instance:=1
"""
import rclpy
import math
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

ACCEPTANCE_RADIUS = 0.5
ARM_AFTER_TICKS = 10
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


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

        self.declare_parameter('vehicle_instance', 1)
        self.vehicle_instance = self.get_parameter('vehicle_instance').value
        self.get_logger().info(f'targeting vehicle instance {self.vehicle_instance}')


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, self.topic('fmu/in/offboard_control_mode'), qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self.topic('fmu/in/trajectory_setpoint'), qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, self.topic('fmu/in/vehicle_command'), qos_profile)

        # This PX4 build republishes VehicleStatus/VehicleLocalPosition under
        # versioned topic names via its translation_node rather than the bare
        # ones (check `ros2 topic list | grep vehicle_local_position` if this
        # ever stops matching after a PX4 update).
        self.status_sub = self.create_subscription(
            VehicleStatus, self.topic('fmu/out/vehicle_status_v1'), self.on_status, qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, self.topic('fmu/out/vehicle_local_position_v1'), self.on_local_position, qos_profile)

        self.tick = 0
        self.waypoint_index = 0
        self.vehicle_status = VehicleStatus()
        self.current_position = None
        self.landed = False
        # No hold begins until both position and heading are within tolerance.
        self.hold_started_ns = None

        self.timer = self.create_timer(0.1, self.on_timer)

    def topic(self, suffix):
        """Build the namespaced topic name for this vehicle instance.

        Instance 0 uses the bare topic (e.g. /fmu/out/vehicle_status_v1).
        Instance N (N != 0) is prefixed with px4_N (e.g. /px4_1/fmu/out/vehicle_status_v1),
        matching PX4 SITL's own uxrce_dds_client namespacing (rcS: `-n px4_$px4_instance`).
        """
        if self.vehicle_instance == 0:
            return f'/{suffix}'
        return f'/px4_{self.vehicle_instance}/{suffix}'

    def on_status(self, msg):
        self.vehicle_status = msg

    def on_local_position(self, msg):
        self.current_position = msg

    def on_timer(self):
        if self.landed:
            return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(self.current_target())

        if self.tick == ARM_AFTER_TICKS:
            self.arm()
            self.engage_offboard_mode()

        self.tick += 1

        if self.tick > ARM_AFTER_TICKS and self.current_position is not None:
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
                f'vehicle {self.vehicle_instance}: advancing to '
                f'waypoint {self.waypoint_index}')
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
        # PX4 SITL assigns MAV_SYS_ID = instance + 1 in its startup script.
        msg.target_system = 1 + self.vehicle_instance
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
