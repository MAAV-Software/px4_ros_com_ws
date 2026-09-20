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
  ros2 run week4_multivehicle offboard_solution --ros-args -p vehicle_instance:=1
or via the provided launch file:
  ros2 launch week4_multivehicle offboard_solution.launch.py instance:=1
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

WAYPOINTS = [
    (0.0, 0.0, -5.0),
    (5.0, 0.0, -5.0),
    (5.0, 5.0, -5.0),
    (0.0, 5.0, -5.0),
]
ACCEPTANCE_RADIUS = 0.5
ARM_AFTER_TICKS = 10
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_solution')

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
                f'vehicle {self.vehicle_instance}: reached waypoint {self.waypoint_index - 1}, '
                f'advancing to {self.waypoint_index}')
        else:
            self.get_logger().info(f'vehicle {self.vehicle_instance}: reached final waypoint, landing')
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
