"""The real flight backend: `ros2 launch` for the mission, ROS 2 topics for PX4 state.

px4_msgs has to match the firmware for vehicle_status, failsafe_flags and vehicle_gnss to
arrive (see flight_state.py for what still works when it doesn't).
"""
import collections
import threading
import time

import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from milestone1_singlevehicle.preflight import (
    DEFAULT_MAX_START_OFFSET_M, arm_blocker, position_problem)
from px4_msgs.msg import (
    BatteryStatus,
    FailsafeFlags,
    VehicleCommand,
    VehicleCommandAck,
    VehicleGnss,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from swarm_launcher.flight_state import (
    ACTIVITY_WHEN_COMMANDED, AGENT_COMPONENT, ARMING_STATE_ARMED, COMMANDS, NAVIGATION_STATE_OFFBOARD,
    VEHICLE_CMD_COMPONENT_ARM_DISARM, VEHICLE_CMD_RESULT_ACCEPTED, FlightTracker, clean_log_line,
    launch_argv, log_level, versioned_topic)
from swarm_launcher.grid import local_to_global
from swarm_launcher.launch_process import LaunchProcess

POSITION_MAX_AGE_S = 2.0


class RosFlight:
    def __init__(self, drone_id, topic_prefix='', system_id=1, now=time.monotonic):
        self.topic_prefix = topic_prefix
        self.system_id = system_id
        self.now = now
        self.tracker = FlightTracker(now)
        self.logs = collections.deque(maxlen=2000)
        self.process = None
        self.commanded = None
        self.accepted_commands = set()
        self.position = None
        self.position_at = None
        self.battery_pct = None
        self.gps_fix = None
        self.rc_lost = None
        self.preflight_ok = None
        self.gcs_lost = None

        # Not rclpy's own SIGINT/SIGTERM handlers: they shut ROS down but leave the process
        # running, so a stopped agent lived on, still connected under its drone's name.
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except ImportError:
            rclpy.init()
        self.node = Node(f'swarm_agent_{drone_id}')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        prefix = topic_prefix
        self.command_pub = self.node.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command', qos)
        for name, message_type, callback in (
                ('vehicle_local_position', VehicleLocalPosition, self._on_position),
                ('vehicle_command_ack', VehicleCommandAck, self._on_ack),
                ('vehicle_land_detected', VehicleLandDetected, self._on_land_detected),
                ('battery_status', BatteryStatus, self._on_battery),
                ('vehicle_status', VehicleStatus, self._on_status),
                ('failsafe_flags', FailsafeFlags, self._on_failsafe_flags),
                ('vehicle_gnss', VehicleGnss, self._on_gnss)):
            self.node.create_subscription(
                message_type, versioned_topic(prefix, name, message_type), callback, qos)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    # --- PX4 topics -------------------------------------------------------------

    def _on_position(self, msg):
        self.position = msg
        self.position_at = self.now()
        self.tracker.on_position(msg.z, msg.vz, msg.z_valid)

    def _on_ack(self, msg):
        if msg.result != VEHICLE_CMD_RESULT_ACCEPTED:
            return
        # The node's arm command. The agent's own use of this command is to disarm.
        if msg.command == VEHICLE_CMD_COMPONENT_ARM_DISARM and msg.target_component != AGENT_COMPONENT:
            self.tracker.on_arm_ack()
        if msg.target_component == AGENT_COMPONENT:
            for name, (command, _, _, _) in COMMANDS.items():
                if command == msg.command:
                    self.accepted_commands.add(name)

    def _on_land_detected(self, msg):
        self.tracker.on_land_detected(msg.landed)

    def _on_battery(self, msg):
        self.battery_pct = round(msg.remaining * 100) if msg.remaining >= 0 else None

    def _on_status(self, msg):
        self.tracker.on_status(msg.arming_state == ARMING_STATE_ARMED, msg.nav_state)
        self.preflight_ok = msg.pre_flight_checks_pass
        self.gcs_lost = msg.gcs_connection_lost

    def _on_failsafe_flags(self, msg):
        self.rc_lost = msg.manual_control_signal_lost

    def _on_gnss(self, msg):
        self.gps_fix = msg.receiver.fix_type

    def _fresh_position(self):
        if self.position_at is None or self.now() - self.position_at > POSITION_MAX_AGE_S:
            return None
        return self.position

    # --- Flight interface -------------------------------------------------------

    def problems(self, launch):
        position = self._fresh_position()
        problem = position_problem(position)
        if problem:
            return [problem]
        # PX4 would refuse to arm, but only 30 s after the node starts asking. This flag judges
        # the current mode, and Offboard always fails with nobody streaming setpoints yet, so
        # in that mode it says nothing about the drone. The agent leaves Offboard after a run.
        if (self.tracker.status_fresh and not self.preflight_ok
                and self.tracker.nav_state != NAVIGATION_STATE_OFFBOARD):
            reason = ': no ground station connected' if self.gcs_lost else ''
            return [f'PX4 preflight checks are failing{reason}. See QGroundControl']
        # The node makes the same check, but only once it is trying to arm.
        args = launch.get('effective', {})
        path = args.get('path')
        if not path or args.get('relative_start') == 'true':
            return []
        try:
            with open(path) as handle:
                waypoints = yaml.safe_load(handle)
            max_offset = float(args.get('max_start_offset_m', DEFAULT_MAX_START_OFFSET_M))
        except (OSError, ValueError, yaml.YAMLError) as error:
            return [f'cannot read the waypoints file: {error}']
        try:
            first_waypoint = waypoints['waypoints'][0]['pos']
        except (KeyError, IndexError, TypeError):
            return []  # Some other kind of path file. The launch knows what to do with it.
        blocker = arm_blocker(position, first_waypoint, max_offset)
        return [blocker] if blocker else []

    def start(self, launch):
        self.tracker.begin_run()
        self.commanded = None
        self.accepted_commands = set()
        self.process = LaunchProcess(
            launch_argv(launch, self.topic_prefix, self.system_id), now=self.now)
        self.process.start()

    def stop(self):
        if self.process:
            self.process.stop()

    def launch_exit_code(self):
        """None while the launch runs, or if none has been started."""
        return self.process.returncode if self.process else None

    def command(self, name):
        command, param1, param2, param3 = COMMANDS[name]
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = self.system_id
        msg.source_component = AGENT_COMPONENT
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
        if name in ACTIVITY_WHEN_COMMANDED:
            self.commanded = name

    def accepted(self, name):
        return name in self.accepted_commands

    def poll(self):
        if not self.process:
            return
        self.process.poll()
        for raw in self.process.drain_lines():
            line = clean_log_line(raw)
            self.tracker.on_log(line)
            self.logs.append((log_level(line), line))

    def snapshot(self):
        position = self._fresh_position()
        running = self.process.running() if self.process else False
        return {
            'position_ok': not position_problem(position),
            'position': None if position is None else {
                'x': round(float(position.x), 1),
                'y': round(float(position.y), 1),
                'z': round(float(position.z), 1)},
            'battery_pct': self.battery_pct,
            'global_position_ok': bool(position is not None and position.xy_global),
            'global': self._global(position),
            'gps_fix': self.gps_fix,
            'rc_lost': self.rc_lost,
            **self.tracker.summary(running, self.commanded),
        }

    @staticmethod
    def _global(position):
        """Latitude and longitude from PX4's own reference, for the grid map."""
        if position is None or not (position.xy_valid and position.xy_global):
            return None
        lat, lon = local_to_global(float(position.ref_lat), float(position.ref_lon),
                                   float(position.x), float(position.y))
        return {'lat': round(lat, 8), 'lon': round(lon, 8)}

    def drain_logs(self):
        lines = []
        while self.logs:
            lines.append(self.logs.popleft())
        return lines

    def close(self):
        self.stop()
        self.executor.shutdown()
        rclpy.try_shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown()
