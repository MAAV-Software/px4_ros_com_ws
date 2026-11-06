"""Runs RosFlight against stand-ins for rclpy and px4_msgs.

This catches wiring mistakes in flight_ros.py on a machine with no ROS. It does not
show that PX4 accepts these commands or that the topics carry data: only a SITL run
can do that.
"""
import importlib
import sys
import types
from types import SimpleNamespace

import pytest

from swarm_launcher import flight_state


class StubMessage:
    def __init__(self):
        self.param3 = None


class StubPublisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class StubNode:
    def __init__(self, name):
        self.name = name
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, kind, topic, qos):
        self.publishers[topic] = StubPublisher()
        return self.publishers[topic]

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_000_000))

    def destroy_node(self):
        pass


class StubExecutor:
    def add_node(self, node):
        pass

    def spin(self):
        pass

    def shutdown(self):
        pass


@pytest.fixture
def flight_ros(monkeypatch, tmp_path):
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda: None
    rclpy.try_shutdown = lambda: None
    node = types.ModuleType('rclpy.node')
    node.Node = StubNode
    executors = types.ModuleType('rclpy.executors')
    executors.SingleThreadedExecutor = StubExecutor
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.QoSReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    qos.QoSDurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
    qos.QoSHistoryPolicy = SimpleNamespace(KEEP_LAST=1)
    msgs_root = types.ModuleType('px4_msgs')
    msgs = types.ModuleType('px4_msgs.msg')
    versions = {'BatteryStatus': 1, 'FailsafeFlags': 0, 'VehicleCommand': 0, 'VehicleCommandAck': 1,
                'VehicleGnss': 0, 'VehicleLandDetected': 0, 'VehicleLocalPosition': 1, 'VehicleStatus': 4}
    for name, version in versions.items():
        setattr(msgs, name, type(name, (StubMessage,), {'MESSAGE_VERSION': version}))
    ament_root = types.ModuleType('ament_index_python')
    ament = types.ModuleType('ament_index_python.packages')
    ament.get_package_share_directory = lambda package: str(tmp_path / package)
    for name, module in [('rclpy', rclpy), ('rclpy.node', node), ('rclpy.executors', executors),
                         ('rclpy.qos', qos), ('px4_msgs', msgs_root), ('px4_msgs.msg', msgs),
                         ('ament_index_python', ament_root), ('ament_index_python.packages', ament)]:
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, 'swarm_launcher.flight_ros', raising=False)
    module = importlib.import_module('swarm_launcher.flight_ros')
    yield module
    sys.modules.pop('swarm_launcher.flight_ros', None)


@pytest.fixture
def flight(flight_ros, clock):
    return flight_ros.RosFlight('d1', '/px4_1', system_id=2, now=clock)


def position(x=0.0, y=0.0, z=0.0, vz=0.0, xy_valid=True, z_valid=True, xy_global=True):
    return SimpleNamespace(x=x, y=y, z=z, vz=vz, xy_valid=xy_valid, z_valid=z_valid, xy_global=xy_global)


def sub(flight, name):
    return flight.node.subscriptions[f'/px4_1/fmu/out/{name}']


def test_topics_carry_the_prefix_and_the_known_good_versions(flight):
    assert set(flight.node.subscriptions) == {
        '/px4_1/fmu/out/vehicle_local_position_v1',
        '/px4_1/fmu/out/vehicle_command_ack_v1',
        '/px4_1/fmu/out/vehicle_land_detected',
        '/px4_1/fmu/out/battery_status_v1',
        '/px4_1/fmu/out/vehicle_status_v4',
        '/px4_1/fmu/out/failsafe_flags',
        '/px4_1/fmu/out/vehicle_gnss',
    }
    assert set(flight.node.publishers) == {'/px4_1/fmu/in/vehicle_command'}


def test_hold_command_fields(flight):
    flight.command('hold')
    message = flight.command_pub.published[-1]
    assert message.command == flight_state.VEHICLE_CMD_DO_SET_MODE
    assert (message.param1, message.param2, message.param3) == (1.0, 4.0, 3.0)
    assert message.target_system == 2
    assert message.source_component == flight_state.AGENT_COMPONENT
    assert message.from_external is True
    assert message.timestamp == 5000


def test_only_an_ack_addressed_to_the_agent_confirms_a_command(flight):
    on_ack = sub(flight, 'vehicle_command_ack_v1')
    hold = flight_state.VEHICLE_CMD_DO_SET_MODE
    on_ack(SimpleNamespace(command=hold, result=0, target_component=1))  # The offboard node's.
    assert not flight.accepted('hold')
    on_ack(SimpleNamespace(command=hold, result=4, target_component=flight_state.AGENT_COMPONENT))
    assert not flight.accepted('hold')
    on_ack(SimpleNamespace(command=hold, result=0, target_component=flight_state.AGENT_COMPONENT))
    assert flight.accepted('hold')


def test_the_agents_own_disarm_ack_does_not_mark_the_drone_armed(flight):
    on_ack = sub(flight, 'vehicle_command_ack_v1')
    on_ack(SimpleNamespace(command=flight_state.VEHICLE_CMD_COMPONENT_ARM_DISARM, result=0,
                           target_component=flight_state.AGENT_COMPONENT))
    assert not flight.snapshot()['armed']


def test_the_nodes_arm_ack_marks_the_drone_armed(flight):
    on_ack = sub(flight, 'vehicle_command_ack_v1')
    on_ack(SimpleNamespace(command=flight_state.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                           result=0, target_component=1))
    assert flight.snapshot()['armed']


def test_stale_position_blocks_the_start(flight, clock):
    launch = {'args': {}, 'waypoints': None}
    assert 'no vehicle_local_position' in flight.problems(launch)[0]
    sub(flight, 'vehicle_local_position_v1')(position())
    assert flight.problems(launch) == []
    clock.advance(10)
    assert 'no vehicle_local_position' in flight.problems(launch)[0]


def test_preflight_reuses_arm_blocker_with_the_first_waypoint(flight, tmp_path):
    folder = tmp_path / 'wp' / 'resource'
    folder.mkdir(parents=True)
    (folder / 'x.yaml').write_text('waypoints:\n  - {pos: [0.0, 0.0, -5.0], yaw: 0.0, hold_s: 5}\n')
    launch = {'args': {}, 'waypoints': {'package': 'wp', 'file': 'resource/x.yaml'}}
    sub(flight, 'vehicle_local_position_v1')(position(x=10.0))
    assert 'from the first waypoint' in flight.problems(launch)[0]
    launch['args'] = {'relative_start': 'true'}
    assert flight.problems(launch) == []
    launch['args'] = {'max_start_offset_m': '50'}
    assert flight.problems(launch) == []


def test_missing_waypoint_file_is_reported_not_raised(flight):
    sub(flight, 'vehicle_local_position_v1')(position())
    launch = {'args': {}, 'waypoints': {'package': 'wp', 'file': 'nope.yaml'}}
    assert 'cannot read the waypoints file' in flight.problems(launch)[0]


def status(armed=False, nav_state=4, preflight=True, gcs_lost=False):
    return SimpleNamespace(arming_state=2 if armed else 1, nav_state=nav_state,
                           pre_flight_checks_pass=preflight, gcs_connection_lost=gcs_lost)


def test_vehicle_status_reaches_the_tracker(flight):
    sub(flight, 'vehicle_status_v4')(status(armed=True, nav_state=14))
    snapshot = flight.snapshot()
    assert snapshot['armed'] is True and snapshot['nav_state'] == 14


def test_failing_px4_preflight_blocks_the_start(flight):
    launch = {'args': {}, 'waypoints': None}
    sub(flight, 'vehicle_local_position_v1')(position())
    sub(flight, 'vehicle_status_v4')(status(preflight=False, gcs_lost=True))
    assert 'no ground station connected' in flight.problems(launch)[0]
    sub(flight, 'vehicle_status_v4')(status(preflight=False))
    assert flight.problems(launch)[0] == 'PX4 preflight checks are failing. See QGroundControl'
    sub(flight, 'vehicle_status_v4')(status(preflight=True))
    assert flight.problems(launch) == []


def test_px4_preflight_is_ignored_while_left_in_offboard(flight):
    """It always fails in Offboard with no setpoints, which says nothing about the drone."""
    sub(flight, 'vehicle_local_position_v1')(position())
    sub(flight, 'vehicle_status_v4')(status(nav_state=14, preflight=False))
    assert flight.problems({'args': {}, 'waypoints': None}) == []


def test_missing_vehicle_status_does_not_block_the_start(flight):
    sub(flight, 'vehicle_local_position_v1')(position())
    assert flight.problems({'args': {}, 'waypoints': None}) == []


def test_gps_fix_and_rc_link_are_reported(flight):
    snapshot = flight.snapshot()
    assert snapshot['gps_fix'] is None and snapshot['rc_lost'] is None
    sub(flight, 'vehicle_gnss')(SimpleNamespace(receiver=SimpleNamespace(fix_type=6)))
    sub(flight, 'failsafe_flags')(SimpleNamespace(manual_control_signal_lost=True))
    snapshot = flight.snapshot()
    assert snapshot['gps_fix'] == 6 and snapshot['rc_lost'] is True


def test_snapshot_reports_battery_and_global_position(flight):
    assert flight.snapshot()['battery_pct'] is None
    assert flight.snapshot()['global_position_ok'] is False
    sub(flight, 'battery_status_v1')(SimpleNamespace(remaining=0.8))
    sub(flight, 'vehicle_local_position_v1')(position())
    snapshot = flight.snapshot()
    assert snapshot['battery_pct'] == 80 and snapshot['global_position_ok'] is True
    sub(flight, 'vehicle_local_position_v1')(position(xy_global=False))
    assert flight.snapshot()['global_position_ok'] is False
    sub(flight, 'battery_status_v1')(SimpleNamespace(remaining=-1))
    assert flight.snapshot()['battery_pct'] is None


def test_snapshot_says_whether_the_launch_is_running(flight):
    assert flight.snapshot()['launch_running'] is False


def test_land_detected_reaches_the_tracker(flight):
    sub(flight, 'vehicle_land_detected')(SimpleNamespace(landed=False))
    assert flight.snapshot()['airborne'] is True


def test_launch_argv_and_output_flow_through(flight, monkeypatch, flight_ros):
    captured = {}

    class StubProcess:
        def __init__(self, argv, now):
            captured['argv'] = argv
            self.returncode = None

        def start(self):
            pass

        def poll(self):
            pass

        def running(self):
            return True

        def stop(self):
            captured['stopped'] = True

        def drain_lines(self):
            return ['\x1b[32m[n-1] [INFO] [1.0] [x]: loaded 22 waypoints from a.yaml\x1b[0m\n']

    monkeypatch.setattr(flight_ros, 'LaunchProcess', StubProcess)
    flight.start({'package': 'p', 'file': 'f.launch.py', 'args': {}, 'waypoints': None})
    assert captured['argv'][:4] == ['ros2', 'launch', 'p', 'f.launch.py']
    assert 'topic_prefix:=/px4_1' in captured['argv']
    flight.poll()
    assert flight.snapshot()['waypoint_total'] == 22
    assert flight.drain_logs() == [('INFO', '[n-1] [INFO] [1.0] [x]: loaded 22 waypoints from a.yaml')]
    flight.stop()
    assert captured['stopped']
