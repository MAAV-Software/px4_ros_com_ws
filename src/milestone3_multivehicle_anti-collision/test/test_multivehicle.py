"""Exercise actual controllers/launch functions with in-memory ROS interfaces.

No ROS transport, simulator, or flight commands are used by these tests.
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from milestone3_multivehicle.vehicle import resolve_vehicle, validate_pair
from milestone3_multivehicle.avoidance import POSITION_TOPIC

ROOT = Path(__file__).parents[1]
PATH = str(ROOT / 'resource/milestone3_collision_0.yaml')


@pytest.mark.parametrize('instance,prefix,system', [(0, '', 1), (1, '/px4_1', 2), (2, '/px4_2', 3)])
def test_default_routing(instance, prefix, system):
    vehicle = resolve_vehicle(instance)
    assert vehicle.topic('/fmu/in/vehicle_command') == prefix + '/fmu/in/vehicle_command'
    assert vehicle.system_id == system


def test_hardware_overrides():
    assert resolve_vehicle(1, '/airframe/b/', 42).topic('fmu/in/trajectory_setpoint') == \
        '/airframe/b/fmu/in/trajectory_setpoint'
    assert resolve_vehicle(1, '/', 7).namespace == ''
    assert resolve_vehicle(1, '/', 7).system_id == 7


@pytest.mark.parametrize('args', [(-1,), (255,), (1.5,), (True,),
                                  (0, 'bad//name'), (0, '2drone'),
                                  (0, 'drone', -1), (0, 'drone', 256)])
def test_invalid_routing(args):
    with pytest.raises(ValueError):
        resolve_vehicle(*args)


def test_duplicate_routes_rejected():
    with pytest.raises(ValueError, match='namespaces'):
        validate_pair(resolve_vehicle(0), resolve_vehicle(1, '/'))
    with pytest.raises(ValueError, match='MAV_SYS_ID'):
        validate_pair(resolve_vehicle(0), resolve_vehicle(1, 'px4_1', 1))


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.publishers = {}
        self.subscriptions = {}
        self.now_ns = 0

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message, topic, qos):
        messages = []
        publisher = SimpleNamespace(messages=messages, publish=messages.append)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def create_timer(self, period, callback):
        return SimpleNamespace(period=period, callback=callback)

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warn=lambda _: None, error=lambda _: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


class Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.__dict__.update(kwargs)


class ConfigSubstitution:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.values[self.name]


@pytest.fixture
def api(monkeypatch):
    def module(name, **attributes):
        m = ModuleType(name)
        m.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    module('rclpy')
    module('rclpy.node', Node=FakeNode)
    module('rclpy.executors', ExternalShutdownException=RuntimeError)
    policy = SimpleNamespace(BEST_EFFORT=1, TRANSIENT_LOCAL=1, VOLATILE=2, KEEP_LAST=1)
    module('rclpy.qos', QoSProfile=Action, QoSReliabilityPolicy=policy,
           QoSDurabilityPolicy=policy, QoSHistoryPolicy=policy)
    command = type('VehicleCommand', (), {'VEHICLE_CMD_COMPONENT_ARM_DISARM': 400,
                                         'VEHICLE_CMD_DO_SET_MODE': 176, 'VEHICLE_CMD_NAV_LAND': 21})
    ack = type('VehicleCommandAck', (), {'VEHICLE_CMD_RESULT_ACCEPTED': 0})
    module('std_msgs')
    module('std_msgs.msg', Float64MultiArray=SimpleNamespace)
    module('px4_msgs')
    module('px4_msgs.msg', VehicleCommand=command, VehicleCommandAck=ack,
           OffboardControlMode=SimpleNamespace, TrajectorySetpoint=SimpleNamespace,
           VehicleLocalPosition=SimpleNamespace)
    module('ament_index_python')
    module('ament_index_python.packages', get_package_share_directory=lambda _: str(ROOT),
           PackageNotFoundError=LookupError)
    module('launch', LaunchDescription=type('LaunchDescription', (Action,), {}))
    module('launch.actions', **{name: type(name, (Action,), {}) for name in (
        'DeclareLaunchArgument', 'EmitEvent', 'IncludeLaunchDescription', 'LogInfo',
        'OpaqueFunction', 'RegisterEventHandler', 'ExecuteProcess')})
    module('launch.event_handlers', OnProcessExit=type('OnProcessExit', (Action,), {}))
    module('launch.events', Shutdown=Action)
    module('launch.launch_description_sources', PythonLaunchDescriptionSource=Action)
    module('launch.substitutions', LaunchConfiguration=ConfigSubstitution, PathJoinSubstitution=Action)
    module('launch_ros')
    module('launch_ros.actions', Node=type('Node', (Action,), {}))
    module('launch_ros.substitutions', FindPackageShare=Action)
    monkeypatch.setattr(FakeNode, 'overrides', {})

    def load(relative):
        name = '_m3_test_' + relative.replace('/', '_').replace('.', '_')
        spec = importlib.util.spec_from_file_location(name, ROOT / relative)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m

    return load


def make_controller(api, instance, **overrides):
    FakeNode.overrides = {'vehicle_instance': instance, 'waypoints_file': PATH, **overrides}
    return api('milestone3_multivehicle/offboard_control_solution.py').OffboardControl()


def position(**kwargs):
    return SimpleNamespace(**{'x': 0., 'y': 0., 'z': -5., 'heading': 0.,
                              'xy_valid': True, 'z_valid': True, 'v_xy_valid': True,
                              'vx': 0., 'vy': 0., **kwargs})


def test_all_endpoints_and_commands_are_isolated(api):
    first, second = make_controller(api, 0), make_controller(api, 1)
    for node, prefix, target in [(first, '', 1), (second, '/px4_1', 2)]:
        assert set(node.publishers) - {POSITION_TOPIC} == {prefix + '/fmu/in/' + suffix for suffix in
                                       ('offboard_control_mode', 'trajectory_setpoint', 'vehicle_command')}
        assert set(node.subscriptions) - {POSITION_TOPIC} == {prefix + '/fmu/out/' + suffix for suffix in
                                          ('vehicle_local_position_v1', 'vehicle_command_ack_v1')}
        node.arm()
        node.engage_offboard_mode()
        node.land()
        commands = node.vehicle_command_pub.messages
        assert [m.command for m in commands] == [400, 176, 21]
        assert all(m.target_system == target for m in commands)
        node.landed = False
        node.on_local_position(position())
        node.on_global_position(SimpleNamespace(data=[1., 20., 0.]))
        node.on_timer()
        assert node.trajectory_setpoint_pub.messages[-1].position == [0., 0., -5.]
        assert node.offboard_control_mode_pub.messages[-1].position is True
    assert set(first.publishers) & set(second.publishers) == {POSITION_TOPIC}
    assert set(first.subscriptions) & set(second.subscriptions) == {POSITION_TOPIC}


def test_hardware_controller_routes_all_endpoints_and_commands(api):
    node = make_controller(api, 1, topic_namespace='/drone_b', target_system_id=12)
    assert all(t.startswith('/drone_b/fmu/') for t in [*node.publishers, *node.subscriptions] if t != POSITION_TOPIC)
    node.arm()
    assert node.vehicle_command_pub.messages[-1].target_system == 12


def test_ack_position_and_mission_state_do_not_cross_vehicles(api, tmp_path):
    path = tmp_path / 'mission.yaml'
    path.write_text('waypoints:\n'
                    '  - {pos: [0, 0, -5], yaw: 0, hold_s: 5}\n'
                    '  - {pos: [5, 0, -5], yaw: 0, hold_s: 5}\n'
                    '  - {pos: [2, 2, -5], yaw: 0, hold_s: 5}\n')
    first, second = make_controller(api, 0, waypoints_file=str(path)), make_controller(api, 1)
    first.subscriptions['/fmu/out/vehicle_local_position_v1'](position())
    first.subscriptions['/fmu/out/vehicle_command_ack_v1'](SimpleNamespace(command=400, result=0))
    assert first.arm_accepted and not second.arm_accepted
    assert first.current_position is not None and second.current_position is None
    first.advance_waypoint_if_reached()
    first.now_ns = 5_000_000_000
    first.advance_waypoint_if_reached()
    assert first.waypoint_index == 1 and second.waypoint_index == 0
    assert second.hold_started_ns is None
    first.waypoint_index = len(first.waypoints) - 1
    first.current_position = position(x=2., y=2.)
    first.advance_waypoint_if_reached()
    first.now_ns += 5_000_000_000
    first.advance_waypoint_if_reached()
    assert first.landed and not second.landed
    assert first.vehicle_command_pub.messages[-1].command == 21
    second.on_local_position(position())
    second.on_timer()
    assert second.trajectory_setpoint_pub.messages  # peer keeps streaming


def test_readiness_only_accepts_valid_selected_vehicle(api):
    FakeNode.overrides = {'vehicle_instance': 1}
    node = api('milestone3_multivehicle/wait_for_px4.py').WaitForPx4()
    assert list(node.subscriptions) == ['/px4_1/fmu/out/vehicle_local_position_v1']
    assert node.missing()
    callback = next(iter(node.subscriptions.values()))
    for msg in [position(xy_valid=False), position(z_valid=False), position(x=math.nan)]:
        callback(msg)
        assert node.missing()
    callback(position())
    assert node.missing() == ''


# --- collision_launch.py: one vehicle per launch ------------------------------------

COLLISION = [str(ROOT / f'resource/milestone3_collision_{i}.yaml') for i in range(2)]


def collision_context(**overrides):
    return SimpleNamespace(is_shutdown=False, values={
        'topic_prefix': '', 'system_id': '1', 'path_1': COLLISION[0], 'path_2': COLLISION[1],
        'spawn_x_1': '0.0', 'spawn_y_1': '0.0', 'spawn_x_2': '20.0', 'spawn_y_2': '0.0', **overrides})


def collision_actions(api, context):
    actions = api('launch/collision_launch.py').launch_vehicle(context)
    handlers = [a.args[0] for a in actions if type(a).__name__ == 'RegisterEventHandler']
    return actions, handlers


@pytest.mark.parametrize('prefix, system_id, instance, namespace, path, origin', [
    ('', '1', 0, '/', 0, (0.0, 0.0)),
    ('/px4_1', '2', 1, '/px4_1', 1, (20.0, 0.0)),
])
def test_each_vehicle_takes_its_own_route_and_origin(api, prefix, system_id, instance, namespace, path, origin):
    context = collision_context(topic_prefix=prefix, system_id=system_id)
    actions, handlers = collision_actions(api, context)
    assert [a.executable for a in actions if type(a).__name__ == 'Node'] == ['m3_wait_ready']
    started = handlers[0].on_exit(SimpleNamespace(returncode=0), context)
    controller = next(a for a in started if type(a).__name__ == 'Node')
    params = controller.parameters[0]
    assert controller.name == f'offboard_control_{instance}'
    assert params['vehicle_instance'] == instance and params['target_system_id'] == int(system_id)
    assert params['topic_namespace'] == namespace
    assert params['waypoints_file'] == COLLISION[path]
    assert (params['initial_global_x'], params['initial_global_y']) == origin


def test_both_drones_can_be_given_the_same_arguments(api):
    """The web launcher sends every ticked drone the same values; each picks out its own."""
    shared = {'spawn_x_2': '25.0', 'path_1': COLLISION[1]}
    first = collision_context(system_id='1', **shared)
    second = collision_context(topic_prefix='/px4_1', system_id='2', **shared)
    params = []
    for context in (first, second):
        _, handlers = collision_actions(api, context)
        started = handlers[0].on_exit(SimpleNamespace(returncode=0), context)
        params.append(next(a for a in started if type(a).__name__ == 'Node').parameters[0])
    assert params[0]['waypoints_file'] == COLLISION[1] and params[0]['initial_global_x'] == 0.0
    assert params[1]['initial_global_x'] == 25.0


def test_collision_timeout_starts_no_mission(api):
    context = collision_context()
    _, handlers = collision_actions(api, context)
    result = handlers[0].on_exit(SimpleNamespace(returncode=1), context)
    assert any(type(a).__name__ == 'EmitEvent' for a in result)
    assert not any(type(a).__name__ == 'Node' for a in result)
    context.is_shutdown = True
    assert handlers[0].on_exit(SimpleNamespace(returncode=0), context) == []


@pytest.mark.parametrize('bad', [
    {'system_id': '3'}, {'system_id': '0'}, {'topic_prefix': '/bad name'},
    {'path_1': '/missing/path.yaml'}, {'spawn_x_1': 'nan'},
])
def test_bad_collision_vehicle_rejected_before_anything_starts(api, bad):
    with pytest.raises((ValueError, OSError)):
        collision_actions(api, collision_context(**bad))


def test_collision_launch_starts_no_simulator():
    """The same file has to run in SITL and on a real drone."""
    text = (ROOT / 'launch' / 'collision_launch.py').read_text()
    for word in ('launch_sitl', 'px4_sitl.launch.py', 'MicroXRCEAgent', 'headless'):
        assert f"'{word}'" not in text
    for argument in ('topic_prefix', 'system_id'):
        assert f"'{argument}'" in text
