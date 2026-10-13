"""Exercise actual controllers/launch functions with in-memory ROS interfaces.

No ROS transport, simulator, or flight commands are used by these tests.
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from milestone2_multivehicle.vehicle import resolve_vehicle, validate_pair

ROOT = Path(__file__).parents[1]
PATH = str(ROOT / 'resource/milestone2_path.yaml')


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
    policy = SimpleNamespace(BEST_EFFORT=1, TRANSIENT_LOCAL=1, KEEP_LAST=1)
    module('rclpy.qos', QoSProfile=Action, QoSReliabilityPolicy=policy,
           QoSDurabilityPolicy=policy, QoSHistoryPolicy=policy)
    command = type('VehicleCommand', (), {'VEHICLE_CMD_COMPONENT_ARM_DISARM': 400,
                                         'VEHICLE_CMD_DO_SET_MODE': 176, 'VEHICLE_CMD_NAV_LAND': 21})
    ack = type('VehicleCommandAck', (), {'VEHICLE_CMD_RESULT_ACCEPTED': 0})
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
        name = '_m2_test_' + relative.replace('/', '_').replace('.', '_')
        spec = importlib.util.spec_from_file_location(name, ROOT / relative)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m

    return load


def make_controller(api, instance, **overrides):
    FakeNode.overrides = {'vehicle_instance': instance, 'waypoints_file': PATH, **overrides}
    return api('milestone2_multivehicle/offboard_control_solution.py').OffboardControl()


def position(**kwargs):
    return SimpleNamespace(**{'x': 0., 'y': 0., 'z': -5., 'heading': 0.,
                              'xy_valid': True, 'z_valid': True, **kwargs})


def test_all_endpoints_and_commands_are_isolated(api):
    first, second = make_controller(api, 0), make_controller(api, 1)
    for node, prefix, target in [(first, '', 1), (second, '/px4_1', 2)]:
        assert set(node.publishers) == {prefix + '/fmu/in/' + suffix for suffix in
                                       ('offboard_control_mode', 'trajectory_setpoint', 'vehicle_command')}
        assert set(node.subscriptions) == {prefix + '/fmu/out/' + suffix for suffix in
                                          ('vehicle_local_position_v1', 'vehicle_command_ack_v1')}
        node.arm()
        node.engage_offboard_mode()
        node.land()
        commands = node.vehicle_command_pub.messages
        assert [m.command for m in commands] == [400, 176, 21]
        assert all(m.target_system == target for m in commands)
        node.on_timer()
        assert node.trajectory_setpoint_pub.messages[-1].position == [0., 0., -5.]
        assert node.offboard_control_mode_pub.messages[-1].position is True
    assert set(first.publishers).isdisjoint(second.publishers)
    assert set(first.subscriptions).isdisjoint(second.subscriptions)


def test_hardware_controller_routes_all_endpoints_and_commands(api):
    node = make_controller(api, 1, topic_namespace='/drone_b', target_system_id=12)
    assert all(t.startswith('/drone_b/fmu/') for t in [*node.publishers, *node.subscriptions])
    node.arm()
    assert node.vehicle_command_pub.messages[-1].target_system == 12


def test_ack_position_and_mission_state_do_not_cross_vehicles(api):
    first, second = make_controller(api, 0), make_controller(api, 1)
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
    second.on_timer()
    assert second.trajectory_setpoint_pub.messages  # peer keeps streaming


def test_readiness_only_accepts_valid_selected_vehicle(api):
    FakeNode.overrides = {'vehicle_instance': 1}
    node = api('milestone2_multivehicle/wait_for_px4.py').WaitForPx4()
    assert list(node.subscriptions) == ['/px4_1/fmu/out/vehicle_local_position_v1']
    assert node.missing()
    callback = next(iter(node.subscriptions.values()))
    for msg in [position(xy_valid=False), position(z_valid=False), position(x=math.nan)]:
        callback(msg)
        assert node.missing()
    callback(position())
    assert node.missing() == ''


def launch_context(**overrides):
    return SimpleNamespace(is_shutdown=False, values={
        'launch_sitl': 'false', 'headless': 'true', 'px4_dir': '/tmp/PX4',
        'namespace_1': '/', 'namespace_2': '/px4_1',
        'system_id_1': '1', 'system_id_2': '2', 'path_1': PATH, 'path_2': PATH, **overrides})


def gate_actions(api, context):
    launch = api('launch/offboard_solution.launch.py')
    actions = launch.launch_pair(context)
    handlers = [a.args[0] for a in actions if type(a).__name__ == 'RegisterEventHandler']
    return actions, handlers


@pytest.mark.parametrize('order', [(0, 1), (1, 0)])
def test_pair_starts_only_when_both_are_ready(api, order):
    context = launch_context()
    actions, handlers = gate_actions(api, context)
    waiters = [a for a in actions if type(a).__name__ == 'Node']
    assert [w.executable for w in waiters] == ['m2_wait_ready', 'm2_wait_ready']
    assert handlers[order[0]].on_exit(SimpleNamespace(returncode=0), context) == []
    result = handlers[order[1]].on_exit(SimpleNamespace(returncode=0), context)
    controllers = [a for a in result if type(a).__name__ == 'Node']
    assert [c.name for c in controllers] == ['offboard_control_0', 'offboard_control_1']
    assert [c.parameters[0]['target_system_id'] for c in controllers] == [1, 2]
    assert len({c.parameters[0]['topic_namespace'] for c in controllers}) == 2


@pytest.mark.parametrize('order', [(0, 1), (1, 0)])
def test_timeout_starts_no_missions_even_if_peer_later_ready(api, order):
    context = launch_context()
    _, handlers = gate_actions(api, context)
    result = handlers[order[0]].on_exit(SimpleNamespace(returncode=1), context)
    assert any(type(a).__name__ == 'EmitEvent' for a in result)
    assert not any(type(a).__name__ == 'Node' for a in result)
    assert handlers[order[1]].on_exit(SimpleNamespace(returncode=0), context) == []


def test_shutdown_never_starts_controllers(api):
    context = launch_context()
    _, handlers = gate_actions(api, context)
    context.is_shutdown = True
    assert all(h.on_exit(SimpleNamespace(returncode=0), context) == [] for h in handlers)


@pytest.mark.parametrize('bad', [
    {'namespace_2': '/'}, {'system_id_2': '1'},
    {'launch_sitl': 'true', 'system_id_2': '12'}, {'path_2': '/missing/path.yaml'},
])
def test_bad_pair_rejected_before_actions_return(api, bad):
    with pytest.raises((ValueError, OSError)):
        gate_actions(api, launch_context(**bad))


@pytest.mark.parametrize('field,value', [('yaw', math.nan), ('hold_s', math.inf),
                                        ('pos', [0, 0, math.nan])])
def test_nonfinite_second_path_rejected_before_either_mission(api, tmp_path, field, value):
    data = yaml.safe_load(Path(PATH).read_text())
    data['waypoints'][0][field] = value
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='finite'):
        gate_actions(api, launch_context(path_2=str(path)))


def test_different_path_files_and_hardware_ids_reach_each_controller(api):
    path2 = str(ROOT / 'resource/milestone2_updown.yaml')
    context = launch_context(path_2=path2, namespace_1='/drone_a', namespace_2='/drone_b',
                             system_id_1='11', system_id_2='12')
    _, handlers = gate_actions(api, context)
    handlers[0].on_exit(SimpleNamespace(returncode=0), context)
    controllers = [a for a in handlers[1].on_exit(SimpleNamespace(returncode=0), context)
                   if type(a).__name__ == 'Node']
    assert [c.parameters[0]['waypoints_file'] for c in controllers] == [PATH, path2]
    assert [c.parameters[0]['target_system_id'] for c in controllers] == [11, 12]


def test_sitl_starts_one_agent_then_second_px4_after_first_ready(api, tmp_path, monkeypatch):
    launch = api('launch/px4_sitl.launch.py')
    binary = tmp_path / 'build/px4_sitl_default/bin/px4'
    binary.parent.mkdir(parents=True)
    binary.touch()
    binary.chmod(0o755)
    monkeypatch.setattr(launch.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=1))
    context = launch_context(px4_dir=str(tmp_path))
    actions = launch.start_sitl(context)
    assert len([a for a in actions if type(a).__name__ == 'ExecuteProcess']) == 1
    nodes = [a for a in actions if type(a).__name__ == 'Node']
    assert [n.name for n in nodes] == ['m2_px4_0', 'm2_wait_world']
    ready_handler = actions[0].args[0]
    second = ready_handler.on_exit(SimpleNamespace(returncode=0), context)[0]
    assert second.name == 'm2_px4_1'
    assert nodes[0].additional_env['PX4_UXRCE_DDS_NS'] == ''
    assert second.additional_env['PX4_UXRCE_DDS_NS'] == 'px4_1'
    assert second.additional_env['PX4_GZ_STANDALONE'] == '1'
    assert second.additional_env['PX4_GZ_MODEL_POSE'] == '20,0,0'
    failure = ready_handler.on_exit(SimpleNamespace(returncode=1), context)
    assert not any(type(a).__name__ == 'Node' for a in failure)


def test_same_basename_logs_produce_separate_vehicle_reports(api, monkeypatch, tmp_path):
    analyzer = api('milestone2_multivehicle/analyze_flight.py')
    monkeypatch.setattr(analyzer, 'read_flight', lambda _: (
        object(), {'log': 'flight.ulg', 'date': '2026-10-13', 'px4': 'test'}))
    monkeypatch.setattr(analyzer.scoring, 'score_flight',
                        lambda *a: SimpleNamespace(overall='PASS'))
    monkeypatch.setattr(analyzer.scoring, 'format_markdown',
                        lambda report, meta: f'Vehicle {meta["vehicle"]}')
    monkeypatch.setattr(analyzer.scoring, 'report_to_dict', lambda report, meta: meta)
    for vehicle in ('0', '1'):
        assert analyzer.main(['flight.ulg', '--vehicle', vehicle,
                              '--path', PATH, '--out', str(tmp_path)]) == 0
    assert len(list(tmp_path.glob('*.json'))) == 2
    assert (tmp_path / 'vehicle_0_flight_sitl.md').read_text() == 'Vehicle 0'
    assert (tmp_path / 'vehicle_1_flight_sitl.md').read_text() == 'Vehicle 1'


def test_controller_paths_do_not_require_scoring_metadata(api, tmp_path):
    path = tmp_path / 'mission.yaml'
    path.write_text('waypoints:\n  - {pos: [0, 0, -3], yaw: 0, hold_s: 5}\n')
    _, handlers = gate_actions(api, launch_context(path_1=str(path), path_2=str(path)))
    assert len(handlers) == 2
