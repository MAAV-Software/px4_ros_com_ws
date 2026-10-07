"""Exercise controller methods without ROS publishers or flight commands."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def controller():
    # Load the actual methods while excluding ROS imports and node startup.
    source = Path(__file__).parents[1] / 'week4_multivehicle/offboard_control_solution.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    namespace = {'math': math, 'ACCEPTANCE_RADIUS': 0.5}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    node = namespace['OffboardControl'].__new__(namespace['OffboardControl'])
    node.waypoints = [
        {'pos': (0.0, 0.0, -5.0), 'yaw': 0.0, 'hold_s': 5.0},
        {'pos': (1.0, 0.0, -5.0), 'yaw': 0.0, 'hold_s': 0.0},
    ]
    node.waypoint_index = 0
    node.vehicle_instance = 1
    node.hold_started_ns = None
    node.current_position = SimpleNamespace(x=0.0, y=0.0, z=-5.0, heading=0.0,
                                           xy_valid=True, z_valid=True)
    node.now_ns = 0
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=node.now_ns))
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    node.landed = False
    node.land_calls = 0
    def land():
        node.land_calls += 1
    node.land = land
    return node


def test_hold_waits_before_advancing(controller):
    controller.advance_waypoint_if_reached()
    assert controller.waypoint_index == 0
    controller.now_ns = 4_900_000_000
    controller.advance_waypoint_if_reached()
    assert controller.waypoint_index == 0
    controller.now_ns = 5_000_000_000
    controller.advance_waypoint_if_reached()
    assert controller.waypoint_index == 1


def test_wrong_heading_blocks_hold(controller):
    controller.current_position.heading = math.radians(90)
    controller.advance_waypoint_if_reached()
    assert controller.hold_started_ns is None
    assert controller.waypoint_index == 0


def test_heading_wraparound(controller):
    controller.waypoints[0]['yaw'] = math.radians(179)
    controller.current_position.heading = math.radians(-179)
    controller.advance_waypoint_if_reached()
    assert controller.hold_started_ns == 0


@pytest.mark.parametrize('field,value', [('x', 1.0), ('heading', 0.2), ('xy_valid', False)])
def test_leaving_tolerance_resets_hold(controller, field, value):
    controller.advance_waypoint_if_reached()
    controller.now_ns = 4_000_000_000
    setattr(controller.current_position, field, value)
    controller.advance_waypoint_if_reached()
    assert controller.hold_started_ns is None
    setattr(controller.current_position, field, 0.0 if field != 'xy_valid' else True)
    controller.now_ns = 5_000_000_000
    controller.advance_waypoint_if_reached()
    controller.now_ns = 9_000_000_000
    controller.advance_waypoint_if_reached()
    assert controller.waypoint_index == 0
    controller.now_ns = 10_000_000_000
    controller.advance_waypoint_if_reached()
    assert controller.waypoint_index == 1


def test_final_hold_before_landing(controller):
    controller.waypoints = controller.waypoints[:1]
    controller.advance_waypoint_if_reached()
    assert controller.land_calls == 0
    controller.now_ns = 5_000_000_000
    controller.advance_waypoint_if_reached()
    assert controller.land_calls == 1
    assert controller.landed


def test_publisher_sends_position_and_yaw(controller):
    controller.publish_trajectory_setpoint.__globals__['TrajectorySetpoint'] = SimpleNamespace
    messages = []
    controller.trajectory_setpoint_pub = SimpleNamespace(publish=messages.append)
    controller.publish_trajectory_setpoint({'pos': (1.0, 2.0, -5.0), 'yaw': 1.5707963267948966})
    assert messages[0].position == [1.0, 2.0, -5.0]
    assert messages[0].yaw == 1.5707963267948966


def test_yaml_yaw_is_already_radians(tmp_path):
    import yaml
    source = Path(__file__).parents[1] / 'week4_multivehicle/offboard_control_solution.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    # Exercise the loading block before ROS publishers are constructed.
    body = []
    for stmt in init.body[1:]:
        if (isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call)
                and stmt.value.args and isinstance(stmt.value.args[0], ast.Constant)
                and stmt.value.args[0].value == 'vehicle_instance'):
            break
        body.append(stmt)
    path = tmp_path / 'path.yaml'
    path.write_text('waypoints:\n  - {pos: [0, 0, -5], yaw: 1.5707963267948966, hold_s: 3}\n')
    node = SimpleNamespace(
        declare_parameter=lambda *args: None,
        get_parameter=lambda *args: SimpleNamespace(value=str(path)),
    )
    exec(compile(ast.Module(body=body, type_ignores=[]), str(source), 'exec'),
         {'self': node, 'math': math, 'yaml': yaml})
    assert node.waypoints[0]['yaw'] == 1.5707963267948966


@pytest.mark.parametrize('instance,system_id', [(0, 1), (1, 2)])
def test_commands_target_selected_sitl_vehicle(controller, instance, system_id):
    controller.publish_vehicle_command.__globals__['VehicleCommand'] = SimpleNamespace
    controller.vehicle_instance = instance
    messages = []
    controller.vehicle_command_pub = SimpleNamespace(publish=messages.append)
    controller.publish_vehicle_command(400, param1=1.0)
    assert messages[0].target_system == system_id
