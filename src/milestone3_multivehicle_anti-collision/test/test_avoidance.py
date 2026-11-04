"""Priority, frame and yield regressions without ROS or a simulator."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from milestone3_multivehicle.avoidance import Avoidance
from test_multivehicle import api, make_controller, position, launch_context, gate_actions


def test_shared_frame_uses_fixed_origin_and_swaps_north_east():
    a = Avoidance(1, 20., -3.)
    assert a.global_xy(position(x=7., y=2.)) == (22., 4.)
    assert a.global_xy(position(x=-1., y=-4.)) == (16., -4.)
    assert a.origin == (20., -3.)


@pytest.mark.parametrize('x,y,conflict', [(3, 4, True), (3, 4.01, False), (0, 0, True)])
@pytest.mark.parametrize('z', [-1., -100.])
def test_radius_is_horizontal_and_includes_five_metres(x, y, conflict, z):
    a = Avoidance(0, 0., 0.)
    a.receive([1., x, y], 0.)
    assert a.conflicts(position(z=z)) is conflict


def test_priority_is_instance_not_message_order():
    a = Avoidance(1, 20., 0.)
    a.receive([0., 20., 0.], 0.)
    assert a.update(position(), 0.) is None
    assert a.state == 'NORMAL'


@pytest.mark.parametrize('data', [[0, 1, 2], [1.5, 1, 2], [2, 1, 2],
                                 [1, math.nan, 2], [1, 1, math.inf], [1, 1], [1, 1, 2, 3]])
def test_bad_or_self_messages_do_not_replace_peer(data):
    a = Avoidance(0, 0., 0.)
    a.receive(data, 0.)
    assert not a.peers


def test_stop_climb_wait_resume_without_repeated_climbs():
    a = Avoidance(0, 0., 0.)
    p = position(z=-3., vx=1.)
    a.receive([1, 4, 0], 0.)
    assert a.update(p, 0.)['pos'] == (0., 0., -3.)
    assert a.state == 'BRAKE'
    p.vx = 0.
    a.update(p, 1.)
    assert a.update(p, 1.4)['pos'][2] == -3.
    assert a.update(p, 1.5)['pos'] == (0., 0., -8.)
    assert a.state == 'CLIMB'
    p.z = -8.
    a.update(p, 2.)
    assert a.state == 'WAIT'
    # Remain at the same absolute target; no second 5 m step, no stale release.
    for t in (3., 9., 30.):
        assert a.update(p, t)['pos'] == (0., 0., -8.)
    a.receive([1, 5, 0], 31.)
    assert a.update(p, 31.) is not None
    a.receive([1, 5.01, 0], 32.)
    assert a.update(p, 32.) is None
    # A new encounter takes a new altitude snapshot.
    a.receive([1, 4, 0], 33.)
    a.update(p, 33.)
    assert a.climb_z == -13.


def test_early_clearance_finishes_climb_before_resuming():
    a = Avoidance(0, 0., 0.)
    a.receive([1, 1, 0], 0.)
    a.update(position(), 0.)
    a.receive([1, 20, 0], .5)
    assert a.update(position(), .5)['pos'][2] == -10.
    assert a.update(position(), .6) is not None
    a.update(position(z=-10.), .7)
    assert a.state == 'WAIT'
    assert a.update(position(z=-10.), .8) is None


def test_missing_stale_and_restored_peer_hold_instead_of_assuming_clear():
    a = Avoidance(0, 0., 0.)
    assert a.update(position(), 0.) is not None
    assert a.state == 'DATA_HOLD'
    a.receive([1, 20, 0], 1.)
    assert a.update(position(), 1.) is None
    assert a.update(position(), 2.01) is not None
    a.receive([1, 20, 0], 3.)
    assert a.update(position(), 3.) is None


def test_invalid_velocity_never_finishes_braking():
    a = Avoidance(0, 0., 0.)
    a.receive([1, 1, 0], 0.)
    a.update(position(v_xy_valid=False), 0.)
    a.update(position(vx=math.nan), 10.)
    assert a.state == 'BRAKE'


def test_controller_yields_without_consuming_waypoint_or_hold(api):
    node = make_controller(api, 0)
    node.confirmed_offboard = True
    node.hold_started_ns = 0
    node.on_local_position(position())
    node.on_global_position(SimpleNamespace(data=[1., 3., 0.]))
    node.on_timer()
    assert node.avoidance.state == 'BRAKE'
    assert node.hold_started_ns is None and node.waypoint_index == 0
    for ns, z in [(600_000_000, -5.), (1_000_000_000, -10.), (6_000_000_000, -10.)]:
        node.now_ns = ns
        node.on_local_position(position(z=z))
        node.on_global_position(SimpleNamespace(data=[1., 3., 0.]))
        node.on_timer()
        assert node.waypoint_index == 0 and node.hold_started_ns is None
    assert node.trajectory_setpoint_pub.messages[-1].position == [0., 0., -10.]
    assert not node.vehicle_command_pub.messages  # no premature land
    node.on_global_position(SimpleNamespace(data=[1., 6., 0.]))
    node.on_timer()
    assert node.trajectory_setpoint_pub.messages[-1].position == [0., 0., -5.]
    assert node.waypoint_index == 0


def test_wire_payload_and_sharing_continue_after_land(api):
    node = make_controller(api, 1, initial_global_x=12., initial_global_y=-8.)
    node.on_local_position(position(x=2., y=3.))
    node.landed = True
    node.on_timer()
    assert node.global_position_pub.messages[-1].data == [1., 15., -6.]
    assert not node.trajectory_setpoint_pub.messages
    node.now_ns = 2_000_000_000
    node.on_timer()
    assert len(node.global_position_pub.messages) == 1  # do not rebroadcast stale PX4 data


def test_missing_peer_blocks_lower_instance_arming(api):
    node = make_controller(api, 0)
    for tick in range(20):
        node.now_ns = tick * 100_000_000
        node.on_local_position(position())
        node.on_timer()
    assert not node.vehicle_command_pub.messages
    assert node.waypoint_index == 0


def test_high_priority_does_not_yield_to_lower(api):
    node = make_controller(api, 1)
    node.on_local_position(position())
    node.on_global_position(SimpleNamespace(data=[0., 20., 0.]))
    node.on_timer()
    assert node.avoidance.state == 'NORMAL'
    assert node.trajectory_setpoint_pub.messages[-1].position == [0., 0., -5.]


def test_launch_origins_are_identical_for_simulator_and_controllers(api):
    ctx = launch_context(launch_sitl='true', spawn_x_1='-2', spawn_y_1='3',
                         spawn_x_2='7', spawn_y_2='8')
    actions, handlers = gate_actions(api, ctx)
    include = next(a for a in actions if type(a).__name__ == 'IncludeLaunchDescription')
    args = dict(include.launch_arguments)
    handlers[0].on_exit(SimpleNamespace(returncode=0), ctx)
    nodes = [a for a in handlers[1].on_exit(SimpleNamespace(returncode=0), ctx)
             if type(a).__name__ == 'Node']
    for i, node in enumerate(nodes, 1):
        for axis in ('x', 'y'):
            assert node.parameters[0][f'initial_global_{axis}'] == float(args[f'spawn_{axis}_{i}'])


def test_nonfinite_origin_rejected_before_startup(api):
    with pytest.raises(ValueError, match='finite'):
        gate_actions(api, launch_context(spawn_x_2='nan'))


def test_two_demo_controllers_exchange_positions_yield_resume_and_finish(api):
    """End-to-end decision loop with a simple rate-limited plant, not PX4 dynamics."""
    root = Path(__file__).parents[1]
    nodes = [make_controller(api, i, waypoints_file=str(
        root / 'resource' / f'milestone3_collision_{i}.yaml')) for i in range(2)]
    positions = [position(z=0.), position(z=0.)]
    histories = [set(), set()]
    for node in nodes:
        node.confirmed_offboard = True  # isolate flight decisions from arm ACK handling
    for tick in range(1800):
        for node, p in zip(nodes, positions):
            node.now_ns = tick * 100_000_000
            node.on_local_position(p)
            node.publish_global_position()
        for node in nodes:
            for peer in nodes:
                node.on_global_position(peer.global_position_pub.messages[-1])
            node.on_timer()
        for i, (node, p) in enumerate(zip(nodes, positions)):
            histories[i].add(node.avoidance.state)
            if node.landed:
                continue
            tx, ty, tz = node.trajectory_setpoint_pub.messages[-1].position
            dx, dy = tx - p.x, ty - p.y
            distance = math.hypot(dx, dy)
            scale = min(1., .2 / distance) if distance else 0.
            positions[i] = position(x=p.x + dx * scale, y=p.y + dy * scale,
                                    z=p.z + max(-.1, min(.1, tz - p.z)),
                                    vx=dx * scale / .1, vy=dy * scale / .1)
        if all(node.landed for node in nodes):
            break
    assert all(node.landed for node in nodes)
    assert {'BRAKE', 'CLIMB', 'WAIT', 'NORMAL'} <= histories[0]
    assert histories[1] == {'NORMAL'}
    assert [node.waypoint_index for node in nodes] == [1, 1]
    assert all(node.vehicle_command_pub.messages[-1].command == 21 for node in nodes)


def test_own_position_loss_pauses_mission_and_does_not_count_as_stopped(api):
    node = make_controller(api, 0)
    node.confirmed_offboard = True
    node.on_local_position(position())
    node.on_global_position(SimpleNamespace(data=[1., 1., 0.]))
    node.on_timer()
    assert node.avoidance.stopped_since == 0.
    node.now_ns = 2_000_000_000
    node.on_timer()
    assert node.avoidance.stopped_since is None
    assert node.waypoint_index == 0
    assert len(node.global_position_pub.messages) == 1
    node.on_local_position(position())
    node.on_global_position(SimpleNamespace(data=[1., 1., 0.]))
    node.on_timer()
    assert node.avoidance.state == 'BRAKE'
