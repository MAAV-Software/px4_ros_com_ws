"""ORCA solver and OrcaAvoidance, with a small kinematic fleet simulation. No ROS."""
import itertools
import math
from types import SimpleNamespace

import pytest

from milestone4_multivehicle import orca
from milestone4_multivehicle.orca_avoidance import OrcaAvoidance, RADIUS_M, PEER_TIMEOUT_S

ORIGINS = ((0., 0.), (20., 0.), (20., 20.), (0., 20.))
# milestone4_headon_N.yaml's second waypoint, local north/east: each drone to the opposite corner.
HEADON = ((20., 20.), (20., -20.), (-20., -20.), (-20., 20.))


def position(**kwargs):
    return SimpleNamespace(**{'x': 0., 'y': 0., 'z': -5., 'heading': 0.,
                              'xy_valid': True, 'z_valid': True, 'v_xy_valid': True,
                              'vx': 0., 'vy': 0., **kwargs})


def waypoint(north, east):
    return {'pos': (north, east, -5.), 'yaw': 0., 'hold_s': 0.}


def fly(starts, goals, origins, max_speed=1.0, seconds=120., dt=0.1, lag_s=0.4, silent=()):
    """Fly OrcaAvoidance drones with a first-order velocity lag (like PX4 tracking a velocity setpoint).

    starts/goals are local north/east per drone. Drones in `silent` stop publishing
    after their first message and never move. Returns (min separation, arrived flags).
    """
    n = len(starts)
    fleet = [OrcaAvoidance(i, *origins[i], vehicle_count=n, max_speed=max_speed) for i in range(n)]
    local = [list(s) for s in starts]
    vel = [[0., 0.] for _ in range(n)]
    closest = math.inf
    arrived = [False] * n
    for k in range(int(seconds / dt)):
        now = k * dt
        positions = [position(x=local[i][0], y=local[i][1], vx=vel[i][0], vy=vel[i][1]) for i in range(n)]
        for i in range(n):
            if i in silent and k > 0:
                continue
            for a in fleet:
                a.receive(fleet[i].payload(positions[i]), now)
        commands = []
        for i in range(n):
            if i in silent:
                commands.append((0., 0.))
                continue
            commands.append(fleet[i].update(positions[i], now, waypoint(*goals[i]))['vel'])
        for i in range(n):
            for axis in range(2):
                vel[i][axis] += (commands[i][axis] - vel[i][axis]) * min(1., dt / lag_s)
                local[i][axis] += vel[i][axis] * dt
            if math.dist(local[i], goals[i]) < 0.5:
                arrived[i] = True
        world = [fleet[i].global_xy(position(x=local[i][0], y=local[i][1])) for i in range(n)]
        for a, b in itertools.combinations(range(n), 2):
            closest = min(closest, math.dist(world[a], world[b]))
    return closest, arrived


def test_no_neighbors_returns_preferred_velocity_clamped_to_max_speed():
    assert orca.solve((0, 0), (0, 0), 1.5, [], (3., 4.), 1., 3., .1) == pytest.approx((.6, .8))
    assert orca.solve((0, 0), (0, 0), 1.5, [], (.3, .4), 1., 3., .1) == (.3, .4)


def test_far_neighbor_moving_away_does_not_change_velocity():
    far = orca.Neighbor(0., 30., 0., 1., 1.5, .5)
    assert orca.solve((0, 0), (0, 1.), 1.5, [far], (0., 1.), 1., 3., .1) == pytest.approx((0., 1.))


def test_head_on_pair_both_turn_the_same_way():
    # Reciprocal: each takes half, and the solver result never exceeds max speed.
    a = orca.solve((0, 0), (0, 1.), 1.5, [orca.Neighbor(.5, 6., 0., -1., 1.5, .5)], (0., 1.), 1., 3., .1)
    assert math.hypot(*a) <= 1. + 1e-9
    assert a[0] < 0  # neighbour is slightly right, so step left


def test_overlap_is_resolved_within_one_step():
    a = orca.solve((0, 0), (0, 0), 1.5, [orca.Neighbor(1., 0., 0., 0., 1.5, .5)], (1., 0.), 1., 3., .1)
    assert a[0] < 0  # push apart, not into the neighbour


def test_infeasible_constraints_return_least_violating_velocity():
    ring = [orca.Neighbor(2.5 * math.cos(t), 2.5 * math.sin(t), -math.cos(t), -math.sin(t), 1.5, .5)
            for t in (0, 2.1, 4.2)]
    v = orca.solve((0, 0), (0, 0), 1.5, ring, (1., 0.), 1., 3., .1)
    assert all(math.isfinite(c) for c in v) and math.hypot(*v) <= 1. + 1e-9


@pytest.mark.parametrize('max_speed', [1.0, 2.0])
def test_head_on_pair_passes_with_margin(max_speed):
    closest, arrived = fly([(0., 0.), (0., 0.)], [(0., 20.), (0., -20.)], [(0., 0.), (20., 0.)],
                           max_speed=max_speed)
    assert closest >= 1.8 * RADIUS_M
    assert all(arrived)


@pytest.mark.parametrize('max_speed', [1.0, 2.0])
def test_four_way_headon_routes_pass_with_margin(max_speed):
    closest, arrived = fly([(0., 0.)] * 4, HEADON, ORIGINS, max_speed=max_speed)
    assert closest >= 1.8 * RADIUS_M
    assert all(arrived)


def test_silent_peer_is_avoided_as_a_static_obstacle():
    # Drone 1 sits silently in the middle of drone 0's route.
    closest, arrived = fly([(0., 0.), (0., -10.)], [(0., 20.), (0., -10.)],
                           [(0., 0.), (20., 0.)], silent={1})
    assert closest >= 2 * RADIUS_M
    assert arrived[0]


def test_shared_frame_swaps_north_east_for_position_and_velocity():
    a = OrcaAvoidance(1, 20., -3.)
    assert a.payload(position(x=7., y=2., vx=1., vy=-2.)) == [1., 22., 4., -2., 1.]


def test_payload_without_valid_velocity_still_shares_position():
    data = OrcaAvoidance(0, 0., 0.).payload(position(v_xy_valid=False))
    assert data[:3] == [0., 0., 0.] and all(math.isnan(v) for v in data[3:])
    b = OrcaAvoidance(1, 20., 0.)
    b.receive(data, 0.)
    assert b.peers[0] == (0., 0., 0., 0., 0.)


@pytest.mark.parametrize('data', [[0, 1, 2, 0, 0], [1.5, 1, 2, 0, 0], [4, 1, 2, 0, 0],
                                 [1, math.nan, 2, 0, 0], [1, 1, 2], [1, 1, 2, 0, 0, 0]])
def test_bad_self_or_priority_style_messages_are_ignored(data):
    a = OrcaAvoidance(0, 0., 0.)
    a.receive(data, 0.)
    assert not a.peers


def test_peers_ready_is_symmetric_and_needs_fresh_messages():
    a = OrcaAvoidance(2, 0., 0.)
    for i in (0, 1, 3):
        a.receive([i, 50., 50., 0., 0.], 0.)
    assert a.peers_ready(PEER_TIMEOUT_S)
    assert not a.peers_ready(PEER_TIMEOUT_S + .01)
    del a.peers[0]
    assert not a.peers_ready(0.)


def test_clear_flight_flies_straight_at_waypoint_altitude_and_keeps_progress():
    a = OrcaAvoidance(0, 0., 0.)
    target = a.update(position(z=-2.), 0., {'pos': (10., 0., -7.), 'yaw': .5, 'hold_s': 3.})
    assert target['vel'] == pytest.approx((1., 0.))
    assert math.isnan(target['pos'][0]) and target['pos'][2] == -7.
    assert target['yaw'] == .5 and target['keep_progress']
    assert a.state == 'CLEAR'
    # Slows into the waypoint.
    assert a.update(position(x=9.5), 0., waypoint(10., 0.))['vel'] == pytest.approx((.4, 0.))


def test_avoiding_state_latches_for_a_second():
    a = OrcaAvoidance(0, 0., 0.)
    a.receive([1, 0., 4., 0., -1.], 0.)  # straight ahead, coming at us
    a.update(position(vx=1.), 0., waypoint(20., 0.))
    assert a.state == 'AVOIDING'
    a.receive([1, 0., 40., 0., 0.], .5)
    a.update(position(vx=1.), .5, waypoint(20., 0.))
    assert a.state == 'AVOIDING'
    a.receive([1, 0., 40., 0., 0.], 1.2)
    a.update(position(vx=1.), 1.2, waypoint(20., 0.))
    assert a.state == 'CLEAR'


def test_invalid_own_velocity_holds_position():
    a = OrcaAvoidance(0, 0., 0.)
    target = a.update(position(x=3., v_xy_valid=False), 0., waypoint(20., 0.))
    assert a.state == 'DATA_HOLD' and target['pos'] == (3., 0., -5.)
    assert 'keep_progress' not in target


@pytest.mark.parametrize('instance,count,speed', [(4, 4, 1.), (-1, 4, 1.), (0, 1, 1.), (0, 4, 0.), (0, 4, math.nan)])
def test_invalid_fleet_identity_or_speed(instance, count, speed):
    with pytest.raises(ValueError):
        OrcaAvoidance(instance, 0., 0., count, speed)


# --- the controller and launch file with avoidance:=orca ----------------------------

from test_multivehicle import api, make_controller, collision_context, started_controller  # noqa: E402,F401


def orca_controller(api, tmp_path, instance=0, hold_s=5):
    path = tmp_path / 'mission.yaml'
    path.write_text('waypoints:\n'
                    f'  - {{pos: [0, 0, -5], yaw: 0, hold_s: {hold_s}}}\n'
                    f'  - {{pos: [10, 0, -5], yaw: 0, hold_s: {hold_s}}}\n')
    return make_controller(api, instance, avoidance='orca', waypoints_file=str(path))


def test_controller_shares_velocity_and_flies_velocity_setpoints(api, tmp_path):
    node = orca_controller(api, tmp_path)
    node.waypoint_index = 1
    node.on_local_position(position(vx=.5, vy=.25))
    node.on_global_position(SimpleNamespace(data=[1., 50., 50., 0., 0.]))
    node.on_timer()
    assert node.global_position_pub.messages[-1].data == [0., 0., 0., .25, .5]
    assert node.offboard_control_mode_pub.messages[-1].velocity is True
    sp = node.trajectory_setpoint_pub.messages[-1]
    assert math.isnan(sp.position[0]) and math.isnan(sp.position[1]) and sp.position[2] == -5.
    assert sp.velocity[:2] == pytest.approx([1., 0.]) and math.isnan(sp.velocity[2])


def test_priority_controller_still_flies_position_setpoints(api):
    node = make_controller(api, 0)
    node.on_local_position(position())
    node.on_timer()
    assert node.offboard_control_mode_pub.messages[-1].velocity is False
    assert node.global_position_pub.messages[-1].data == [0., 0., 0.]
    assert not hasattr(node.trajectory_setpoint_pub.messages[-1], 'velocity')


def test_waypoint_hold_counts_while_avoiding_and_landing_waits_for_clear(api, tmp_path):
    node = orca_controller(api, tmp_path)
    node.confirmed_offboard = True
    node.waypoint_index = 1
    here = position(x=10.)
    node.on_local_position(here)
    # A peer right beside the waypoint, closing: ORCA deviates.
    node.on_global_position(SimpleNamespace(data=[1., 2., 10., -1., 0.]))
    node.on_timer()
    assert node.avoidance.state == 'AVOIDING'
    assert node.hold_started_ns is not None  # progress continues while avoiding
    node.now_ns = 6_000_000_000
    node.on_local_position(here)
    node.on_global_position(SimpleNamespace(data=[1., 2., 10., -1., 0.]))
    node.on_timer()
    assert not node.landed and node.hold_started_ns is not None
    # The peer leaves; a second later ORCA is quiet and the drone lands without a new hold.
    for t in (6.5, 7.0, 7.5, 8.1):
        node.now_ns = int(t * 1e9)
        node.on_local_position(here)
        node.on_global_position(SimpleNamespace(data=[1., 60., 60., 0., 0.]))
        node.on_timer()
    assert node.landed
    assert node.vehicle_command_pub.messages[-1].command == 21


def test_launch_passes_avoidance_and_speed_to_the_controller(api):
    params = started_controller(api, collision_context(avoidance='priority', max_speed='2')).parameters[0]
    assert params['avoidance'] == 'priority' and params['max_speed'] == 2.0
    params = started_controller(api, collision_context()).parameters[0]
    assert params['avoidance'] == 'orca' and params['max_speed'] == 1.0


def test_off_the_leg_steers_back_onto_it_not_straight_at_the_waypoint():
    a = OrcaAvoidance(0, 0., 0.)
    start, end = waypoint(0., 0.), waypoint(80., 0.)  # a row due north
    vn, ve = a.update(position(x=10., y=3.), 0., end, start)['vel']  # local north, east
    assert vn > 0 and ve < -0.5  # mostly back west toward the row, still moving north
    straight = a.update(position(x=10., y=3.), 0., end)['vel']
    assert straight[1] == pytest.approx(-3. / math.hypot(70., 3.), abs=1e-3)  # nearly due north


def test_no_bias_for_a_peer_that_is_not_in_the_way():
    a = OrcaAvoidance(0, 0., 0.)
    a.receive([1, 8., 30., 0., -1.], 0.)  # a parallel row 8 m east, flying the other way
    target = a.update(position(x=20., vx=1.), 0., waypoint(80., 0.), waypoint(0., 0.))
    assert target['vel'] == pytest.approx((1., 0.))
    assert a.state == 'CLEAR'

