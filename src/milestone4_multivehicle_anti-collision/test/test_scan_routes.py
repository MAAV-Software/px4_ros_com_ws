"""The field-scan routes: coverage, spacing, the YAMLs on disk, and an ORCA fly-through. No ROS."""
import itertools
import math
from pathlib import Path

import pytest

from milestone4_multivehicle import scan_routes as s
from milestone4_multivehicle.orca_avoidance import OrcaAvoidance
from test_orca import position
from test_multivehicle import api  # noqa: F401 (a fixture)

ROOT = Path(__file__).parents[1]


def world(drone, point):
    spawn = s.SPAWNS[drone]
    return spawn[0] + point['pos'][1], spawn[1] + point['pos'][0]


def covered(intervals, extent):
    """True when the union of (low, high) intervals covers [0, extent]."""
    reach = 0.0
    for low, high in sorted(intervals):
        if low > reach + 1e-9:
            return False
        reach = max(reach, high)
    return reach >= extent - 1e-9


@pytest.mark.parametrize('drone', range(4))
def test_yaml_on_disk_matches_the_generator(drone):
    assert (ROOT / f'resource/milestone4_scan_{drone}.yaml').read_text() == s.yaml_text(drone)


@pytest.mark.parametrize('drone', range(4))
def test_yaml_passes_the_launch_check(api, drone):
    api('milestone4_multivehicle/fleet_launch.py').check_waypoints(
        str(ROOT / f'resource/milestone4_scan_{drone}.yaml'))


def test_eight_rows_cover_the_width_with_the_long_side_of_the_footprint():
    east, north = s.footprint()
    assert (east, north) == pytest.approx((3.36, 2.24))
    rows = s.row_eastings()
    assert len(rows) == 8 and rows == sorted(rows)
    assert covered([(e - east / 2, e + east / 2) for e in rows], s.FIELD_EAST_M)
    gaps = [b - a for a, b in zip(rows, rows[1:])]
    assert max(gaps) - min(gaps) < 0.002 and east - max(gaps) > 0.3  # even, with overlap to spare
    assert [s.drone_rows(d) for d in range(4)] == [rows[0:2], rows[2:4], rows[4:6], rows[6:8]]


def test_row_ends_put_the_footprint_past_the_field_edge_when_the_drone_turns():
    south, north = s.row_ends()
    turn_south, turn_north = south + s.ACCEPTANCE_M, north - s.ACCEPTANCE_M
    half = s.footprint()[1] / 2
    assert turn_south - half <= 1e-9 and turn_north + half >= s.FIELD_NORTH_M - 1e-9
    assert (south, north) == pytest.approx((0.62, 90.82))


@pytest.mark.parametrize('drone', range(4))
def test_each_drone_flies_up_one_row_down_the_next_and_lands_south_of_the_field(drone):
    points = s.waypoints(drone)
    assert len(points) == 6
    first, second = s.drone_rows(drone)
    south, north = s.row_ends()
    assert [world(drone, p) for p in points] == [pytest.approx(w) for w in (
        s.SPAWNS[drone], (first, south), (first, north), (second, north), (second, south), (second, -3.))]


@pytest.mark.parametrize('drone', range(4))
def test_nose_stays_east_at_scan_altitude_with_the_holds(drone):
    points = s.waypoints(drone)
    assert all(p['yaw'] == pytest.approx(math.pi / 2, abs=1e-3) for p in points)
    assert all(p['pos'][2] == -5.6 for p in points)
    assert points[0]['pos'] == [0., 0., -5.6] and points[0]['hold_s'] == 10.
    assert all(p['hold_s'] == 0. for p in points[1:-1]) and points[-1]['hold_s'] == 2.


def test_drones_on_the_same_leg_are_six_metres_apart_and_land_apart():
    routes = [[world(d, p) for p in s.waypoints(d)] for d in range(4)]
    for leg in range(1, 6):
        assert all(math.dist(routes[d][leg], routes[d + 1][leg]) == pytest.approx(6.007, abs=.01)
                   for d in range(3))
    landings = [r[-1] for r in routes]
    assert all(math.dist(a, b) >= 6 for a, b in itertools.combinations(landings, 2))


def test_spawns_are_south_of_the_field_and_far_enough_apart():
    assert all(n < 0 and 0 <= e <= s.FIELD_EAST_M for e, n in s.SPAWNS)
    assert all(math.dist(a, b) >= 6 for a, b in itertools.combinations(s.SPAWNS, 2))


def test_scan_launch_flies_these_routes_from_these_origins(api):
    from test_multivehicle import collision_context  # noqa: E402
    launch = api('launch/scan_launch.py')
    declared = {a.args[0] if a.args else a.name: a for a in launch.generate_launch_description().args[0]
                if type(a).__name__ == 'DeclareLaunchArgument'}
    for i, (east, north) in enumerate(s.SPAWNS):
        assert declared[f'spawn_x_{i + 1}'].default_value == str(east)
        assert declared[f'spawn_y_{i + 1}'].default_value == str(north)
        assert declared[f'path_{i + 1}'].default_value.args[0][-1] == f'milestone4_scan_{i}.yaml'
    assert collision_context  # same context shape as collision_launch


def test_orca_fleet_flies_the_whole_scan_on_its_rows_without_contact():
    """Kinematic fly-through at 1 m/s with a 0.4 s velocity lag; yaw and climbs ignored.

    Run 4 in SITL bowed the rows of an earlier plan by up to 4 m (median 0.7 m off the row),
    from a right bias applied whenever a peer was within 10 m; this simulation reproduced it.
    """
    routes = [s.waypoints(d) for d in range(4)]
    fleet = [OrcaAvoidance(d, *s.SPAWNS[d], vehicle_count=4, max_speed=1.) for d in range(4)]
    local = [[0., 0.] for _ in range(4)]
    vel = [[0., 0.] for _ in range(4)]
    index, hold_since, done = [1] * 4, [None] * 4, [False] * 4
    closest, dt, now, cross_track, finished = math.inf, 0.1, 0., [], [None] * 4
    while not all(done) and now < 600:
        pos = [position(x=local[d][0], y=local[d][1], vx=vel[d][0], vy=vel[d][1]) for d in range(4)]
        for d in range(4):
            for a in fleet:
                a.receive(fleet[d].payload(pos[d]), now)
        for d in range(4):
            if done[d]:
                continue
            target = routes[d][index[d]]
            cmd = fleet[d].update(pos[d], now, target, routes[d][index[d] - 1])['vel']
            for axis in range(2):
                vel[d][axis] += (cmd[axis] - vel[d][axis]) * dt / .4
                local[d][axis] += vel[d][axis] * dt
            if math.dist(local[d], target['pos'][:2]) <= .5:
                hold_since[d] = now if hold_since[d] is None else hold_since[d]
                if now - hold_since[d] >= target['hold_s']:
                    hold_since[d] = None
                    if index[d] == len(routes[d]) - 1:
                        done[d], finished[d] = True, now  # lands; still an obstacle where it is
                        vel[d] = [0., 0.]
                    else:
                        index[d] += 1
            else:
                hold_since[d] = None
            east, north = fleet[d].global_xy(position(x=local[d][0], y=local[d][1]))
            if 5 < north < 86 and abs(vel[d][0]) > .5:  # on a row
                cross_track.append(min(abs(east - row) for row in s.drone_rows(d)))
        xy = [fleet[d].global_xy(position(x=local[d][0], y=local[d][1])) for d in range(4)]
        for a, b in itertools.combinations(range(4), 2):
            closest = min(closest, math.dist(xy[a], xy[b]))
        now += dt
    assert all(done), f'unfinished after {now:.0f} s: waypoints {index}'
    assert max(finished) < 215  # about 192 m of flying each, after takeoff
    assert closest >= 5.0  # 6 m in step; less at a corner, where shorter transits put a drone ahead
    cross_track.sort()
    assert cross_track[len(cross_track) // 2] < 0.1
    assert cross_track[int(.9 * len(cross_track))] < 0.3
