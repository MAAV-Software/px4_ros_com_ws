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


def test_lanes_are_two_metres_apart_and_cover_the_edge():
    width = s.lanes(s.FIELD_EAST_M)
    assert width[:12] == [1. + 2 * i for i in range(12)] and width[-1] == pytest.approx(23.384)
    length = s.lanes(s.FIELD_NORTH_M)
    assert len(length) == 46 and length[-1] == pytest.approx(90.44)
    assert all(b - a <= s.ACROSS_M + 1e-9 for a, b in zip(length, length[1:]))


def test_three_strips_split_the_width_into_thirds_and_cover_it():
    rows = [s.strip_rows(d) for d in range(3)]
    assert [len(r) for r in rows] == [4, 4, 5]
    assert sorted(itertools.chain(*rows)) == s.lanes(s.FIELD_EAST_M)
    for d, r in enumerate(rows):
        third = s.FIELD_EAST_M / 3
        assert all(d * third - 1 <= e <= (d + 1) * third + 1 for e in r)
    assert covered([(e - 1, e + 1) for e in itertools.chain(*rows)], s.FIELD_EAST_M)


@pytest.mark.parametrize('drone', range(3))
def test_strip_rows_run_the_full_length_facing_along_the_row(drone):
    points = s.waypoints(drone)
    rows = list(zip(points[1:-1:2], points[2:-1:2]))
    assert len(rows) == len(s.strip_rows(drone))
    for i, (start, end) in enumerate(rows):
        (e0, n0), (e1, n1) = world(drone, start), world(drone, end)
        assert e0 == e1 == s.strip_rows(drone)[i]
        assert sorted([n0 - 1.5, n1 + 1.5]) == pytest.approx([0., s.FIELD_NORTH_M]) or \
            sorted([n0 + 1.5, n1 - 1.5]) == pytest.approx([0., s.FIELD_NORTH_M])
        north = n1 > n0
        assert north == (i % 2 == 0)  # starts at the south end, then alternates
        assert start['yaw'] == end['yaw'] == pytest.approx(0. if north else math.pi, abs=1e-3)


def test_drone_3_sweeps_east_west_rows_from_the_south_over_the_whole_field():
    points = s.waypoints(3)
    rows = list(zip(points[1::2], points[2::2]))
    assert len(rows) == 46
    norths = []
    for i, (start, end) in enumerate(rows):
        (e0, n0), (e1, n1) = world(3, start), world(3, end)
        assert n0 == n1
        norths.append(n0)
        assert sorted([e0, e1]) == pytest.approx([1.5, s.FIELD_EAST_M - 1.5])
        west = e1 < e0
        assert west == (i % 2 == 0)  # starts from the east, nearest its spawn
        assert start['yaw'] == end['yaw'] == pytest.approx(-math.pi / 2 if west else math.pi / 2, abs=1e-3)
    assert norths == sorted(norths) and covered([(n - 1, n + 1) for n in norths], s.FIELD_NORTH_M)


@pytest.mark.parametrize('drone', range(4))
def test_takeoff_altitude_holds_and_landing_place(drone):
    points = s.waypoints(drone)
    assert points[0]['pos'] == [0., 0., -5.] and points[0]['hold_s'] == 10.
    assert all(p['pos'][2] == -5. for p in points)
    assert all(p['hold_s'] == 0. for p in points[1:-1]) and points[-1]['hold_s'] == 2.
    if drone < 3:
        assert points[-1]['pos'][:2] == [0., 0.]  # home, off the field
    else:
        assert world(3, points[-1])[1] > s.FIELD_NORTH_M - 2


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

    Run 4 in SITL bowed the strip rows by up to 4 m (median 0.7 m off the row), from a
    right bias applied whenever a peer was within 10 m; this simulation reproduced it.
    """
    routes = [s.waypoints(d) for d in range(4)]
    fleet = [OrcaAvoidance(d, *s.SPAWNS[d], vehicle_count=4, max_speed=1.) for d in range(4)]
    local = [[0., 0.] for _ in range(4)]
    vel = [[0., 0.] for _ in range(4)]
    index, hold_since, done = [1] * 4, [None] * 4, [False] * 4
    closest, dt, now, cross_track = math.inf, 0.1, 0., []
    while not all(done) and now < 2500:
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
                        done[d] = True  # lands; still an obstacle where it is
                        vel[d] = [0., 0.]
                    else:
                        index[d] += 1
            else:
                hold_since[d] = None
            east, north = fleet[d].global_xy(position(x=local[d][0], y=local[d][1]))
            if d < 3 and 5 < north < 86 and abs(vel[d][0]) > .5:  # on a strip row
                cross_track.append(min(abs(east - row) for row in s.strip_rows(d)))
        xy = [fleet[d].global_xy(position(x=local[d][0], y=local[d][1])) for d in range(4)]
        for a, b in itertools.combinations(range(4), 2):
            closest = min(closest, math.dist(xy[a], xy[b]))
        now += dt
    assert all(done), f'unfinished after {now:.0f} s: waypoints {index}'
    assert closest >= 2.7
    cross_track.sort()
    assert cross_track[len(cross_track) // 2] < 0.1
    assert cross_track[int(.9 * len(cross_track))] < 0.3
