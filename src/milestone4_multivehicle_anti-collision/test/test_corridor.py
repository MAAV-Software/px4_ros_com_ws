"""The corridor search: formations, the risk-aware final path, and whole flights under ORCA. No ROS."""
import itertools
import math

import pytest

from milestone4_multivehicle import corridor as c
from milestone4_multivehicle import mines as m
from milestone4_multivehicle import scan_routes as s
from milestone4_multivehicle.orca_avoidance import OrcaAvoidance
from test_orca import position
from test_multivehicle import api  # noqa: F401 (a fixture)


def test_formations_keep_three_metres_and_trade_width_for_looks():
    for name, slots in c.formations().items():
        places = [(east, -back * c.ALONG_M) for east, back in slots]
        assert all(math.dist(a, b) >= 3.2 - 1e-9 for a, b in itertools.combinations(places, 2)), name
    assert [c.pick_formation(p) for p in (.5, .75, .9, .99)] == ['astern', 'astern', '2x2', 'abreast']
    widths = {name: c.CorridorSearch(set(), formation=name, log=lambda _: None) for name in c.formations()}
    assert [round(widths[n].half_band * 2, 2) for n in ('abreast', '2x2', 'astern')] == [12.96, 6.56, 3.36]


def test_posterior_falls_with_looks_and_the_final_path_prefers_ground_seen_more():
    assert c.posterior(0, .9, .025) == pytest.approx(.025)
    assert c.posterior(2, .5, .025) == pytest.approx(.025 * .25 / (.025 * .25 + .975))
    assert c.posterior(3, 1., .025) == 0.
    # Two equal strips; one was looked at once and the other three times.
    looks = {(col, r): 1 for col in range(5, 10) for r in range(m.ROWS)}
    looks.update({(col, r): 3 for col in range(25, 30) for r in range(m.ROWS)})
    plan, death = c.best_plan(set(), looks, .5, .025)
    assert plan.complete and plan.green == 2 and plan.cells[0][0] == 27
    assert death == pytest.approx(1 - (1 - c.posterior(3, .5, .025)) ** 150)
    once, risk = c.best_plan(set(), {k: 1 for k in looks}, .5, .025)
    assert risk > 2 * death
    assert c.best_plan(set(), {}, .9, .025) == (None, None)
    wall = {(col, 70) for col in range(m.COLS)}
    assert c.best_plan(wall, {cell: 1 for cell in c.ALL_CELLS}, .9, .025) == (None, None)


def fly(formation, seed=10, probability=.9, limit_s=900.):
    """A whole flight, kinematically, with ORCA. Returns (search, log, seconds, closest pair)."""
    log = []
    search = c.CorridorSearch(m.place_mines(150, seed), probability, seed, formation=formation, log=log.append)
    fleet = [OrcaAvoidance(d, *s.SPAWNS[d], vehicle_count=4, max_speed=1.) for d in range(4)]
    local, vel = [[0., 0.] for _ in range(4)], [[0., 0.] for _ in range(4)]
    routes = [[s.SPAWNS[d]] for d in range(4)]
    index, taken, done = [0] * 4, [0] * 4, [False] * 4
    closest, dt, now = math.inf, 0.1, 0.

    def way(d, point):
        return {'pos': (point[1] - s.SPAWNS[d][1], point[0] - s.SPAWNS[d][0], 0.), 'yaw': s.YAW, 'hold_s': 0.}

    while not all(done) and now < limit_s:
        pos = [position(x=local[d][0], y=local[d][1], vx=vel[d][0], vy=vel[d][1]) for d in range(4)]
        xy = [fleet[d].global_xy(pos[d]) for d in range(4)]
        for d in range(4):
            for a in fleet:
                a.receive(fleet[d].payload(pos[d]), now)
            search.observe(d, *xy[d], 0. if done[d] else s.ALTITUDE_M, s.YAW, now)
        for d in range(4):
            if done[d]:
                continue
            version, final, points = m.read_message(search.message(d))
            if version > taken[d]:  # the controller's take_route
                routes[d][index[d] + 1:] = [xy[d]] + [p[:2] for p in points]
                index[d] += 2
                taken[d] = version
            target = routes[d][index[d]]
            cmd = fleet[d].update(pos[d], now, way(d, target), way(d, routes[d][index[d] - 1]) if index[d] else None)['vel']
            for axis in range(2):
                vel[d][axis] += (cmd[axis] - vel[d][axis]) * dt / .4
                local[d][axis] += vel[d][axis] * dt
            if math.dist(xy[d], target) <= .5:
                if index[d] < len(routes[d]) - 1:
                    index[d] += 1
                elif final:
                    done[d], vel[d] = True, [0., 0.]
        for a, b in itertools.combinations(range(4), 2):
            closest = min(closest, math.dist(xy[a], xy[b]))
        now += dt
    assert all(done), f'unfinished after {now:.0f} s in phase {search.phase}\n' + '\n'.join(log[-6:])
    return search, log, now, closest


@pytest.mark.parametrize('formation,looks,limit', [('abreast', 1, 200), ('2x2', 2, 330), ('astern', 4, 520)])
def test_a_formation_flies_the_band_and_ends_on_a_complete_path(formation, looks, limit):
    search, log, seconds, closest = fly(formation)
    assert search.final and search.plan is not None and search.plan.complete
    assert closest >= 2.7 and seconds < limit
    corridor = search.plan.corridor()
    assert not corridor & set(search.detector.found)
    assert min(search.looks[cell] for cell in corridor) >= 1
    counts = sorted(search.looks.values())
    assert counts[len(counts) // 2] >= looks
    assert any(line.startswith('Final: S,') for line in log)


def test_a_wall_across_the_band_makes_the_formation_sidestep():
    # A solid wall across the middle band, rows 60-61, with open field either side.
    wall = {(col, row) for col in range(12, 28) for row in (60, 61)}
    log = []
    search = c.CorridorSearch(wall, 1., 1, formation='2x2', log=log.append)
    for d in range(4):
        east, north = search.target(d, search.start_north)
        search.observe(d, east, north, s.ALTITUDE_M, s.YAW, 0.)
    search.phase = 'transit'
    for step in range(1, 600):  # glide the formation north from the south edge, a look at a time
        for d in range(4):
            east, north = search.target(d, search.start_north + step * .1)
            search.observe(d, east, north, s.ALTITUDE_M, s.YAW, step * .1)
        if search.phase == 'sidestep':
            break
    assert search.phase == 'sidestep' and search.sidesteps == 1
    # Almost a whole band sideways: the new band just covers the old one's edge.
    assert 4.8 < abs(search.centre - s.FIELD_EAST_M / 2) < search.half_band * 2
    assert search.restart_north < 60 * m.SQUARE_M
    version, final, points = m.read_message(search.message(0))
    assert len(points) == 2 and not final and points[0][1] < points[1][1]
    assert any('blocked at row' in line for line in log)


def test_corridor_launch_offers_formation_and_mines(api):
    launch = api('launch/corridor_launch.py')
    declared = {a.args[0] if a.args else a.name: a for a in launch.generate_launch_description().args[0]
                if type(a).__name__ == 'DeclareLaunchArgument'}
    assert declared['formation'].default_value == 'auto' and declared['mines'].default_value == '150'
    assert declared['path_1'].default_value.args[0][-1] == 'milestone4_corridor_0.yaml'


def test_the_frontier_is_where_only_unscanned_ground_stops_the_path():
    # Columns 10-19 are scanned up to row 30, with a wall across them at row 20.
    scanned = {(col, row) for col in range(10, 20) for row in range(31)}
    wall = {(col, 20) for col in range(10, 20)}
    top, cells = c.frontier(wall, scanned, green=0)
    assert top == 19
    # The path could carry on past either edge of the scanned band, or start beside it...
    assert (9, 19) in cells and (20, 19) in cells and (0, 0) in cells
    # ...but not through the wall, and nothing north of the wall is in reach.
    assert not any(row >= 20 for _, row in cells)
    # Ground about to be scanned opens no frontier.
    west = {(col, row) for col in range(10) for row in range(m.ROWS)}
    assert all(col >= 20 for col, _ in c.frontier(wall, scanned, 0, pending=west)[1])
    # A green zone of 1 needs a square more room on each side.
    assert c.frontier(wall, scanned, green=1)[0] == 18


def test_norway_is_searched_toward_the_frontier():
    search, log, seconds, closest = fly('2x2', seed='norway')
    steps = [line for line in log if 'blocked at row' in line]
    # East to the gap in the wall, then west along the one way out of the maze: never
    # further east, where nothing leads, and never on to the far west edge.
    assert len(steps) == search.sidesteps == 3
    assert ' east ' in steps[0] and all(' west ' in line for line in steps[1:])
    assert steps[0].startswith('Wide path') and all(line.startswith('Path') for line in steps[1:])
    assert search.plan is not None and search.plan.green == 0 and search.minutes < 3.3
    assert closest > 3.0
