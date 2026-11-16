"""Simulated mines: placement, detection, the best path, and the second look. No ROS."""
import itertools
import math

import pytest

from milestone4_multivehicle import mines as m
from milestone4_multivehicle import scan_routes as s
from milestone4_multivehicle.orca_avoidance import OrcaAvoidance
from test_orca import position
from test_multivehicle import api, make_controller  # noqa: F401 (api is a fixture)

ROUTE_PATH = __file__.replace('test/test_mines.py', 'resource/milestone4_scan_3.yaml')


def test_mines_are_repeatable_on_the_field_and_clumped():
    mines = m.place_mines(150, 10)
    assert mines == m.place_mines(150, 10) != m.place_mines(150, 11)
    assert len(mines) == 150 and all(0 <= c < m.COLS and 0 <= r < m.ROWS for c, r in mines)
    touching = sum(any((c + dc, r + dr) in mines for dc in (-1, 0, 1) for dr in (-1, 0, 1) if dc or dr)
                   for c, r in mines)
    assert touching > 50  # scattered evenly, about 27 would have a neighbour
    assert len(m.place_mines(0, 1)) == 0 and len(m.place_mines(7000, 1)) == m.COLS * m.ROWS


def test_footprint_follows_height_and_heading():
    assert m.footprint(5.) == (3., 2.) and m.footprint(5.6) == pytest.approx((3.36, 2.24))
    east = math.pi / 2
    assert m.in_footprint((11.6, 20.), 10., 20., 5.6, east) and not m.in_footprint((11.7, 20.), 10., 20., 5.6, east)
    assert m.in_footprint((10., 21.1), 10., 20., 5.6, east) and not m.in_footprint((10., 21.2), 10., 20., 5.6, east)
    assert m.in_footprint((10., 21.6), 10., 20., 5.6, 0.) and not m.in_footprint((11.2, 20.), 10., 20., 5.6, 0.)
    assert not m.in_footprint((10., 20.), 10., 20., 2.9, east)  # too low to be scanning


def test_a_look_finds_a_mine_with_the_set_chance_and_once_per_pass():
    mines = {(c, r) for c in range(0, 40, 8) for r in range(0, 150, 5)}  # never two under one footprint
    for probability, low, high in ((1., 1., 1.), (0.9, .82, .97), (0.5, .38, .62)):
        detector = m.Detector(mines, probability, seed=3)
        for mine in sorted(mines):
            east, north = m.centre(mine)
            first = detector.observe(0, east, north, 5.6, math.pi / 2)
            assert detector.observe(0, east, north, 5.6, math.pi / 2) == []  # hovering gives no second chance
            assert first in ([], [mine])
        assert low <= len(detector.found) / len(mines) <= high
    detector = m.Detector({(5, 5)}, 0.5, seed=0)
    east, north = m.centre((5, 5))
    for _ in range(20):  # leaving and coming back is a new pass
        detector.observe(0, east, north, 5.6, 0.)
        detector.observe(0, east + 10, north, 5.6, 0.)
    assert detector.found == {(5, 5): 0}


def test_with_no_mines_the_best_path_is_straight_with_the_widest_green_zone():
    plan = m.plan_path(set())
    assert plan.green == m.MAX_GREEN and plan.steps == 150 and plan.runs() == [('U', 150)]
    assert plan.text() == f'S,{plan.cells[0][0]},{m.MAX_GREEN}\nU,150\n'
    assert (plan.width_ft, plan.length_ft) == (18, 300)
    assert plan.score(0) == pytest.approx(150000 * 18 / 300)
    assert plan.score(4, ounces_over=1) == pytest.approx(150000 * 18 / (300 * (1 + 28 + 100)))


@pytest.mark.parametrize('seed', range(8))
def test_best_path_is_valid_clear_and_no_worse_than_a_narrower_one(seed):
    mines = m.place_mines(150, seed)
    plan = m.plan_path(mines)
    assert plan.cells[0][1] == 0 and plan.cells[-1][1] == m.ROWS - 1
    assert len(set(plan.cells)) == len(plan.cells)
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1 for a, b in zip(plan.cells, plan.cells[1:]))
    assert not plan.corridor() & mines
    assert all(plan.green <= c <= m.COLS - 1 - plan.green for c, _ in plan.cells)
    assert sum(n for _, n in plan.runs()) == plan.steps
    for green in range(m.MAX_GREEN + 1):
        other = m.plan_path(mines, max_green=green)
        assert other.green <= green and plan.score(4) >= other.score(4) - 1e-9


def test_score_counts_real_mines_on_the_path_and_in_the_green_zone():
    plan = m.Plan([(10, r) for r in range(150)], 2)
    assert plan.score(1, {(10, 40)}) == 0.
    clear = plan.score(1)
    assert plan.score(1, {(12, 40), (8, 90), (13, 5)}) == pytest.approx(clear / 3)


def test_a_wall_across_the_field_leaves_no_path():
    assert m.plan_path({(c, 70) for c in range(m.COLS)}) is None


def test_a_partial_path_stays_on_scanned_ground_and_stops_where_it_does():
    strip = {(c, r) for c in range(13, 19) for r in range(0, 60)}
    plan = m.plan_route(set(), strip)
    assert not plan.complete and plan.green == 2 and plan.runs() == [('U', 56)]  # ends on a multiple of 5 rows
    assert plan.corridor() <= strip and plan.score(1) == 0.
    assert m.plan_route({(15, 30), (16, 30)}, strip).corridor() <= strip - {(15, 30), (16, 30)}
    assert m.plan_route(set(), set()) is None
    # Two strips reaching equally far: it takes the one the checker is over, or has checked.
    two = strip | {(c, r) for c in range(28, 34) for r in range(0, 60)}
    assert m.plan_route(set(), two, near_col=14).cells[0][0] == 15
    assert m.plan_route(set(), two, near_col=35).cells[0][0] == 31
    checked = {(c, r) for c in range(29, 34) for r in range(0, 40)}
    assert m.plan_route(set(), two, checked, near_col=14).cells[0][0] == 30
    # A complete path beats a wider partial one, and the width in use is kept if it is close.
    whole = strip | {(20, r) for r in range(150)}
    assert m.plan_route(set(), whole).complete and m.plan_route(set(), whole).green == 0
    assert m.plan_route(set(), None, keep_green=4).green == 4
    wide = {(c, r) for c in range(10, 17) for r in range(150)}
    assert m.plan_route(set(), wide).green == 3 and m.plan_route(set(), wide, keep_green=1).green == 3


def test_cells_under_the_camera_and_the_checking_height():
    under = m.cells_under(10., 20., 5.6, math.pi / 2)
    assert under == {(c, r) for c in range(14, 19) for r in range(31, 35)}
    assert m.cells_under(10., 20., 2., 0.) == set()
    straight = m.Plan([(6, r) for r in range(150)], 2)
    assert m.pass_altitude(straight) == 5.6 and m.pass_altitude(m.Plan(straight.cells, 4)) == 9.64
    bent = m.Plan([(6, r) for r in range(50)] + [(c, 49) for c in range(7, 12)] + [(11, r) for r in range(50, 150)], 2)
    # An east-west leg needs the footprint's 2 m side to span the 5-square corridor.
    assert m.pass_altitude(bent) == 8.37 and m.pass_altitude(m.Plan(bent.cells, 4)) == m.MAX_PASS_ALTITUDE_M
    route = m.stretch_route(bent.cells, 8.37)
    assert [p[:2] for p in route] == [pytest.approx(p) for p in (
        (3.962, 1.), (3.962, 30.175), (7.01, 30.175), (7.01, 90.44))]
    assert all(p[2:] == (8.37, s.YAW) for p in route)
    assert [p[:2] for p in m.stretch_route(bent.cells[::-1][100:110], 5.6)] == [(7.01, 30.175), (3.962, 30.175), (3.962, 27.737)]


def test_route_message_round_trips_and_rejects_rubbish():
    survey = m.MineSurvey(set(), log=lambda _: None)
    assert m.read_message(survey.message()) == (0, False, [])
    survey.route, survey.version = m.stretch_route([(5, r) for r in range(40)], 5.6), 3
    assert m.read_message(survey.message()) == (3, False, survey.route)
    assert m.read_message([1., 0.]) is None
    assert m.read_message(survey.message()[:-1]) is None
    assert m.read_message([1., 0., 0.]) is None  # a numbered route must have points
    assert m.read_message([1., 0., 1., math.nan, 0., 0., 0.]) is None


def fly_survey(seed, probability, limit_s=900.):
    """The whole flight, kinematically: three drones mow under ORCA while the survey sends
    the checker along the path. Returns (survey, log, seconds, closest pair, checker's metres)."""
    log = []
    mines = m.place_mines(150, seed)
    survey = m.MineSurvey(mines, probability, seed, log=log.append)
    routes = [[(s.SPAWNS[d][0] + p['pos'][1], s.SPAWNS[d][1] + p['pos'][0], -p['pos'][2], p['hold_s'])
               for p in s.waypoints(d)] for d in range(4)]
    fleet = [OrcaAvoidance(d, *s.SPAWNS[d], vehicle_count=4, max_speed=1.) for d in range(4)]
    local, vel = [[0., 0.] for _ in range(4)], [[0., 0.] for _ in range(4)]
    index, hold_since, done = [1, 1, 1, 0], [None] * 4, [False] * 4
    taken, closest, flown, dt, now = 0, math.inf, 0., 0.1, 0.

    def way(d, point):
        return {'pos': (point[1] - s.SPAWNS[d][1], point[0] - s.SPAWNS[d][0], 0.), 'yaw': s.YAW, 'hold_s': 0.}

    while not all(done) and now < limit_s:
        pos = [position(x=local[d][0], y=local[d][1], vx=vel[d][0], vy=vel[d][1]) for d in range(4)]
        xy = [fleet[d].global_xy(pos[d]) for d in range(4)]
        for d in range(4):
            for a in fleet:
                a.receive(fleet[d].payload(pos[d]), now)
            # A landed drone is on the ground, so it sees nothing.
            survey.observe(d, *xy[d], 0. if done[d] else routes[d][index[d]][2], s.YAW, now)
        version, final, points = m.read_message(survey.message())
        if version > taken:  # the controller's take_route
            routes[3][index[3]:] = [(*xy[3], points[0][2], 0.)] + [(e, n, up, 0.) for e, n, up, _ in points]
            index[3] += 1
            taken = version
        for d in range(4):
            if done[d]:
                continue
            target = routes[d][index[d]]
            previous = way(d, routes[d][index[d] - 1]) if index[d] else None
            cmd = fleet[d].update(pos[d], now, way(d, target), previous)['vel']
            for axis in range(2):
                vel[d][axis] += (cmd[axis] - vel[d][axis]) * dt / .4
                local[d][axis] += vel[d][axis] * dt
            flown += math.hypot(*vel[d]) * dt if d == 3 else 0.
            if math.dist(xy[d], target[:2]) <= .5:
                hold_since[d] = now if hold_since[d] is None else hold_since[d]
                last = index[d] == len(routes[d]) - 1
                if now - hold_since[d] >= target[3] and not (last and d == 3 and not (final and taken == version)):
                    hold_since[d] = None
                    if last:
                        done[d], vel[d] = True, [0., 0.]
                    else:
                        index[d] += 1
            else:
                hold_since[d] = None
        for a, b in itertools.combinations(range(4), 2):
            closest = min(closest, math.dist(xy[a], xy[b]))
        now += dt
    assert all(done), f'unfinished after {now:.0f} s: waypoints {index}\n' + '\n'.join(log[-6:])
    return survey, log, now, closest, flown


def test_a_perfect_camera_ends_on_the_best_path_with_all_of_it_checked():
    survey, log, seconds, closest, flown = fly_survey(seed=4, probability=1.)
    assert survey.scan_over and len(survey.detector.found) == 150
    assert survey.final and survey.plan.complete
    assert survey.plan == m.plan_route(survey.mines, survey.scanned, survey.checked | survey.skipped,
                                       survey.plan.green, int(survey.checker_at[0] / m.SQUARE_M))
    corridor = survey.plan.corridor()
    assert not corridor & survey.mines and corridor <= survey.scanned
    assert len(corridor - survey.checked) <= 0.02 * len(corridor)  # all but the odd cut corner
    assert closest >= 2.7 and seconds < 600
    assert any(line.startswith('Path so far: S,') for line in log)  # it set off before the scan was over
    assert any(line.startswith('Final: S,') for line in log)


@pytest.mark.parametrize('seed', (10,))
def test_a_camera_that_misses_still_ends_on_a_path_clear_of_every_mine_it_found(seed):
    survey, log, seconds, closest, flown = fly_survey(seed=seed, probability=0.9)
    assert 120 <= len(survey.detector.found) < 150
    assert survey.final and survey.plan.complete
    assert not survey.plan.corridor() & set(survey.detector.found)
    assert survey.plan.corridor() <= survey.scanned
    assert closest >= 2.7


def test_the_checker_follows_each_new_route_from_where_it_is_and_lands_only_when_told(api):
    node = make_controller(api, 3, vehicle_count=4, avoidance='orca', second_pass=True,
                           waypoints_file=ROUTE_PATH, initial_global_x=23., initial_global_y=-3.)
    assert '/milestone4/best_path' in node.subscriptions

    def send(*data):
        node.on_route(type('M', (), {'data': [float(v) for v in data]})())

    node.confirmed_offboard = True
    node.current_position = position(x=0., y=0., z=-1.)
    send(1, 0, 2, 4., 1., 5.6, 1.5708, 4., 30., 5.6, 1.5708)
    node.take_route()
    assert node.route_version == 0 and len(node.waypoints) == 1  # still climbing out
    node.current_position = position(x=0., y=0., z=-5.6)
    send(0, 0, 0)
    node.take_route()
    assert node.route_version == 0  # the survey has nothing for it yet
    send(1, 0, 2, 4., 1., 5.6, 1.5708, 4., 30., 5.6, 1.5708)
    node.take_route()
    assert node.route_version == 1 and node.waypoint_index == 1 and len(node.waypoints) == 3
    assert node.waypoints[0]['pos'] == (0., 0., -5.6)  # where it was: the leg starts here
    assert node.waypoints[1]['pos'] == pytest.approx((4., -19., -5.6))
    assert not node.route_final()
    node.current_position = position(x=20., y=-19., z=-5.6)
    send(2, 0, 2, 4., 40., 8.37, 1.5708, 10., 40., 8.37, 1.5708)
    node.take_route()
    assert node.route_version == 2 and node.waypoint_index == 2 and len(node.waypoints) == 4
    assert node.waypoints[1]['pos'] == (20., -19., -8.37) and node.waypoints[2]['pos'] == pytest.approx((43., -19., -8.37))
    send(3, 1, 2, 10.9, 90.4, 8.37, 1.5708, 10.9, 94.44, 8.37, 1.5708)
    assert not node.route_final()
    node.take_route()
    assert node.route_final() and node.waypoints[-1]['hold_s'] == 2. and node.waypoints[-2]['hold_s'] == 0.
    send(9, 1)
    assert node.route_final()  # a malformed message is ignored


def test_the_checker_waits_while_the_survey_talks_and_lands_when_it_goes_quiet(api):
    node = make_controller(api, 3, vehicle_count=4, avoidance='orca', second_pass=True,
                           waypoints_file=ROUTE_PATH, initial_global_x=23., initial_global_y=-3.)
    assert not node.survey_silent(int(5e9))
    assert not node.survey_silent(int(64e9))
    assert node.survey_silent(int(66e9))
    node.route_heard_ns = int(60e9)
    assert not node.survey_silent(int(100e9)) and node.survey_silent(int(121e9))


def test_scan_launch_offers_the_mine_arguments(api):
    launch = api('launch/scan_launch.py')
    def names(description):
        return {a.args[0] if a.args else a.name: a for a in description.args[0]
                if type(a).__name__ == 'DeclareLaunchArgument'}
    declared = names(launch.generate_launch_description())
    assert declared['mines'].default_value == '150'
    assert {'mine_seed', 'detect_probability', 'path_file'} <= declared.keys()
    plain = api('launch/collision_launch.py')
    assert 'mines' not in names(plain.generate_launch_description())


def test_stored_layouts_from_the_competition_site():
    from milestone4_multivehicle import layouts as stored
    assert len(stored.LAYOUTS['norway']) == 150 and len(stored.SITE_SEEDS) == 9
    assert all(len(stored.LAYOUTS[name]) == 135 for name in stored.SITE_SEEDS)
    assert all(0 <= col < m.COLS and 0 <= row < m.ROWS for cells in stored.LAYOUTS.values() for col, row in cells)
    # A stored layout comes back whole, whatever the count; ROS hands 0.1934 over as a number.
    assert m.place_mines(10, 'norway') == set(stored.LAYOUTS['norway'])
    assert m.place_mines(150, 0.1934) == m.place_mines(150, '0.1934') == set(stored.LAYOUTS['0.1934'])
    assert stored.seed_value('Norway') == 'norway' and stored.seed_value(0.866) == '0.866'
    # Whole numbers still generate a field, the same one as before.
    assert stored.seed_value('10') == stored.seed_value(10.0) == 10
    assert m.place_mines(150, '10') == m.place_mines(150, 10)
    with pytest.raises(ValueError):
        stored.seed_value(0.5)
    with pytest.raises(ValueError):
        stored.seed_value('sweden')


def test_norway_has_one_way_through_one_square_wide():
    from milestone4_multivehicle import corridor
    from milestone4_multivehicle import layouts as stored
    wall = set(stored.LAYOUTS['norway'])
    assert corridor.reach_row(wall, green=0) == m.ROWS - 1
    assert corridor.reach_row(wall, green=1) < 70
