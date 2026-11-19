"""The alternative scan strategies (mow4, swarm2x2, quadrant), the cover-then-check survey,
the recorded results, and how the launch files wire them. No ROS."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from milestone4_multivehicle import scan_results, strategies
from milestone4_multivehicle import scan_routes as s
from milestone4_multivehicle.mines import COLS, ROWS, MineSurvey
from test_multivehicle import api  # noqa: F401 (a fixture)

ROOT = Path(__file__).parents[1]
IDS = strategies.strategy_ids()


def covered(intervals, extent):
    """True when the union of (low, high) intervals covers [0, extent]."""
    reach = 0.0
    for low, high in sorted(intervals):
        if low > reach + 1e-9:
            return False
        reach = max(reach, high)
    return reach >= extent - 1e-9


# --- the three strategies offer a route per drone, written out as YAML -----------------

def test_there_are_three_named_strategies_each_with_a_checker():
    assert set(IDS) == {'mow4', 'swarm2x2', 'quadrant'}
    assert {i: strategies.STRATEGIES[i].checker for i in IDS} == {'mow4': 3, 'swarm2x2': 3, 'quadrant': 0}


@pytest.mark.parametrize('sid', IDS)
@pytest.mark.parametrize('drone', range(4))
def test_yaml_on_disk_matches_the_generator(sid, drone):
    assert (ROOT / f'resource/milestone4_{sid}_{drone}.yaml').read_text() == strategies.yaml_text(sid, drone)


@pytest.mark.parametrize('sid', IDS)
@pytest.mark.parametrize('drone', range(4))
def test_yaml_passes_the_launch_check(api, drone, sid):
    api('milestone4_multivehicle/fleet_launch.py').check_waypoints(
        str(ROOT / f'resource/milestone4_{sid}_{drone}.yaml'))


@pytest.mark.parametrize('sid', IDS)
@pytest.mark.parametrize('drone', range(4))
def test_every_route_takes_off_holds_and_stays_east_at_altitude(sid, drone):
    points = strategies.waypoints(sid, drone)
    assert len(points) >= 2
    assert all(p['yaw'] == pytest.approx(math.pi / 2, abs=1e-3) for p in points)
    assert all(p['pos'][2] == -s.ALTITUDE_M for p in points)
    assert points[0]['pos'] == [0., 0., -s.ALTITUDE_M] and points[0]['hold_s'] == s.TAKEOFF_HOLD_S
    assert all(p['hold_s'] == 0. for p in points[1:-1]) and points[-1]['hold_s'] == s.FINAL_HOLD_S


def test_mow4_splits_the_nine_rows_three_two_two_two_across_four_drones():
    groups = strategies._mow4_groups()
    assert [len(g) for g in groups] == [3, 2, 2, 2]
    assert sorted(e for group in groups for e in group) == s.row_eastings()
    # Every drone mows the full height of the field and lands north of it.
    for drone in range(4):
        route = strategies.STRATEGIES['mow4'].world_route(drone)
        assert route[-1][1] == pytest.approx(s.LANDING_NORTH_M)


def test_swarm_two_columns_a_look_apart_sweep_the_whole_width():
    east = s.footprint()[0]
    columns = sorted({round(centre + off, 3)
                      for centre in strategies._swarm_band_centres()
                      for off, _ in strategies._SWARM_OFFSETS})
    assert covered([(c - east / 2, c + east / 2) for c in columns], s.FIELD_EAST_M)
    # The 2x2 block: two eastings and two northings, so each square is looked at twice.
    assert sorted({off[0] for off in strategies._SWARM_OFFSETS}) == [-strategies.BLOCK_HALF_EAST_M,
                                                                     strategies.BLOCK_HALF_EAST_M]
    assert sorted({off[1] for off in strategies._SWARM_OFFSETS}) == [0.0, strategies.BLOCK_RANK_M]


def test_quadrants_split_the_field_four_ways_and_the_checker_lands_south():
    strat = strategies.STRATEGIES['quadrant']
    mid = strategies._mid_north()
    for drone in (0, 1):  # south quadrants stay in the south half and land south of the field
        norths = [n for _, n in strat.world_route(drone)[:-1]]
        assert max(norths) <= mid + 0.1
        assert strat.world_route(drone)[-1][1] == pytest.approx(strategies.SOUTH_LAND_M)
    for drone in (2, 3):  # north quadrants cover the north half and land north of the field
        norths = [n for _, n in strat.world_route(drone)[:-1]]
        assert min(norths) >= mid - 0.1
        assert strat.world_route(drone)[-1][1] == pytest.approx(s.LANDING_NORTH_M)
    west, east = strategies._quadrant_halves()
    assert len(west) == 5 and len(east) == 4 and west + east == s.row_eastings()


# --- the cover-then-check survey config --------------------------------------------------

@pytest.mark.parametrize('sid', IDS)
def test_survey_config_has_all_four_cover_then_the_checker(sid):
    config = strategies.survey_config(sid)
    strat = strategies.STRATEGIES[sid]
    assert config['check_after_coverage'] and config['mowers'] == [0, 1, 2, 3]
    assert config['checker'] == strat.checker
    for drone in range(4):
        assert config['coverage_ends'][drone] == strat.world_route(drone)[-1]


# --- MineSurvey in cover-then-check mode -------------------------------------------------

def test_coverage_mode_waits_for_every_drone_before_the_checker_takes_over():
    config = strategies.survey_config('mow4')
    survey = MineSurvey(set(), 1.0, 1, 4, log=lambda *a: None, **config)
    ends = config['coverage_ends']
    assert not survey.scan_over and survey.covered == set()
    for drone in (0, 1, 2):
        east, north = ends[drone]
        survey.observe(drone, east, north, s.ALTITUDE_M, s.YAW, 1.0)
        assert not survey.scan_over
    # Drone 3 is the checker but still mows until it too reaches the end of its route.
    survey.observe(3, 2.0, 2.0, s.ALTITUDE_M, s.YAW, 1.0)
    assert not survey.scan_over and 3 not in survey.covered
    east, north = ends[3]
    survey.observe(3, east, north, s.ALTITUDE_M, s.YAW, 2.0)
    assert survey.scan_over and survey.covered == {0, 1, 2, 3}


def test_classic_survey_defaults_are_unchanged():
    survey = MineSurvey(set(), 1.0, 1, 4, log=lambda *a: None)
    assert survey.checker == 3 and list(survey.mowers) == [0, 1, 2]
    assert not survey.check_after_coverage and survey.coverage_ends == {}
    # scan_over is still by row count, not by reaching a point.
    assert not survey.scan_over
    survey.rows_flown = {0: s.ROWS_PER_DRONE, 1: s.ROWS_PER_DRONE, 2: s.ROWS_PER_DRONE}
    assert survey.scan_over


# --- recorded results --------------------------------------------------------------------

def test_results_round_trip_and_summary(tmp_path):
    path = tmp_path / 'scan_results.jsonl'
    assert scan_results.load(str(path)) == []
    scan_results.record(str(path), {'strategy': 'mow4', 'score': 10.0, 'mine_seed': 10})
    scan_results.record(str(path), {'strategy': 'quadrant', 'score': 25.0, 'mine_seed': 10})
    scan_results.record(str(path), {'strategy': 'mow4', 'score': 18.0, 'mine_seed': 11})
    rows = scan_results.load(str(path))
    assert [r['strategy'] for r in rows] == ['mow4', 'quadrant', 'mow4']
    assert all('time' in row for row in rows)
    summary = scan_results.summarize(rows)
    assert summary['runs'] == 3 and summary['best']['strategy'] == 'quadrant'
    assert summary['by_strategy']['mow4']['score'] == 18.0


def test_recording_to_an_impossible_path_is_swallowed(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('not a directory')
    assert scan_results.record(str(blocker / 'nested.jsonl'), {'score': 1.0}) is None


# --- the launch files wire the strategy, checker and results file ------------------------

def _strategy_context(sid, **overrides):
    routes = [str(ROOT / f'resource/milestone4_{sid}_{i}.yaml') for i in range(4)]
    values = {'topic_prefix': '', 'system_id': '1', 'vehicle_count': '4', 'avoidance': 'orca',
              'max_speed': '1.0', 'mines': '150', 'mine_seed': '10', 'mine_case': '',
              'detect_probability': '0.9', 'path_file': '/tmp/path.txt',
              'results_file': '/tmp/scan_results.jsonl'}
    for i in range(4):
        values.update({f'path_{i + 1}': routes[i], f'spawn_x_{i + 1}': str(s.SPAWNS[i][0]),
                       f'spawn_y_{i + 1}': str(s.SPAWNS[i][1])})
    return SimpleNamespace(is_shutdown=False, values={**values, **overrides})


@pytest.mark.parametrize('sid', IDS)
@pytest.mark.parametrize('instance', range(4))
def test_only_the_checker_runs_the_mine_field_and_gets_the_second_pass(api, instance, sid):
    checker = strategies.STRATEGIES[sid].checker
    context = _strategy_context(sid, topic_prefix='' if instance == 0 else f'/px4_{instance}',
                                system_id=str(instance + 1))
    fleet = api('milestone4_multivehicle/fleet_launch.py')
    actions = fleet.launch_vehicle(context, mines=True, search=sid)
    mine_fields = [a for a in actions if type(a).__name__ == 'Node' and a.executable == 'mine_field']
    assert bool(mine_fields) == (instance == checker)

    handlers = [a.args[0] for a in actions if type(a).__name__ == 'RegisterEventHandler']
    started = handlers[0].on_exit(SimpleNamespace(returncode=0), context)
    controller = next(a for a in started if type(a).__name__ == 'Node')
    assert controller.parameters[0]['second_pass'] == (instance == checker)
    assert controller.parameters[0]['route_topic'] == '/milestone4/best_path'
    if mine_fields:
        params = mine_fields[0].parameters[0]
        assert params['search'] == sid and params['results_file'] == '/tmp/scan_results.jsonl'
        assert params['vehicle_count'] == 4


@pytest.mark.parametrize('sid', IDS)
def test_strategy_launch_offers_results_and_mine_arguments(api, sid):
    launch = api(f'launch/{sid}_launch.py')
    declared = {a.args[0] if a.args else a.name: a
                for a in launch.generate_launch_description().args[0]
                if type(a).__name__ == 'DeclareLaunchArgument'}
    assert declared['results_file'].default_value == '~/launcher_runs/scan_results.jsonl'
    assert declared['mines'].default_value == '150'
    assert declared['path_1'].default_value.args[0][-1] == f'milestone4_{sid}_0.yaml'


@pytest.mark.parametrize('sid', IDS)
def test_a_strategy_needs_mines_and_four_vehicles(api, sid):
    fleet = api('milestone4_multivehicle/fleet_launch.py')
    with pytest.raises(ValueError):
        fleet.launch_vehicle(_strategy_context(sid, vehicle_count='2', system_id='1'),
                             mines=True, search=sid)
    with pytest.raises(ValueError):
        fleet.launch_vehicle(_strategy_context(sid, mines='0'), mines=True, search=sid)
