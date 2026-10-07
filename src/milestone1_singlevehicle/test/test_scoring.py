"""Unit tests for the Milestone 1 flight scorer, using made-up flight data."""
import math

import numpy as np
import pytest

from milestone1_singlevehicle import scoring

LIMITS = {'position_error_m': 0.5, 'overshoot_m': 0.5, 'settle_time_s': 3.0, 'cross_track_m': 1.0,
          'heading_error_deg': 5.0, 'hover_drift_m': 0.2, 'tilt_max_deg': 45.0}
THRESHOLDS = {'sitl': dict(LIMITS), 'hitl': dict(LIMITS)}

DT = 0.1


# --- Path file ---------------------------------------------------------------

def test_shipped_path_yaw_is_read_as_radians():
    import os
    path = os.path.join(os.path.dirname(__file__), '..', 'resource', 'milestone1_path.yaml')
    yaws = [w.yaw_deg for w in scoring.load_config(path).waypoints]
    assert yaws[14] == pytest.approx(90) and yaws[15] == pytest.approx(180) and yaws[18] == pytest.approx(-90)
    assert all(abs(y) <= 180 for y in yaws)


def test_parse_accepts_plain_and_full_waypoints():
    config = scoring.parse_config({
        'waypoints': [[0, 0, -5], {'pos': [5, 0, -5], 'yaw': math.pi / 2, 'hold_s': 5, 'tests': ['T3', 'Y1']}],
        'thresholds': THRESHOLDS,
    })
    plain, full = config.waypoints
    assert plain.pos.tolist() == [0, 0, -5] and plain.yaw_deg is None and plain.tests == []
    assert full.yaw_deg == pytest.approx(90) and full.hold_s == 5 and full.tests == ['T3', 'Y1']
    assert config.thresholds['hitl']['overshoot_m'] == 0.5


@pytest.mark.parametrize('data, message', [
    ({'waypoints': [[0, 0, -5]]}, 'thresholds'),
    ({'waypoints': [[0, 0, -5]], 'thresholds': {'sitl': LIMITS, 'hitl': {'overshoot_m': 1}}},
     'thresholds.hitl is missing'),
    ({'waypoints': [], 'thresholds': THRESHOLDS}, 'non-empty'),
    ({'waypoints': [[0, 0]], 'thresholds': THRESHOLDS}, '[x, y, z]'),
    ({'waypoints': [{'pos': [0, 0, -5], 'yaw_deg': 90}], 'thresholds': THRESHOLDS}, 'unknown key'),
    ({'waypoints': [{'pos': [0, 0, -5], 'hold_s': -1}], 'thresholds': THRESHOLDS}, 'negative'),
    ({'waypoints': [{'pos': [0, 0, -5], 'tests': 'T1'}], 'thresholds': THRESHOLDS}, 'list of test IDs'),
])
def test_parse_rejects_bad_files(data, message):
    with pytest.raises(ValueError, match=message.replace('[', r'\[').replace(']', r'\]')):
        scoring.parse_config(data)


def test_tilt_from_quaternion():
    level = [1.0, 0.0, 0.0, 0.0]
    rolled_30 = [math.cos(math.radians(15)), math.sin(math.radians(15)), 0.0, 0.0]
    yawed_90 = [math.cos(math.radians(45)), 0.0, 0.0, math.sin(math.radians(45))]
    np.testing.assert_allclose(scoring.tilt_from_quaternion([level, rolled_30, yawed_90]),
                               [0.0, 30.0, 0.0], atol=1e-6)


# --- One leg -----------------------------------------------------------------

def _leg_samples(points):
    t = np.arange(len(points)) * DT
    return t, np.array(points, dtype=float)


def test_leg_measures_known_values():
    # Fly north from (0,0) to (5,0) 0.3 m east of the line, arrive, overshoot
    # to 0.8 m past the target, then hover within ±0.05 m.
    points = [[2.0 * i * DT, 0.3, -5.0] for i in range(20)]   # t 0.0-1.9: approach
    points += [[4.7, 0.0, -5.0]]                                # t 2.0: arrival (0.3 m away)
    points += [[5.8, 0.0, -5.0]] * 5                            # t 2.1-2.5: overshoot 0.8 m
    points += [[5.0, 0.05 * (-1) ** i, -5.0] for i in range(50)]  # t 2.6-7.5: hover
    t, pos = _leg_samples(points)

    reached, m = scoring.leg_measures(t, pos, np.zeros(len(t)), np.full(len(t), 10.0),
                                      np.array([5.0, 0.0, -5.0]), None, np.array([0.0, 0.0, -5.0]))

    assert reached
    assert m['arrival_s'] == pytest.approx(2.0)
    assert m['cross_track_m'] == pytest.approx(0.3)
    assert m['overshoot_m'] == pytest.approx(0.8)
    assert m['settle_time_s'] == pytest.approx(0.6)
    assert m['position_error_m'] == pytest.approx(0.05)
    assert m['hover_drift_m'] == pytest.approx(0.05)
    assert m['tilt_max_deg'] == pytest.approx(10.0)
    assert m['heading_error_deg'] is None  # no yaw commanded


def test_leg_never_reached():
    t, pos = _leg_samples([[0.0, 0.0, -5.0]] * 30)
    reached, m = scoring.leg_measures(t, pos, np.zeros(30), np.zeros(30),
                                      np.array([5.0, 0.0, -5.0]), None, np.array([0.0, 0.0, -5.0]))
    assert not reached
    assert m['position_error_m'] == pytest.approx(5.0)
    assert scoring._check('T3', True, reached, m, LIMITS) == ('FAIL', ['not reached'])


def test_leg_without_hold_has_no_data():
    # Arrives on the last samples, as the node does today (it moves on immediately).
    t, pos = _leg_samples([[x, 0.0, -5.0] for x in np.linspace(0, 4.8, 30)])
    reached, m = scoring.leg_measures(t, pos, np.zeros(30), np.zeros(30),
                                      np.array([5.0, 0.0, -5.0]), None, np.array([0.0, 0.0, -5.0]))
    assert reached and m['overshoot_m'] is None and m['settle_time_s'] is None
    status, missing = scoring._check('T3', True, reached, m, LIMITS)
    assert status == 'NO DATA' and 'overshoot_m' in missing


def test_leg_never_settles():
    points = [[4.8, 0.0, -5.0]] + [[5.8, 0.0, -5.0]] * 20
    t, pos = _leg_samples(points)
    reached, m = scoring.leg_measures(t, pos, np.zeros(len(t)), np.zeros(len(t)),
                                      np.array([5.0, 0.0, -5.0]), None, np.array([0.0, 0.0, -5.0]))
    assert reached and math.isinf(m['settle_time_s'])
    assert scoring._check('T3', True, reached, m, LIMITS)[0] == 'FAIL'


def test_heading_error_after_turn():
    # Turn on the spot from 0° to 90°, then hold at 88°.
    n = 40
    heading = np.radians(np.r_[np.linspace(0, 88, 10), np.full(n - 10, 88.0)])
    t, pos = _leg_samples([[2.0, 2.0, -5.0]] * n)
    reached, m = scoring.leg_measures(t, pos, heading, np.zeros(n),
                                      np.array([2.0, 2.0, -5.0]), 90.0, np.array([2.0, 2.0, -5.0]))
    assert reached
    assert m['heading_error_deg'] == pytest.approx(2.0, abs=1e-4)
    assert scoring._check('Y1', True, reached, m, LIMITS)[0] == 'PASS'
    assert scoring._check('Y1', True, reached, m, {**LIMITS, 'heading_error_deg': 1.0})[0] == 'FAIL'


def test_check_limit_is_inclusive_and_incomplete_legs_are_not_scored():
    m = {'position_error_m': 0.5, 'overshoot_m': 0.5, 'settle_time_s': 3.0, 'cross_track_m': 1.0}
    assert scoring._check('T3', True, True, m, LIMITS) == ('PASS', [])
    assert scoring._check('T3', True, True, {**m, 'overshoot_m': 0.51}, LIMITS) == ('FAIL', ['overshoot_m'])
    assert scoring._check('T3', False, True, m, LIMITS)[0] == 'INCOMPLETE'
    assert scoring._check('V1', True, True, m, LIMITS)[0] == 'NOT SCORED'


# --- Whole flight ------------------------------------------------------------

PATH = scoring.parse_config({
    'waypoints': [
        {'pos': [0, 0, -5], 'hold_s': 5, 'tests': ['T1']},
        {'pos': [5, 0, -5], 'hold_s': 5, 'tests': ['T3']},
        {'pos': [5, 5, -5], 'hold_s': 5, 'tests': ['T5', 'H1']},
    ],
    'thresholds': THRESHOLDS,
})


def make_flight(targets, end='land', leg_s=8.0, extra_setpoint=None, failsafe=True):
    """A flight that arms into Offboard at t=1 s, flies straight to each target in
    the first 40% of its leg and hovers exactly on it for the rest, then ends with
    `end`: 'land' (Land mode, then disarm), or a switch into nav_state `end`,
    by failsafe or (failsafe=False) by the pilot."""
    pos_t, pos, sp_t, sp_pos = [], [], [], []
    current, t = np.array([0.0, 0.0, 0.0]), 1.0

    # PX4's own Hold setpoints before Offboard: these drift and must be ignored.
    for i in range(5):
        sp_t.append(0.2 * i)
        sp_pos.append([0.0, 0.0, -0.01 * i])

    setpoints = [np.array(target, dtype=float) for target in targets]
    if extra_setpoint is not None:
        setpoints.insert(1, np.array(extra_setpoint, dtype=float))
    for target in setpoints:
        for k in range(int(leg_s / DT)):
            fraction = min(1.0, k * DT / (0.4 * leg_s))
            pos_t.append(t + k * DT)
            pos.append(current + fraction * (target - current))
        for k in range(int(leg_s / 0.2)):
            sp_t.append(t + 0.2 * k)
            sp_pos.append(target)
        current, t = target, t + leg_s
    end_t = t

    status = [(0.0, 1, scoring.NAV_STATE_HOLD, False), (1.0, 2, scoring.NAV_STATE_OFFBOARD, False)]
    if end == 'land':
        status += [(end_t, 2, scoring.NAV_STATE_LAND, False), (end_t + 6.0, 1, scoring.NAV_STATE_LAND, False)]
    else:
        status += [(end_t, 2, end, failsafe)]
    status_t, arming, nav, failsafe = (np.array(column) for column in zip(*status))

    pos_t = np.array(pos_t)
    return scoring.FlightData(
        pos_t=pos_t, pos=np.array(pos), heading=np.zeros(len(pos_t)),
        att_t=pos_t, tilt_deg=np.full(len(pos_t), 12.0),
        sp_t=np.array(sp_t), sp_pos=np.array(sp_pos), sp_yaw=np.zeros(len(sp_t)),
        status_t=status_t, arming_state=arming, nav_state=nav, failsafe=failsafe.astype(bool))


def test_clean_flight_passes():
    report = scoring.score_flight(make_flight([w.pos for w in PATH.waypoints]), PATH, 'sitl')

    assert [leg.waypoint for leg in report.legs] == [0, 1, 2]
    assert all(leg.complete and leg.reached for leg in report.legs)
    assert report.legs[-1].ended_by == 'land'
    for row in ('T1', 'T3', 'T5', 'H1', 'R1', 'S1', 'L1'):
        assert report.rows[row].status == 'PASS', (row, report.rows[row])
    assert report.rows['F1'].status == 'NOT TESTED'
    assert report.overall == 'PASS'
    assert report.notes == []


def test_failsafe_into_hold_passes_f1():
    # Node stopped after waypoint 1: offboard-loss failsafe switches PX4 to Hold.
    flight = make_flight([w.pos for w in PATH.waypoints[:2]], end=scoring.NAV_STATE_HOLD)
    report = scoring.score_flight(flight, PATH, 'sitl')

    assert report.rows['F1'].status == 'PASS' and report.failsafe_mode == 'Hold'
    assert report.legs[-1].ended_by == 'failsafe' and not report.legs[-1].complete
    assert report.rows['T3'].status == 'INCOMPLETE'
    assert report.rows['T5'].status == 'NOT FLOWN'
    assert report.overall == 'PASS'


def test_failsafe_into_rtl_fails_f1():
    flight = make_flight([w.pos for w in PATH.waypoints[:2]], end=5)
    report = scoring.score_flight(flight, PATH, 'sitl')
    assert report.rows['F1'].status == 'FAIL' and report.failsafe_mode == 'Return (RTL)'
    assert report.overall == 'FAIL'


def test_pilot_takeover_is_noted_and_incomplete():
    flight = make_flight([w.pos for w in PATH.waypoints[:2]], end=2, failsafe=False)  # Position mode
    report = scoring.score_flight(flight, PATH, 'hitl')
    assert report.legs[-1].ended_by == 'mode_change'
    assert any('Position' in note and 'takeover' in note for note in report.notes)
    assert report.rows['F1'].status == 'NOT TESTED'
    assert report.overall == 'INCOMPLETE'


def test_setpoint_not_in_path_is_reported():
    flight = make_flight([w.pos for w in PATH.waypoints], extra_setpoint=[9, 9, -5])
    report = scoring.score_flight(flight, PATH, 'sitl')
    assert any('[9.0, 9.0, -5.0]' in note for note in report.notes)
    assert report.overall == 'INCOMPLETE'


def test_tilt_over_limit_fails_r1():
    flight = make_flight([w.pos for w in PATH.waypoints])
    flight.tilt_deg[len(flight.tilt_deg) // 2] = 50.0
    report = scoring.score_flight(flight, PATH, 'sitl')
    assert report.rows['R1'].status == 'FAIL' and report.overall == 'FAIL'


def test_report_formats():
    report = scoring.score_flight(make_flight([w.pos for w in PATH.waypoints]), PATH, 'hitl')
    meta = {'log': 'test.ulg', 'date': '2026-10-06', 'px4': 'PX4_SITL abc', 'path': 'path.yaml'}
    markdown = scoring.format_markdown(report, meta)
    assert '| Overall | **PASS** |' in markdown
    assert '| 2026-10-06 | HITL | PX4_SITL abc | path.yaml |' in markdown
    data = scoring.report_to_dict(report, meta)
    assert data['overall'] == 'PASS' and data['rows']['T1']['status'] == 'PASS'
